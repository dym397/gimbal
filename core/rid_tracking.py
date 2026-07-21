"""RID serial parsing, identity tracks, geodesy, and SORT association.

XP-MRID-04 frames use ``08 17 30`` + a two-byte little-endian length field +
UTF-8 JSON + ``3F 55``.  The length value includes its own two bytes, so the
JSON byte length is ``length_value - 2`` and the complete frame length is
``length_value + 5``.
"""

from __future__ import annotations

import json
import math
import threading
import time
import bisect
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment


EARTH_RADIUS_M = 6_371_008.8


def circular_error_deg(a, b):
    """Return the absolute shortest separation between two headings."""
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def signed_circular_error_deg(a, b):
    """Return signed shortest heading difference a-b in [-180, 180)."""
    return (float(a) - float(b) + 180.0) % 360.0 - 180.0


def horizontal_distance_and_bearing(lat1, lon1, lat2, lon2):
    """WGS-84-like spherical distance and initial true-north bearing."""
    lat1 = math.radians(float(lat1))
    lon1 = math.radians(float(lon1))
    lat2 = math.radians(float(lat2))
    lon2 = math.radians(float(lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    hav = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    hav = min(1.0, max(0.0, hav))
    distance_m = 2.0 * 6371008.8 * math.asin(math.sqrt(hav))

    y = math.sin(dlon) * math.cos(lat2)
    x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    )
    bearing_deg = math.degrees(math.atan2(y, x)) % 360.0
    return distance_m, bearing_deg


def elevation_from_relative_height(horizontal_distance_m, relative_height_m):
    """Legacy helper for an explicitly known station-relative height."""
    distance_m = _finite_float(horizontal_distance_m)
    height_m = _finite_float(relative_height_m)
    if distance_m is None or distance_m < 0.0 or height_m is None:
        return None
    # XP-MRID uses -1000 as an unavailable height sentinel in field captures.
    if height_m <= -999.0:
        return None
    return math.degrees(math.atan2(height_m, distance_m))


def elevation_from_altitudes(
    horizontal_distance_m,
    target_altitude_m,
    station_altitude_m,
):
    """Compute elevation from target/station altitudes in the same datum."""
    distance_m = _finite_float(horizontal_distance_m)
    target_altitude_m = _finite_float(target_altitude_m)
    station_altitude_m = _finite_float(station_altitude_m)
    if (
        distance_m is None
        or distance_m < 0.0
        or target_altitude_m is None
        or station_altitude_m is None
    ):
        return None
    # XP-MRID field captures use -1000 for unavailable absolute altitude.
    if target_altitude_m <= -999.0:
        return None
    return math.degrees(
        math.atan2(target_altitude_m - station_altitude_m, distance_m)
    )


class RIDStreamParser:
    """Strictly decode XP-MRID-04 frames from a split/sticky byte stream."""

    HEADER = b"\x08\x17\x30"
    TAIL = b"\x3f\x55"
    LENGTH_FIELD_SIZE = 2
    MAX_PROTOCOL_PAYLOAD_BYTES = 0xFFFF - LENGTH_FIELD_SIZE

    def __init__(
        self,
        max_buffer_bytes=1024 * 1024,
        max_payload_bytes=MAX_PROTOCOL_PAYLOAD_BYTES,
    ):
        self.buffer = bytearray()
        self.max_buffer_bytes = max(4096, int(max_buffer_bytes))
        self.max_payload_bytes = min(
            self.MAX_PROTOCOL_PAYLOAD_BYTES,
            max(1, int(max_payload_bytes)),
            self.max_buffer_bytes - 7,
        )

        # Compatibility counters retained for existing field logging.
        # discarded_bytes now means bytes discarded while resynchronizing,
        # not the normal frame wrapper bytes.
        self.discarded_bytes = 0
        self.decode_errors = 0
        self.valid_frames = 0
        self.header_errors = 0
        self.length_errors = 0
        self.tail_errors = 0
        self.truncated_frames = 0
        self.utf8_errors = 0
        self.json_errors = 0
        self.json_type_errors = 0
        self.last_feed_frames = []

    def _discard_for_resync(self, byte_count):
        byte_count = min(max(0, int(byte_count)), len(self.buffer))
        if byte_count:
            self.discarded_bytes += byte_count
            del self.buffer[:byte_count]

    def _discard_non_header_prefix(self):
        """Discard garbage but preserve a possible split header suffix."""
        if self.buffer.endswith(self.HEADER[:2]):
            keep = 2
        elif self.buffer.endswith(self.HEADER[:1]):
            keep = 1
        else:
            keep = 0
        discard_count = len(self.buffer) - keep
        if discard_count > 0:
            self.header_errors += 1
            self._discard_for_resync(discard_count)

    def _resync_after_bad_frame(self, fallback_discard):
        next_header = self.buffer.find(self.HEADER, 1)
        self._discard_for_resync(
            next_header if next_header >= 0 else fallback_discard
        )

    def feed(self, chunk):
        self.last_feed_frames = []
        if chunk:
            self.buffer.extend(bytes(chunk))
        results = []

        while True:
            start = self.buffer.find(self.HEADER)
            if start < 0:
                self._discard_non_header_prefix()
                break
            if start > 0:
                self.header_errors += 1
                self._discard_for_resync(start)

            if len(self.buffer) < 5:
                break

            length_value = int(self.buffer[3]) | (int(self.buffer[4]) << 8)
            payload_length = length_value - self.LENGTH_FIELD_SIZE
            if payload_length < 0 or payload_length > self.max_payload_bytes:
                self.length_errors += 1
                self._resync_after_bad_frame(len(self.HEADER))
                continue

            frame_length = length_value + len(self.HEADER) + len(self.TAIL)
            if len(self.buffer) < frame_length:
                # Raw header bytes cannot legally occur inside UTF-8 JSON, so
                # a later header proves that the current frame was truncated
                # or its length field was corrupted. Resynchronize promptly
                # instead of waiting for the corrupt declared length.
                next_header = self.buffer.find(self.HEADER, len(self.HEADER))
                if next_header >= 0:
                    self.truncated_frames += 1
                    self._discard_for_resync(next_header)
                    continue
                break

            payload_end = len(self.HEADER) + length_value
            if bytes(self.buffer[payload_end:frame_length]) != self.TAIL:
                self.tail_errors += 1
                self._resync_after_bad_frame(frame_length)
                continue

            raw = bytes(self.buffer[5:payload_end])
            frame_metadata = {
                "length_field": length_value,
                "payload_length": payload_length,
                "frame_length": frame_length,
            }
            del self.buffer[:frame_length]

            try:
                text = raw.decode("utf-8")
            except UnicodeDecodeError:
                self.utf8_errors += 1
                self.decode_errors += 1
                continue
            try:
                value = json.loads(text)
            except json.JSONDecodeError:
                self.json_errors += 1
                self.decode_errors += 1
                continue
            if not isinstance(value, dict):
                self.json_type_errors += 1
                continue

            self.valid_frames += 1
            self.last_feed_frames.append(frame_metadata)
            results.append(value)

        return results


def _finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def is_valid_rid_position(longitude, latitude):
    """Return False for malformed coordinates and the RID 0/0 sentinel."""
    longitude = _finite_float(longitude)
    latitude = _finite_float(latitude)
    if (
        longitude is None
        or latitude is None
        or not (-180.0 <= longitude <= 180.0)
        or not (-90.0 <= latitude <= 90.0)
    ):
        return False
    return not (longitude == 0.0 and latitude == 0.0)


@dataclass
class RIDTrack:
    key: tuple
    rid_id: str
    id_type: int
    standard: str
    ui_id: int
    first_receive_ts: float
    last_receive_ts: float
    update_seq: int = 0
    update_count: int = 0
    measurement_seq: int = 0
    measurement_count: int = 0
    duplicate_count: int = 0
    invalid_position_count: int = 0
    last_changed_ts: float = 0.0
    signature: tuple | None = None
    data: dict = field(default_factory=dict)
    measurement_history: list = field(default_factory=list)

    def snapshot(self):
        item = dict(self.data)
        item.update({
            "key": self.key,
            "key_text": "|".join(str(part) for part in self.key),
            "rid_id": self.rid_id,
            "id_type": self.id_type,
            "standard": self.standard,
            "ui_id": self.ui_id,
            "first_receive_ts": self.first_receive_ts,
            "last_receive_ts": self.last_receive_ts,
            "last_changed_ts": self.last_changed_ts,
            "update_seq": self.update_seq,
            "update_count": self.update_count,
            "measurement_seq": self.measurement_seq,
            "measurement_count": self.measurement_count,
            "duplicate_count": self.duplicate_count,
            "invalid_position_count": self.invalid_position_count,
            "measurement_history": [
                dict(sample) for sample in self.measurement_history
            ],
        })
        return item


class RIDTrackManager:
    """Maintain RID identities directly by standard, ID type, and RID ID."""

    def __init__(self, track_ttl_s=5.0, history_size=64, delete_after_s=300.0):
        self.track_ttl_s = max(0.1, float(track_ttl_s))
        self.delete_after_s = max(self.track_ttl_s, float(delete_after_s))
        self.history_size = max(4, int(history_size))
        self.lock = threading.Lock()
        self.tracks = {}
        self.next_ui_id = 1
        self.global_update_seq = 0
        self.global_measurement_seq = 0

    def _delete_expired_locked(self, now_ts):
        expired = []
        for key, track in list(self.tracks.items()):
            age_s = max(0.0, float(now_ts) - float(track.last_receive_ts))
            if age_s < self.delete_after_s:
                continue
            item = track.snapshot()
            item["expired_age_s"] = age_s
            expired.append(item)
            del self.tracks[key]
        return expired

    def prune_expired(self, now_ts=None):
        """Permanently delete tracks silent for the configured retention time."""
        now_ts = time.time() if now_ts is None else float(now_ts)
        with self.lock:
            return self._delete_expired_locked(now_ts)

    def update_payload(self, payload, receive_ts=None):
        receive_ts = time.time() if receive_ts is None else float(receive_ts)
        uav = payload.get("UAVInfo") if isinstance(payload, dict) else None
        if not isinstance(uav, dict):
            return {"accepted": False, "reason": "missing_UAVInfo"}

        rid_id = str(uav.get("ID", "")).strip()
        if not rid_id:
            return {"accepted": False, "reason": "missing_ID"}
        standard = str(uav.get("RID_Standard", "")).strip() or "unknown"
        try:
            id_type = int(uav.get("ID_Type", 0))
        except (TypeError, ValueError):
            id_type = 0
        longitude = _finite_float(uav.get("Lon"))
        latitude = _finite_float(uav.get("Lat"))
        if (
            longitude is None
            or latitude is None
            or not (-180.0 <= longitude <= 180.0)
            or not (-90.0 <= latitude <= 90.0)
        ):
            return {
                "accepted": False,
                "reason": "invalid_UAV_coordinate",
                "rid_id": rid_id,
            }
        position_valid = is_valid_rid_position(longitude, latitude)

        operator = payload.get("OperatorInfo")
        operator = operator if isinstance(operator, dict) else {}
        monitor = payload.get("MonitorInfo")
        monitor = monitor if isinstance(monitor, dict) else {}
        key = (standard, id_type, rid_id)
        data = {
            "longitude": longitude,
            "latitude": latitude,
            "position_valid": position_valid,
            "alt_geo": _finite_float(uav.get("AltGeo")),
            "height": _finite_float(uav.get("Height")),
            "alt_baro": _finite_float(uav.get("AltBaro")),
            "horizontal_speed": _finite_float(uav.get("H_Speed")),
            "vertical_speed": _finite_float(uav.get("V_Speed")),
            "track_heading": _finite_float(uav.get("Trk")),
            "status": uav.get("Sta"),
            "uav_type": uav.get("Type"),
            "registration": str(uav.get("Reg", "")),
            "rid_timestamp": uav.get("T_Stamp"),
            "operator_longitude": _finite_float(operator.get("Lon")),
            "operator_latitude": _finite_float(operator.get("Lat")),
            "operator_height": _finite_float(operator.get("Height")),
            "monitor_name": str(monitor.get("Name", "")),
            "monitor_sn": str(monitor.get("SN", "")),
            "monitor_channel": monitor.get("Ch"),
            "monitor_temperature": monitor.get("Temp"),
        }
        signature = (
            data["rid_timestamp"],
            longitude,
            latitude,
            data["alt_geo"],
            data["height"],
            data["horizontal_speed"],
            data["vertical_speed"],
            data["status"],
            position_valid,
        )

        with self.lock:
            expired_tracks = self._delete_expired_locked(receive_ts)
            same_identity_expired = any(
                item.get("key") == key for item in expired_tracks
            )
            track = self.tracks.get(key)
            if track is None:
                track = RIDTrack(
                    key=key,
                    rid_id=rid_id,
                    id_type=id_type,
                    standard=standard,
                    ui_id=self.next_ui_id,
                    first_receive_ts=receive_ts,
                    last_receive_ts=receive_ts,
                    last_changed_ts=receive_ts,
                )
                self.next_ui_id += 1
                self.tracks[key] = track
                is_new = True
            else:
                is_new = False

            self.global_update_seq += 1
            duplicate = track.signature == signature
            previous_position_valid = bool(track.data.get("position_valid", False))
            track.last_receive_ts = receive_ts
            track.update_seq = self.global_update_seq
            track.update_count += 1
            if not position_valid:
                track.invalid_position_count += 1
            if duplicate:
                track.duplicate_count += 1
            elif position_valid:
                self.global_measurement_seq += 1
                track.measurement_seq = self.global_measurement_seq
                track.measurement_count += 1
                track.last_changed_ts = receive_ts
                track.measurement_history.append({
                    "measurement_seq": track.measurement_seq,
                    "timestamp": receive_ts,
                    "longitude": longitude,
                    "latitude": latitude,
                    "alt_geo": data["alt_geo"],
                    "horizontal_speed": data["horizontal_speed"],
                    "track_heading": data["track_heading"],
                })
                if len(track.measurement_history) > self.history_size:
                    del track.measurement_history[:-self.history_size]
            track.signature = signature
            track.data = data
            snapshot = track.snapshot()

        if not position_valid:
            reason = (
                "new_track_position_invalid_zero"
                if is_new
                else (
                    "duplicate_position_invalid_zero"
                    if duplicate
                    else "update_position_invalid_zero"
                )
            )
        elif not is_new and not previous_position_valid:
            reason = "position_acquired"
        else:
            reason = (
                "new_track_after_expiry"
                if is_new and same_identity_expired
                else ("new_track" if is_new else ("duplicate" if duplicate else "update"))
            )

        return {
            "accepted": True,
            "reason": reason,
            "is_new": is_new,
            "duplicate": duplicate,
            "position_valid": position_valid,
            "track": snapshot,
            "expired_tracks": expired_tracks,
        }

    def snapshot(self, now_ts=None, include_stale=False):
        now_ts = time.time() if now_ts is None else float(now_ts)
        with self.lock:
            self._delete_expired_locked(now_ts)
            values = [track.snapshot() for track in self.tracks.values()]
        for item in values:
            item["age_s"] = max(0.0, now_ts - float(item["last_receive_ts"]))
        if not include_stale:
            values = [item for item in values if item["age_s"] <= self.track_ttl_s]
        return sorted(values, key=lambda item: int(item["ui_id"]))


def enrich_rid_tracks(
    rid_tracks,
    station_latitude,
    station_longitude,
    station_altitude=None,
):
    enriched = []
    for item in rid_tracks:
        if not bool(item.get("position_valid", False)):
            continue
        distance_m, map_az = horizontal_distance_and_bearing(
            station_latitude,
            station_longitude,
            item["latitude"],
            item["longitude"],
        )
        value = dict(item)
        value["distance_m"] = distance_m
        value["map_az"] = map_az
        station_altitude_m = _finite_float(station_altitude)
        target_altitude_m = _finite_float(item.get("alt_geo"))
        value["station_altitude_m"] = station_altitude_m
        value["vertical_delta_m"] = (
            target_altitude_m - station_altitude_m
            if (
                target_altitude_m is not None
                and target_altitude_m > -999.0
                and station_altitude_m is not None
            )
            else None
        )
        value["elevation_deg"] = elevation_from_altitudes(
            distance_m,
            target_altitude_m,
            station_altitude_m,
        )
        trajectory_history = []
        for sample in item.get("measurement_history", ()):
            sample_distance, sample_map_az = horizontal_distance_and_bearing(
                station_latitude,
                station_longitude,
                sample["latitude"],
                sample["longitude"],
            )
            trajectory_history.append({
                "measurement_seq": int(sample["measurement_seq"]),
                "timestamp": float(sample["timestamp"]),
                "map_az": sample_map_az,
                "distance_m": sample_distance,
            })
        value["trajectory_history"] = trajectory_history
        enriched.append(value)
    return enriched


def _project_geodetic(latitude, longitude, reference_latitude):
    """Project nearby WGS-84 points into a stable local-metre plane."""
    latitude = math.radians(float(latitude))
    longitude = math.radians(float(longitude))
    scale = math.cos(math.radians(float(reference_latitude)))
    return (
        longitude * EARTH_RADIUS_M * scale,
        latitude * EARTH_RADIUS_M,
    )


def _unproject_geodetic(east_m, north_m, reference_latitude):
    scale = math.cos(math.radians(float(reference_latitude)))
    latitude = math.degrees(float(north_m) / EARTH_RADIUS_M)
    longitude = math.degrees(float(east_m) / (EARTH_RADIUS_M * scale))
    return latitude, longitude


def _rid_reported_velocity(sample, max_speed_mps):
    speed = _finite_float(sample.get("horizontal_speed"))
    heading = _finite_float(sample.get("track_heading"))
    if (
        speed is None
        or heading is None
        or speed < 0.0
        or speed > float(max_speed_mps)
        or not (0.0 <= heading < 360.0)
    ):
        return None
    heading_rad = math.radians(heading)
    return speed * math.sin(heading_rad), speed * math.cos(heading_rad)


class _RIDRenderTrack:
    """Causal alpha-beta state plus endpoint-bounded display history."""

    def __init__(self, key, ui_id, sample, reference_latitude, max_speed_mps):
        self.key = key
        self.ui_id = int(ui_id)
        self.reference_latitude = float(reference_latitude)
        self.max_speed_mps = float(max_speed_mps)
        east_m, north_m = _project_geodetic(
            sample["latitude"], sample["longitude"], self.reference_latitude
        )
        velocity = _rid_reported_velocity(sample, self.max_speed_mps)
        self.east_m = east_m
        self.north_m = north_m
        self.velocity_east_mps, self.velocity_north_mps = velocity or (0.0, 0.0)
        self.altitude_m = _finite_float(sample.get("alt_geo"))
        self.state_ts = float(sample["timestamp"])
        self.last_receive_ts = self.state_ts
        self.last_measurement_ts = self.state_ts
        self.last_measurement_seq = int(sample["measurement_seq"])
        self.measurements = [
            (
                self.state_ts,
                east_m,
                north_m,
                self.altitude_m,
            )
        ]
        self.display_east_m = east_m
        self.display_north_m = north_m
        self.display_initialized = True
        self.was_active = True
        self.last_update_mode = "new_track"

    def update(
        self,
        sample,
        active_ttl_s,
        alpha,
        beta,
        turn_reset_deg,
        history_points,
    ):
        measurement_seq = int(sample["measurement_seq"])
        if measurement_seq <= self.last_measurement_seq:
            return False

        timestamp = float(sample["timestamp"])
        east_m, north_m = _project_geodetic(
            sample["latitude"], sample["longitude"], self.reference_latitude
        )
        altitude_m = _finite_float(sample.get("alt_geo"))
        gap_s = timestamp - self.last_measurement_ts
        previous_measurement = self.measurements[-1]
        self.last_measurement_seq = measurement_seq
        self.last_measurement_ts = timestamp

        if gap_s <= 1.0e-6 or gap_s > float(active_ttl_s):
            velocity = _rid_reported_velocity(sample, self.max_speed_mps)
            self.east_m = east_m
            self.north_m = north_m
            self.velocity_east_mps, self.velocity_north_mps = velocity or (0.0, 0.0)
            self.altitude_m = altitude_m
            self.state_ts = timestamp
            self.measurements = [(timestamp, east_m, north_m, altitude_m)]
            self.display_east_m = east_m
            self.display_north_m = north_m
            self.display_initialized = True
            self.was_active = True
            self.last_update_mode = "reacquired"
            return True

        measured_velocity_east = (east_m - previous_measurement[1]) / gap_s
        measured_velocity_north = (north_m - previous_measurement[2]) / gap_s
        old_speed = math.hypot(self.velocity_east_mps, self.velocity_north_mps)
        measured_speed = math.hypot(
            measured_velocity_east, measured_velocity_north
        )
        turn_angle_deg = 0.0
        if old_speed >= 1.0 and measured_speed >= 1.0:
            cosine = (
                self.velocity_east_mps * measured_velocity_east
                + self.velocity_north_mps * measured_velocity_north
            ) / (old_speed * measured_speed)
            turn_angle_deg = math.degrees(
                math.acos(max(-1.0, min(1.0, cosine)))
            )

        self.measurements.append((timestamp, east_m, north_m, altitude_m))
        if len(self.measurements) > int(history_points):
            del self.measurements[:-int(history_points)]

        if turn_angle_deg > float(turn_reset_deg):
            self.east_m = east_m
            self.north_m = north_m
            reported_velocity = _rid_reported_velocity(
                sample, self.max_speed_mps
            )
            if reported_velocity is not None:
                self.velocity_east_mps, self.velocity_north_mps = reported_velocity
            else:
                self.velocity_east_mps = measured_velocity_east
                self.velocity_north_mps = measured_velocity_north
            self.altitude_m = altitude_m
            self.state_ts = timestamp
            self.last_update_mode = "turn_reset"
            return True

        predicted_east = self.east_m + self.velocity_east_mps * gap_s
        predicted_north = self.north_m + self.velocity_north_mps * gap_s
        residual_east = east_m - predicted_east
        residual_north = north_m - predicted_north
        self.east_m = predicted_east + float(alpha) * residual_east
        self.north_m = predicted_north + float(alpha) * residual_north
        self.velocity_east_mps += float(beta) * residual_east / gap_s
        self.velocity_north_mps += float(beta) * residual_north / gap_s

        reported_velocity = _rid_reported_velocity(sample, self.max_speed_mps)
        if reported_velocity is not None:
            self.velocity_east_mps = (
                0.75 * self.velocity_east_mps + 0.25 * reported_velocity[0]
            )
            self.velocity_north_mps = (
                0.75 * self.velocity_north_mps + 0.25 * reported_velocity[1]
            )
        speed = math.hypot(self.velocity_east_mps, self.velocity_north_mps)
        if speed > self.max_speed_mps:
            scale = self.max_speed_mps / speed
            self.velocity_east_mps *= scale
            self.velocity_north_mps *= scale
        if altitude_m is not None:
            self.altitude_m = (
                altitude_m
                if self.altitude_m is None
                else self.altitude_m + float(alpha) * (altitude_m - self.altitude_m)
            )
        self.state_ts = timestamp
        self.last_update_mode = "measurement"
        return True

    def render(
        self,
        now_ts,
        real_dt_s,
        render_delay_s,
        max_prediction_s,
        display_tau_s,
    ):
        render_ts = float(now_ts) - max(0.0, float(render_delay_s))
        measurement_times = [item[0] for item in self.measurements]
        right = bisect.bisect_right(measurement_times, render_ts)
        altitude_m = self.altitude_m
        prediction_age_s = 0.0

        if right == 0:
            _, desired_east, desired_north, altitude_m = self.measurements[0]
            render_mode = "hold_before_first"
        elif right < len(self.measurements):
            left_item = self.measurements[right - 1]
            right_item = self.measurements[right]
            duration = right_item[0] - left_item[0]
            ratio = (
                1.0
                if duration <= 1.0e-6
                else (render_ts - left_item[0]) / duration
            )
            ratio = max(0.0, min(1.0, ratio))
            desired_east = left_item[1] + (right_item[1] - left_item[1]) * ratio
            desired_north = left_item[2] + (right_item[2] - left_item[2]) * ratio
            if left_item[3] is not None and right_item[3] is not None:
                altitude_m = left_item[3] + (right_item[3] - left_item[3]) * ratio
            render_mode = "interpolate"
        else:
            latest_item = self.measurements[-1]
            unbounded_age_s = max(0.0, render_ts - latest_item[0])
            prediction_age_s = min(unbounded_age_s, max(0.0, float(max_prediction_s)))
            desired_east = latest_item[1] + self.velocity_east_mps * prediction_age_s
            desired_north = latest_item[2] + self.velocity_north_mps * prediction_age_s
            altitude_m = latest_item[3]
            render_mode = (
                "predict"
                if unbounded_age_s <= float(max_prediction_s)
                else "freeze"
            )

        if not self.was_active or not self.display_initialized:
            self.display_east_m = desired_east
            self.display_north_m = desired_north
            self.display_initialized = True
        else:
            gain = (
                1.0
                if float(display_tau_s) <= 0.0
                else 1.0
                - math.exp(-max(0.0, float(real_dt_s)) / float(display_tau_s))
            )
            self.display_east_m += gain * (desired_east - self.display_east_m)
            self.display_north_m += gain * (desired_north - self.display_north_m)
        self.was_active = True
        return {
            "east_m": self.display_east_m,
            "north_m": self.display_north_m,
            "altitude_m": altitude_m,
            "render_ts": render_ts,
            "render_mode": render_mode,
            "prediction_age_s": prediction_age_s,
            "filter_update_mode": self.last_update_mode,
        }


class RIDTrajectoryRenderer:
    """Turn sparse RID measurements into a bounded, causal UI trajectory.

    The renderer deliberately lags wall time so that most UI points lie between
    two measurements already received from the serial link.  It never sees a
    future RID frame.  When no right endpoint is available it predicts only for
    a short bounded interval and then freezes until the next measurement.
    """

    def __init__(
        self,
        render_delay_s=0.8,
        max_prediction_s=0.5,
        active_ttl_s=5.0,
        display_tau_s=0.2,
        alpha=0.85,
        beta=0.18,
        turn_reset_deg=90.0,
        history_points=12,
        max_speed_mps=40.0,
        delete_after_s=300.0,
    ):
        self.render_delay_s = max(0.0, float(render_delay_s))
        self.max_prediction_s = max(0.0, float(max_prediction_s))
        self.active_ttl_s = max(0.1, float(active_ttl_s))
        self.display_tau_s = max(0.0, float(display_tau_s))
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self.beta = max(0.0, float(beta))
        self.turn_reset_deg = max(0.0, min(180.0, float(turn_reset_deg)))
        self.history_points = max(2, int(history_points))
        self.max_speed_mps = max(0.1, float(max_speed_mps))
        self.delete_after_s = max(self.active_ttl_s, float(delete_after_s))
        self.tracks = {}
        self.last_render_ts = None

    def render(self, rid_tracks, now_ts=None):
        now_ts = time.time() if now_ts is None else float(now_ts)
        real_dt_s = (
            0.0
            if self.last_render_ts is None
            else max(0.0, now_ts - self.last_render_ts)
        )
        self.last_render_ts = now_ts
        rendered_tracks = []
        seen_keys = set()

        for item in rid_tracks:
            if not bool(item.get("position_valid", False)):
                continue
            key = item["key"]
            ui_id = int(item["ui_id"])
            samples = sorted(
                item.get("measurement_history") or (),
                key=lambda sample: int(sample["measurement_seq"]),
            )
            if not samples:
                continue

            state = self.tracks.get(key)
            if state is None or state.ui_id != ui_id:
                state = _RIDRenderTrack(
                    key,
                    ui_id,
                    samples[0],
                    reference_latitude=samples[0]["latitude"],
                    max_speed_mps=self.max_speed_mps,
                )
                self.tracks[key] = state
                samples = samples[1:]
            for sample in samples:
                state.update(
                    sample,
                    active_ttl_s=self.active_ttl_s,
                    alpha=self.alpha,
                    beta=self.beta,
                    turn_reset_deg=self.turn_reset_deg,
                    history_points=self.history_points,
                )

            state.last_receive_ts = float(item["last_receive_ts"])
            seen_keys.add(key)
            receive_age_s = max(0.0, now_ts - state.last_receive_ts)
            if receive_age_s > self.active_ttl_s:
                state.was_active = False
                continue

            rendered = state.render(
                now_ts,
                real_dt_s=real_dt_s,
                render_delay_s=self.render_delay_s,
                max_prediction_s=self.max_prediction_s,
                display_tau_s=self.display_tau_s,
            )
            latitude, longitude = _unproject_geodetic(
                rendered["east_m"],
                rendered["north_m"],
                state.reference_latitude,
            )
            value = dict(item)
            value["rid_raw_latitude"] = item.get("latitude")
            value["rid_raw_longitude"] = item.get("longitude")
            value["rid_raw_alt_geo"] = item.get("alt_geo")
            value["latitude"] = latitude
            value["longitude"] = longitude
            value["alt_geo"] = rendered["altitude_m"]
            value["rid_render_mode"] = rendered["render_mode"]
            value["rid_filter_update_mode"] = rendered["filter_update_mode"]
            value["rid_render_timestamp"] = rendered["render_ts"]
            value["rid_render_delay_s"] = self.render_delay_s
            value["rid_prediction_age_s"] = rendered["prediction_age_s"]
            rendered_tracks.append(value)

        for key, state in list(self.tracks.items()):
            if key not in seen_keys:
                state.was_active = False
            if (now_ts - state.last_receive_ts) >= self.delete_after_s:
                del self.tracks[key]

        return sorted(rendered_tracks, key=lambda item: int(item["ui_id"]))


class RIDSortAssociator:
    """Associate RID identities with SORT tracks using synchronized azimuth curves."""

    BLOCKED_COST = 1.0e6

    def __init__(
        self,
        max_az_error_deg=8.0,
        ambiguity_margin_deg=2.0,
        confirm_updates=3,
        hold_seconds=3.0,
        hold_max_az_error_deg=12.0,
        trajectory_points=10,
        min_trajectory_points=4,
        history_seconds=12.0,
        sync_tolerance_seconds=0.50,
        current_weight=0.35,
        curve_weight=0.40,
        trend_weight=0.25,
        max_curve_error_deg=None,
    ):
        self.max_az_error_deg = max(0.1, float(max_az_error_deg))
        self.ambiguity_margin_deg = max(0.0, float(ambiguity_margin_deg))
        self.confirm_updates = max(1, int(confirm_updates))
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.hold_max_az_error_deg = max(
            self.max_az_error_deg, float(hold_max_az_error_deg)
        )
        self.trajectory_points = max(2, int(trajectory_points))
        self.min_trajectory_points = min(
            self.trajectory_points, max(1, int(min_trajectory_points))
        )
        self.history_seconds = max(1.0, float(history_seconds))
        self.sync_tolerance_seconds = max(0.01, float(sync_tolerance_seconds))
        self.max_curve_error_deg = (
            self.max_az_error_deg
            if max_curve_error_deg is None
            else max(0.1, float(max_curve_error_deg))
        )
        weights = np.array(
            [current_weight, curve_weight, trend_weight], dtype=float
        )
        weights = np.maximum(weights, 0.0)
        if float(weights.sum()) <= 0.0:
            weights = np.array([1.0, 0.0, 0.0], dtype=float)
        weights /= weights.sum()
        self.current_weight = float(weights[0])
        self.curve_weight = float(weights[1])
        self.trend_weight = float(weights[2])

        self.confirmed = {}
        self.candidates = {}
        self.sort_histories = {}
        self.rid_histories = {}
        self.rid_last_measurement_seq = {}

    @staticmethod
    def _second_smallest(values):
        usable = sorted(
            float(value)
            for value in values
            if math.isfinite(value) and float(value) < RIDSortAssociator.BLOCKED_COST
        )
        return usable[1] if len(usable) >= 2 else math.inf

    @staticmethod
    def _append_timed_sample(history, sample):
        if history and abs(history[-1]["timestamp"] - sample["timestamp"]) < 1.0e-6:
            history[-1] = sample
        else:
            history.append(sample)

    def _prune_timed_history(self, history, now_ts):
        oldest = now_ts - self.history_seconds
        while history and history[0]["timestamp"] < oldest:
            history.pop(0)

    def _record_histories(self, sort_tracks, rid_tracks, now_ts):
        for sort_item in sort_tracks:
            sort_id = int(sort_item["track_id"])
            history = self.sort_histories.setdefault(sort_id, [])
            self._append_timed_sample(history, {
                "timestamp": now_ts,
                "map_az": float(sort_item["map_az"]) % 360.0,
            })
            self._prune_timed_history(history, now_ts)

        for rid_item in rid_tracks:
            rid_key = rid_item["key"]
            history = self.rid_histories.setdefault(rid_key, [])
            source_samples = list(rid_item.get("trajectory_history") or ())
            if not source_samples:
                source_samples = [{
                    "timestamp": float(
                        rid_item.get("last_changed_ts")
                        or rid_item.get("last_receive_ts")
                        or now_ts
                    ),
                    "measurement_seq": int(
                        rid_item.get("measurement_seq", rid_item["update_seq"])
                    ),
                    "map_az": float(rid_item["map_az"]),
                    "distance_m": float(rid_item["distance_m"]),
                }]
            last_consumed_seq = int(self.rid_last_measurement_seq.get(rid_key, 0))
            for source_sample in source_samples:
                measurement_seq = int(source_sample["measurement_seq"])
                if measurement_seq <= last_consumed_seq:
                    continue
                self._append_timed_sample(history, {
                    "timestamp": float(source_sample["timestamp"]),
                    "measurement_seq": measurement_seq,
                    "map_az": float(source_sample["map_az"]) % 360.0,
                    "distance_m": float(source_sample["distance_m"]),
                })
                last_consumed_seq = measurement_seq
            self._prune_timed_history(history, now_ts)
            self.rid_last_measurement_seq[rid_key] = last_consumed_seq

        for history in self.sort_histories.values():
            self._prune_timed_history(history, now_ts)
        for history in self.rid_histories.values():
            self._prune_timed_history(history, now_ts)

    def _interpolate_sort_azimuth(self, sort_id, sample_ts):
        history = self.sort_histories.get(int(sort_id), ())
        if not history:
            return None
        if sample_ts <= history[0]["timestamp"]:
            if history[0]["timestamp"] - sample_ts <= self.sync_tolerance_seconds:
                return float(history[0]["map_az"])
            return None
        if sample_ts >= history[-1]["timestamp"]:
            if sample_ts - history[-1]["timestamp"] <= self.sync_tolerance_seconds:
                return float(history[-1]["map_az"])
            return None

        for before, after in zip(history, history[1:]):
            if before["timestamp"] <= sample_ts <= after["timestamp"]:
                nearest_gap = min(
                    sample_ts - before["timestamp"],
                    after["timestamp"] - sample_ts,
                )
                if nearest_gap > self.sync_tolerance_seconds:
                    return None
                span = after["timestamp"] - before["timestamp"]
                if span <= 1.0e-9:
                    return float(after["map_az"])
                ratio = (sample_ts - before["timestamp"]) / span
                delta = signed_circular_error_deg(
                    after["map_az"], before["map_az"]
                )
                return (float(before["map_az"]) + ratio * delta) % 360.0
        return None

    def _trajectory_metrics(self, sort_item, rid_item):
        sort_id = int(sort_item["track_id"])
        rid_key = rid_item["key"]
        current_error = circular_error_deg(sort_item["map_az"], rid_item["map_az"])
        rid_history = self.rid_histories.get(rid_key, ())[-self.trajectory_points:]
        sort_history = self.sort_histories.get(sort_id, ())
        sort_start_ts = float(
            sort_item.get("created_ts")
            or (sort_history[0]["timestamp"] if sort_history else math.inf)
        )
        aligned = []
        for rid_sample in rid_history:
            if rid_sample["timestamp"] < sort_start_ts:
                continue
            sort_az = self._interpolate_sort_azimuth(
                sort_id, rid_sample["timestamp"]
            )
            if sort_az is None:
                continue
            aligned.append({
                "timestamp": rid_sample["timestamp"],
                "sort_map_az": sort_az,
                "rid_map_az": float(rid_sample["map_az"]),
                "residual_deg": signed_circular_error_deg(
                    sort_az, rid_sample["map_az"]
                ),
            })

        sample_count = len(aligned)
        if not aligned:
            return {
                "current_error_deg": current_error,
                "curve_error_deg": current_error,
                "shape_error_deg": 0.0,
                "trend_error_deg": 0.0,
                "curve_bias_deg": 0.0,
                "association_cost_deg": current_error,
                "trajectory_samples": 0,
                "trajectory_ready": False,
            }

        residuals = [sample["residual_deg"] for sample in aligned]
        curve_error = math.sqrt(
            sum(value * value for value in residuals) / sample_count
        )
        sin_sum = sum(math.sin(math.radians(value)) for value in residuals)
        cos_sum = sum(math.cos(math.radians(value)) for value in residuals)
        curve_bias = math.degrees(math.atan2(sin_sum, cos_sum))
        shape_residuals = [
            signed_circular_error_deg(value, curve_bias) for value in residuals
        ]
        shape_error = math.sqrt(
            sum(value * value for value in shape_residuals) / sample_count
        )

        trend_residuals = []
        for previous, current in zip(aligned, aligned[1:]):
            sort_delta = signed_circular_error_deg(
                current["sort_map_az"], previous["sort_map_az"]
            )
            rid_delta = signed_circular_error_deg(
                current["rid_map_az"], previous["rid_map_az"]
            )
            trend_residuals.append(
                signed_circular_error_deg(sort_delta, rid_delta)
            )
        trend_error = (
            math.sqrt(
                sum(value * value for value in trend_residuals)
                / len(trend_residuals)
            )
            if trend_residuals else 0.0
        )
        association_cost = (
            self.current_weight * current_error
            + self.curve_weight * curve_error
            + self.trend_weight * trend_error
        )
        return {
            "current_error_deg": current_error,
            "curve_error_deg": curve_error,
            "shape_error_deg": shape_error,
            "trend_error_deg": trend_error,
            "curve_bias_deg": curve_bias,
            "association_cost_deg": association_cost,
            "trajectory_samples": sample_count,
            "trajectory_ready": sample_count >= self.min_trajectory_points,
        }

    def _binding_payload(self, state, metrics, sort_item, rid_item):
        return {
            "state": state,
            "az_error_deg": metrics["current_error_deg"],
            "sort": sort_item,
            "rid": rid_item,
            **metrics,
        }

    def observe(self, sort_tracks, rid_tracks, now_ts=None):
        """Accumulate histories without running identity assignment."""
        now_ts = time.time() if now_ts is None else float(now_ts)
        self._record_histories(sort_tracks, rid_tracks, now_ts)

    def associate(self, sort_tracks, rid_tracks, now_ts=None):
        now_ts = time.time() if now_ts is None else float(now_ts)
        self._record_histories(sort_tracks, rid_tracks, now_ts)
        sort_by_id = {int(item["track_id"]): item for item in sort_tracks}
        rid_by_key = {item["key"]: item for item in rid_tracks}
        bindings = {}
        reserved_sort = set()
        reserved_rid = set()
        decisions = {}
        metrics_by_pair = {}

        # Keep established identities through short detection/RID gaps. If one
        # side is absent, reserve the visible counterpart so it cannot be
        # reassigned before the hold interval expires.
        for sort_id, state in list(self.confirmed.items()):
            rid_key = state["rid_key"]
            sort_item = sort_by_id.get(sort_id)
            rid_item = rid_by_key.get(rid_key)
            hold_alive = now_ts - state["last_good_ts"] <= self.hold_seconds
            if sort_item is None or rid_item is None:
                if not hold_alive:
                    del self.confirmed[sort_id]
                    continue
                if sort_item is None:
                    reserved_rid.add(rid_key)
                if rid_item is None:
                    reserved_sort.add(sort_id)
                continue

            metrics = self._trajectory_metrics(sort_item, rid_item)
            metrics_by_pair[(sort_id, rid_key)] = metrics
            curve_good = (
                not metrics["trajectory_ready"]
                or metrics["curve_error_deg"] <= self.hold_max_az_error_deg
            )
            if (
                metrics["current_error_deg"] <= self.hold_max_az_error_deg
                and curve_good
            ):
                state["last_good_ts"] = now_ts
                binding_state = "confirmed"
            elif hold_alive:
                binding_state = "held"
            else:
                del self.confirmed[sort_id]
                continue
            bindings[sort_id] = self._binding_payload(
                binding_state, metrics, sort_item, rid_item
            )
            reserved_sort.add(sort_id)
            reserved_rid.add(rid_key)
            decisions[(sort_id, rid_key)] = f"retain_{binding_state}"

        remaining_sort = [
            item for item in sort_tracks if int(item["track_id"]) not in reserved_sort
        ]
        remaining_rid = [
            item for item in rid_tracks if item["key"] not in reserved_rid
        ]
        selected_pairs = set()
        ambiguous_pairs = set()

        if remaining_sort and remaining_rid:
            raw_cost = np.zeros((len(remaining_sort), len(remaining_rid)), dtype=float)
            gated_cost = np.full_like(raw_cost, self.BLOCKED_COST)
            for row, sort_item in enumerate(remaining_sort):
                sort_id = int(sort_item["track_id"])
                for col, rid_item in enumerate(remaining_rid):
                    pair = (sort_id, rid_item["key"])
                    metrics = self._trajectory_metrics(sort_item, rid_item)
                    metrics_by_pair[pair] = metrics
                    raw_cost[row, col] = metrics["association_cost_deg"]
                    if (
                        metrics["trajectory_ready"]
                        and metrics["current_error_deg"] <= self.max_az_error_deg
                        and metrics["curve_error_deg"] <= self.max_curve_error_deg
                    ):
                        gated_cost[row, col] = metrics["association_cost_deg"]

            rows, cols = linear_sum_assignment(gated_cost)
            for row, col in zip(rows, cols):
                if gated_cost[row, col] >= self.BLOCKED_COST:
                    continue
                sort_item = remaining_sort[row]
                rid_item = remaining_rid[col]
                sort_id = int(sort_item["track_id"])
                rid_key = rid_item["key"]
                pair = (sort_id, rid_key)
                metrics = metrics_by_pair[pair]
                selected_pairs.add(pair)

                row_second = self._second_smallest(gated_cost[row, :])
                col_second = self._second_smallest(gated_cost[:, col])
                selected_cost = float(gated_cost[row, col])
                if (
                    row_second - selected_cost < self.ambiguity_margin_deg
                    or col_second - selected_cost < self.ambiguity_margin_deg
                ):
                    ambiguous_pairs.add(pair)
                    decisions[pair] = "ambiguous_trajectory"
                    self.candidates.pop(sort_id, None)
                    continue

                candidate = self.candidates.get(sort_id)
                if candidate is None or candidate["rid_key"] != rid_key:
                    candidate = {
                        "rid_key": rid_key,
                        "hits": 0,
                        "last_measurement_seq": None,
                        "first_seen_ts": now_ts,
                    }
                    self.candidates[sort_id] = candidate
                measurement_seq = int(
                    rid_item.get("measurement_seq", rid_item["update_seq"])
                )
                if candidate["last_measurement_seq"] != measurement_seq:
                    candidate["hits"] += 1
                    candidate["last_measurement_seq"] = measurement_seq
                candidate["last_seen_ts"] = now_ts
                decisions[pair] = (
                    f"candidate_{candidate['hits']}/{self.confirm_updates}"
                )
                if candidate["hits"] >= self.confirm_updates:
                    self.confirmed[sort_id] = {
                        "rid_key": rid_key,
                        "confirmed_ts": now_ts,
                        "last_good_ts": now_ts,
                    }
                    bindings[sort_id] = self._binding_payload(
                        "confirmed", metrics, sort_item, rid_item
                    )
                    reserved_sort.add(sort_id)
                    reserved_rid.add(rid_key)
                    decisions[pair] = "confirmed_now"
                    self.candidates.pop(sort_id, None)

        selected_sort_ids = {sort_id for sort_id, _ in selected_pairs}
        for sort_id in list(self.candidates):
            if sort_id not in selected_sort_ids:
                self.candidates.pop(sort_id, None)

        diagnostics = []
        for sort_item in sort_tracks:
            sort_id = int(sort_item["track_id"])
            for rid_item in rid_tracks:
                rid_key = rid_item["key"]
                pair = (sort_id, rid_key)
                metrics = metrics_by_pair.get(pair)
                if metrics is None:
                    metrics = self._trajectory_metrics(sort_item, rid_item)
                    metrics_by_pair[pair] = metrics
                if pair in decisions:
                    reason = decisions[pair]
                elif sort_id in reserved_sort or rid_key in reserved_rid:
                    reason = "reserved_by_confirmed_binding"
                elif metrics["current_error_deg"] > self.max_az_error_deg:
                    reason = "az_error_exceeds_gate"
                elif not metrics["trajectory_ready"]:
                    reason = (
                        "trajectory_warmup_"
                        f"{metrics['trajectory_samples']}/{self.min_trajectory_points}"
                    )
                elif metrics["curve_error_deg"] > self.max_curve_error_deg:
                    reason = "curve_error_exceeds_gate"
                else:
                    reason = "not_selected_by_assignment"
                diagnostics.append({
                    "sort_track_id": sort_id,
                    "rid_key": rid_key,
                    "rid_ui_id": int(rid_item["ui_id"]),
                    "rid_id": rid_item["rid_id"],
                    "sort_map_az": float(sort_item["map_az"]),
                    "rid_map_az": float(rid_item["map_az"]),
                    "az_error_deg": metrics["current_error_deg"],
                    "curve_error_deg": metrics["curve_error_deg"],
                    "shape_error_deg": metrics["shape_error_deg"],
                    "trend_error_deg": metrics["trend_error_deg"],
                    "curve_bias_deg": metrics["curve_bias_deg"],
                    "association_cost_deg": metrics["association_cost_deg"],
                    "trajectory_samples": metrics["trajectory_samples"],
                    "trajectory_ready": metrics["trajectory_ready"],
                    "max_az_error_deg": self.max_az_error_deg,
                    "max_curve_error_deg": self.max_curve_error_deg,
                    "selected": pair in selected_pairs or pair in decisions,
                    "ambiguous": pair in ambiguous_pairs,
                    "reason": reason,
                    "distance_m": float(rid_item["distance_m"]),
                    "rid_age_s": float(rid_item.get("age_s", 0.0)),
                    "rid_update_seq": int(rid_item["update_seq"]),
                    "rid_measurement_seq": int(
                        rid_item.get("measurement_seq", rid_item["update_seq"])
                    ),
                })

        return bindings, diagnostics
