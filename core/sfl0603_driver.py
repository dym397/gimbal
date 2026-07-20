"""Reusable driver for the SFL0603 erbium-glass laser rangefinder.

Protocol source: ``SFL系列铒玻璃通讯协议V0.005.pdf``.

Safety defaults:

* Opening the serial port never starts ranging.
* ``stop_on_open=True`` sends command 0x00 and requires a valid standby reply.
* Laser-emitting commands are blocked until :meth:`arm_ranging` is called.
* Closing the driver attempts to stop continuous ranging before releasing the port.

The V0.005 revision history mentions APD gain set/query commands, but the body of
the supplied four-page document does not define their command bytes, payloads, or
responses.  The corresponding methods therefore fail explicitly instead of
guessing values that could be unsafe for the hardware.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Iterable, Optional, Union

import serial


STX = 0x55
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT_S = 1.0
MAX_PAYLOAD_LENGTH = 64
MIN_CONTINUOUS_PERIOD_MS = 100
MAX_CONTINUOUS_PERIOD_MS = 1000


class Command(IntEnum):
    STOP = 0x00
    SINGLE_RANGING = 0x01
    CONTINUOUS_RANGING = 0x02
    SELF_TEST = 0x03
    SET_NEAREST_DISTANCE = 0x04
    QUERY_SHOT_COUNT = 0x06
    SET_TARGET_MODE = 0x22
    SET_BAUDRATE = 0x26


class TargetMode(IntEnum):
    SINGLE = 0x0000
    TRIPLE = 0x0001
    FIRST_LAST = 0x0010


class RangingState(Enum):
    UNKNOWN = "unknown"
    STANDBY = "standby"
    CONTINUOUS = "continuous"


class SFL0603Error(Exception):
    """Base class for all driver errors."""


class ConnectionError(SFL0603Error):
    """The serial port could not be opened or is not open."""


class ProtocolError(SFL0603Error):
    """A malformed or unexpected protocol frame was received."""


class ChecksumError(ProtocolError):
    """A frame failed XOR checksum validation."""


class ResponseTimeoutError(ProtocolError):
    """No complete matching response was received before the timeout."""


class UnexpectedResponseError(ProtocolError):
    """A response did not contain the expected command or value."""


class RangingNotArmedError(SFL0603Error):
    """A laser-emitting command was blocked by the software interlock."""


class ProtocolDefinitionMissingError(SFL0603Error):
    """The supplied protocol revision does not define this operation."""


@dataclass(frozen=True)
class Frame:
    command: int
    data: bytes
    raw: bytes


@dataclass(frozen=True)
class Acknowledgement:
    command: Command
    value: int
    frame: Frame


@dataclass(frozen=True)
class MeasurementFlags:
    raw: int
    main_wave: bool
    echo: bool
    laser_ok: bool
    not_timeout: bool
    apd_ok: bool
    has_front_target: bool
    has_back_target: bool


@dataclass(frozen=True)
class Measurement:
    command: Command
    flags: MeasurementFlags
    target_1_m: float
    target_2_m: float
    target_3_m: float
    frame: Frame

    @property
    def targets_m(self) -> tuple[float, ...]:
        """Return reported non-zero targets, ordered from near to far."""
        return tuple(
            value
            for value in (self.target_1_m, self.target_2_m, self.target_3_m)
            if value > 0.0
        )

    @property
    def valid(self) -> bool:
        """Whether target 1 is usable and the documented health bits are normal."""
        return (
            self.flags.echo
            and self.flags.laser_ok
            and self.flags.not_timeout
            and self.flags.apd_ok
            and self.target_1_m > 0.0
        )


@dataclass(frozen=True)
class SelfTestResult:
    minus_5v_v: float
    nearest_distance_m: int
    apd_high_voltage_v: int
    apd_temperature_c: int
    plus_5v_v: float
    frame: Frame


def xor_checksum(data: Iterable[int]) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


def _require_u16(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{name} must be in range 0..65535")
    return value


def build_command_frame(command: Union[Command, int], value: int = 0) -> bytes:
    """Build the six-byte request frame defined by the protocol."""
    command_value = int(command)
    if not 0 <= command_value <= 0xFF:
        raise ValueError("command must be in range 0..255")
    value = _require_u16(value, "value")
    body = bytes((STX, command_value, 0x02, value >> 8, value & 0xFF))
    return body + bytes((xor_checksum(body),))


def parse_frame(raw: bytes) -> Frame:
    """Validate and parse one complete response frame."""
    if len(raw) < 4:
        raise ProtocolError(f"frame is too short: {len(raw)} bytes")
    if raw[0] != STX:
        raise ProtocolError(f"invalid STX: 0x{raw[0]:02X}")
    expected_length = 4 + raw[2]
    if len(raw) != expected_length:
        raise ProtocolError(
            f"frame length mismatch: header requires {expected_length}, got {len(raw)}"
        )
    expected_checksum = xor_checksum(raw[:-1])
    if raw[-1] != expected_checksum:
        raise ChecksumError(
            f"checksum mismatch: expected 0x{expected_checksum:02X}, "
            f"got 0x{raw[-1]:02X}"
        )
    return Frame(command=raw[1], data=raw[3:-1], raw=raw)


def _decode_u24be(data: bytes) -> int:
    if len(data) != 3:
        raise ValueError("u24 requires exactly three bytes")
    return int.from_bytes(data, byteorder="big", signed=False)


def _decode_i8(value: int) -> int:
    return value - 0x100 if value & 0x80 else value


class SFL0603:
    """Thread-safe SFL0603 serial driver.

    Parameters
    ----------
    port:
        Serial device name, for example ``COM15`` or ``/dev/ttyUSB0``.
    baudrate:
        Current module baudrate. The factory default is 115200 baud.
    timeout:
        Default response timeout in seconds.
    auto_open:
        Open the serial port during construction.
    stop_on_open:
        Require a valid stop acknowledgement immediately after opening. Keep this
        enabled in the system so startup cannot silently inherit continuous mode.
    allow_ranging:
        Initial state of the software ranging interlock. The safer default is
        ``False``; call :meth:`arm_ranging` only after the scene is safe.
    serial_instance:
        Optional pyserial-compatible object, primarily for tests or dependency
        injection. When supplied, ``port`` may be ``None``.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT_S,
        *,
        auto_open: bool = True,
        stop_on_open: bool = True,
        allow_ranging: bool = False,
        serial_instance=None,
    ) -> None:
        if timeout <= 0:
            raise ValueError("timeout must be greater than zero")
        if serial_instance is None and not port:
            raise ValueError("port is required when serial_instance is not supplied")

        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.stop_on_open = bool(stop_on_open)
        self._serial = serial_instance
        self._owns_serial = serial_instance is None
        self._lock = threading.RLock()
        self._ranging_armed = bool(allow_ranging)
        self._state = RangingState.UNKNOWN

        if auto_open:
            self.open()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(getattr(self._serial, "is_open", True))

    @property
    def ranging_armed(self) -> bool:
        return self._ranging_armed

    @property
    def state(self) -> RangingState:
        return self._state

    def open(self) -> None:
        """Open the port and, by default, require confirmed standby state."""
        with self._lock:
            if self.is_open and self._state is not RangingState.UNKNOWN:
                return

            if self._serial is None:
                try:
                    self._serial = serial.Serial(
                        port=self.port,
                        baudrate=self.baudrate,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=min(0.05, self.timeout),
                        write_timeout=self.timeout,
                    )
                except (OSError, serial.SerialException) as exc:
                    self._serial = None
                    raise ConnectionError(
                        f"failed to open SFL0603 serial port {self.port!r}: {exc}"
                    ) from exc
            elif not self.is_open and hasattr(self._serial, "open"):
                try:
                    self._serial.open()
                except Exception as exc:
                    raise ConnectionError(f"failed to open injected serial port: {exc}") from exc

            self._state = RangingState.UNKNOWN
            if self.stop_on_open:
                try:
                    self.stop_measurement()
                except Exception:
                    self._close_serial_only()
                    raise

    def close(self, *, raise_on_stop_error: bool = False) -> None:
        """Request standby and close the serial port.

        Shutdown normally prioritizes releasing resources, so a stop failure is
        suppressed unless ``raise_on_stop_error=True``. The state remains UNKNOWN
        when standby could not be confirmed.
        """
        with self._lock:
            stop_error = None
            if self.is_open:
                try:
                    self.stop_measurement()
                except Exception as exc:
                    self._state = RangingState.UNKNOWN
                    stop_error = exc
                finally:
                    self._close_serial_only()
            self._ranging_armed = False
            if stop_error is not None and raise_on_stop_error:
                raise stop_error

    def _close_serial_only(self) -> None:
        if self._serial is not None and self.is_open:
            try:
                self._serial.close()
            finally:
                self._state = RangingState.UNKNOWN

    def __enter__(self) -> "SFL0603":
        if not self.is_open:
            self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def arm_ranging(self) -> None:
        """Enable laser-emitting methods after the caller confirms a safe scene."""
        self._ranging_armed = True

    def disarm_ranging(self, *, stop: bool = True) -> None:
        """Block future ranging commands and optionally stop active measurement."""
        with self._lock:
            if stop and self.is_open:
                self.stop_measurement()
            self._ranging_armed = False

    def _require_open(self) -> None:
        if not self.is_open:
            raise ConnectionError("SFL0603 serial port is not open")

    def _require_ranging_armed(self) -> None:
        if not self._ranging_armed:
            raise RangingNotArmedError(
                "laser ranging is disarmed; call arm_ranging() only after confirming "
                "a safe target distance and scene"
            )

    def _write_command(self, command: Command, value: int = 0) -> bytes:
        self._require_open()
        raw = build_command_frame(command, value)
        try:
            written = self._serial.write(raw)
            if written is not None and written != len(raw):
                raise ConnectionError(f"serial write incomplete: {written}/{len(raw)} bytes")
            self._serial.flush()
        except (OSError, serial.SerialException) as exc:
            raise ConnectionError(f"failed to write SFL0603 command: {exc}") from exc
        return raw

    def _read_exact(self, size: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < size and time.monotonic() < deadline:
            try:
                chunk = self._serial.read(size - len(result))
            except (OSError, serial.SerialException) as exc:
                raise ConnectionError(f"failed to read SFL0603 response: {exc}") from exc
            if chunk:
                result.extend(chunk)
        return bytes(result)

    def _read_frame_until(self, deadline: float) -> Frame:
        self._require_open()
        while time.monotonic() < deadline:
            first = self._read_exact(1, deadline)
            if not first:
                break
            if first[0] != STX:
                continue

            header_tail = self._read_exact(2, deadline)
            if len(header_tail) != 2:
                break
            payload_length = header_tail[1]
            if payload_length > MAX_PAYLOAD_LENGTH:
                continue
            tail = self._read_exact(payload_length + 1, deadline)
            if len(tail) != payload_length + 1:
                break
            return parse_frame(first + header_tail + tail)
        raise ResponseTimeoutError("timed out waiting for a complete SFL0603 frame")

    def read_frame(self, timeout: Optional[float] = None) -> Frame:
        """Read one validated frame, resynchronizing on the 0x55 start byte."""
        wait = self.timeout if timeout is None else float(timeout)
        if wait <= 0:
            raise ValueError("timeout must be greater than zero")
        with self._lock:
            return self._read_frame_until(time.monotonic() + wait)

    def _wait_for_command(self, command: Command, timeout: Optional[float] = None) -> Frame:
        wait = self.timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            frame = self._read_frame_until(deadline)
            if frame.command == int(command):
                return frame
        raise ResponseTimeoutError(
            f"timed out waiting for SFL0603 command 0x{int(command):02X} response"
        )

    def _transact(self, command: Command, value: int = 0) -> Frame:
        with self._lock:
            self._write_command(command, value)
            return self._wait_for_command(command)

    @staticmethod
    def _decode_ack(frame: Frame, command: Command) -> Acknowledgement:
        if len(frame.data) != 2:
            raise UnexpectedResponseError(
                f"command 0x{int(command):02X} expected 2 data bytes, got {len(frame.data)}"
            )
        return Acknowledgement(
            command=command,
            value=int.from_bytes(frame.data, byteorder="big", signed=False),
            frame=frame,
        )

    @staticmethod
    def _require_echo(ack: Acknowledgement, expected: int) -> Acknowledgement:
        if ack.value != expected:
            raise UnexpectedResponseError(
                f"command 0x{int(ack.command):02X} echoed 0x{ack.value:04X}; "
                f"expected 0x{expected:04X}"
            )
        return ack

    def stop_measurement(self) -> Acknowledgement:
        """Stop continuous ranging and require the documented 0x00 echo."""
        with self._lock:
            try:
                frame = self._transact(Command.STOP)
                ack = self._require_echo(self._decode_ack(frame, Command.STOP), 0)
            except Exception:
                self._state = RangingState.UNKNOWN
                raise
            self._state = RangingState.STANDBY
            return ack

    def measure_once(self) -> Measurement:
        """Emit one laser pulse and return its parsed measurement."""
        self._require_ranging_armed()
        with self._lock:
            frame = self._transact(Command.SINGLE_RANGING)
            measurement = self._decode_measurement(frame)
            self._state = RangingState.STANDBY
            return measurement

    def start_continuous(self, period_ms: int = 1000) -> None:
        """Start periodic ranging; results are consumed with read_measurement()."""
        self._require_ranging_armed()
        period_ms = _require_u16(period_ms, "period_ms")
        if not MIN_CONTINUOUS_PERIOD_MS <= period_ms <= MAX_CONTINUOUS_PERIOD_MS:
            raise ValueError(
                f"period_ms must be {MIN_CONTINUOUS_PERIOD_MS}.."
                f"{MAX_CONTINUOUS_PERIOD_MS} for the SFL0603 (1..10 Hz)"
            )
        with self._lock:
            self._write_command(Command.CONTINUOUS_RANGING, period_ms)
            self._state = RangingState.CONTINUOUS

    def read_measurement(self, timeout: Optional[float] = None) -> Measurement:
        """Read the next single or continuous ranging response."""
        wait = self.timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + wait
        with self._lock:
            while time.monotonic() < deadline:
                frame = self._read_frame_until(deadline)
                if frame.command in (
                    int(Command.SINGLE_RANGING),
                    int(Command.CONTINUOUS_RANGING),
                ):
                    return self._decode_measurement(frame)
        raise ResponseTimeoutError("timed out waiting for an SFL0603 measurement")

    @staticmethod
    def _decode_measurement(frame: Frame) -> Measurement:
        if len(frame.data) != 10:
            raise UnexpectedResponseError(
                f"measurement expected 10 data bytes, got {len(frame.data)}"
            )
        try:
            command = Command(frame.command)
        except ValueError as exc:
            raise UnexpectedResponseError(
                f"unexpected measurement command 0x{frame.command:02X}"
            ) from exc
        if command not in (Command.SINGLE_RANGING, Command.CONTINUOUS_RANGING):
            raise UnexpectedResponseError(
                f"command 0x{frame.command:02X} is not a measurement response"
            )

        flag_byte = frame.data[0]
        flags = MeasurementFlags(
            raw=flag_byte,
            main_wave=bool(flag_byte & 0x80),
            echo=bool(flag_byte & 0x40),
            laser_ok=bool(flag_byte & 0x20),
            not_timeout=bool(flag_byte & 0x10),
            apd_ok=bool(flag_byte & 0x04),
            has_front_target=bool(flag_byte & 0x02),
            has_back_target=bool(flag_byte & 0x01),
        )
        distances_m = tuple(
            _decode_u24be(frame.data[offset : offset + 3]) / 10.0
            for offset in (1, 4, 7)
        )
        return Measurement(
            command=command,
            flags=flags,
            target_1_m=distances_m[0],
            target_2_m=distances_m[1],
            target_3_m=distances_m[2],
            frame=frame,
        )

    def self_test(self) -> SelfTestResult:
        """Run the non-ranging self-test and decode voltages, gate and APD data."""
        frame = self._transact(Command.SELF_TEST)
        if len(frame.data) != 8:
            raise UnexpectedResponseError(
                f"self-test expected 8 data bytes, got {len(frame.data)}"
            )
        return SelfTestResult(
            minus_5v_v=-int.from_bytes(frame.data[0:2], "big") * 0.01,
            nearest_distance_m=int.from_bytes(frame.data[2:4], "big"),
            apd_high_voltage_v=frame.data[4],
            apd_temperature_c=_decode_i8(frame.data[5]),
            plus_5v_v=int.from_bytes(frame.data[6:8], "big") * 0.01,
            frame=frame,
        )

    def set_nearest_distance(self, distance_m: int) -> Acknowledgement:
        """Set the nearest-distance gate in metres (stored across power cycles)."""
        distance_m = _require_u16(distance_m, "distance_m")
        frame = self._transact(Command.SET_NEAREST_DISTANCE, distance_m)
        return self._require_echo(
            self._decode_ack(frame, Command.SET_NEAREST_DISTANCE), distance_m
        )

    def query_shot_count(self) -> int:
        """Return the accumulated laser pulse count as an unsigned 32-bit value."""
        frame = self._transact(Command.QUERY_SHOT_COUNT)
        if len(frame.data) != 4:
            raise UnexpectedResponseError(
                f"shot-count response expected 4 data bytes, got {len(frame.data)}"
            )
        return int.from_bytes(frame.data, byteorder="big", signed=False)

    def set_target_mode(self, mode: Union[TargetMode, int]) -> Acknowledgement:
        """Select single-target, three-target, or first/last-target output."""
        try:
            target_mode = TargetMode(mode)
        except ValueError as exc:
            valid = ", ".join(f"0x{int(item):04X}" for item in TargetMode)
            raise ValueError(f"unsupported target mode; expected one of {valid}") from exc
        frame = self._transact(Command.SET_TARGET_MODE, int(target_mode))
        return self._require_echo(
            self._decode_ack(frame, Command.SET_TARGET_MODE), int(target_mode)
        )

    def set_baudrate(self, baudrate: int, *, reconfigure_port: bool = True) -> Acknowledgement:
        """Set the module baudrate and optionally update the local serial port.

        The protocol encodes ``baudrate * 0.01`` in a 16-bit field, so the baudrate
        must be divisible by 100. The document does not describe recovery if the
        reply is lost; use this command only when changing the factory setting is
        genuinely required.
        """
        if isinstance(baudrate, bool) or not isinstance(baudrate, int):
            raise TypeError("baudrate must be an integer")
        if baudrate <= 0 or baudrate % 100 != 0:
            raise ValueError("baudrate must be positive and divisible by 100")
        parameter = _require_u16(baudrate // 100, "baudrate / 100")
        with self._lock:
            frame = self._transact(Command.SET_BAUDRATE, parameter)
            ack = self._require_echo(
                self._decode_ack(frame, Command.SET_BAUDRATE), parameter
            )
            if reconfigure_port:
                self._serial.baudrate = baudrate
                self.baudrate = baudrate
            return ack

    # Compatibility methods matching the old SDDMLaser call shape used by the
    # gimbal system. New code should prefer measure_once/start_continuous.
    def start_measurement(self, continuous: bool = True, period_ms: int = 1000):
        if continuous:
            self.start_continuous(period_ms=period_ms)
            return None
        self._require_ranging_armed()
        with self._lock:
            self._write_command(Command.SINGLE_RANGING)
            self._state = RangingState.STANDBY
        return None

    def read_distance(self, debug: bool = False) -> Optional[float]:
        """Compatibility helper returning target 1 in metres, or None if invalid."""
        try:
            measurement = self.read_measurement()
        except ResponseTimeoutError:
            return None
        if debug:
            print(f"[SFL0603 RAW] {measurement.frame.raw.hex(' ')}")
        return measurement.target_1_m if measurement.valid else None

    def set_apd_gain_mode(self, mode: int):
        """Fail explicitly because V0.005 does not define this command in its body."""
        raise ProtocolDefinitionMissingError(
            "SFL protocol V0.005 mentions APD gain mode in the revision history, "
            "but does not define its command byte, payload, or response"
        )

    def query_apd_gain_mode(self):
        """Fail explicitly because V0.005 does not define this command in its body."""
        raise ProtocolDefinitionMissingError(
            "SFL protocol V0.005 mentions APD gain query in the revision history, "
            "but does not define its command byte, payload, or response"
        )


__all__ = [
    "Acknowledgement",
    "ChecksumError",
    "Command",
    "ConnectionError",
    "DEFAULT_BAUDRATE",
    "Frame",
    "Measurement",
    "MeasurementFlags",
    "ProtocolDefinitionMissingError",
    "ProtocolError",
    "RangingNotArmedError",
    "RangingState",
    "ResponseTimeoutError",
    "SFL0603",
    "SFL0603Error",
    "SelfTestResult",
    "TargetMode",
    "UnexpectedResponseError",
    "build_command_frame",
    "parse_frame",
    "xor_checksum",
]
