import ast
from pathlib import Path


MAIN_PATH = Path(__file__).resolve().parents[1] / "core" / "main_tracking_v9.py"


def _read_calibration_constants():
    tree = ast.parse(MAIN_PATH.read_text(encoding="utf-8"))
    values = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if isinstance(target, ast.Name) and target.id in {
            "GIMBAL_AZ_BASE",
            "DEVICE_THETA",
        }:
            values[target.id] = ast.literal_eval(node.value)
    return values["GIMBAL_AZ_BASE"], values["DEVICE_THETA"]


def _control_azimuth(base, relative_azimuth):
    signed_relative = (
        relative_azimuth
        if relative_azimuth <= 180.0
        else relative_azimuth - 360.0
    )
    return base + signed_relative


def test_theta_table_preserves_base_shift_except_manual_layer3():
    old_horizontal = [
        33.4501, 17.7874, 1.0000, 344.8421, 327.6701,
        35.3086, 16.5593, 1.0759, 343.7710, 322.0000,
        32.9870, 20.6921, 1.9703, 340.9173, 323.7531,
        37.7386, 19.9455, 1.7703, 342.8870, 324.5302,
        38.5403, 19.5903, 1.7703, 341.7103, 322.7703,
        42.7703, 22.7703, 2.7703, 342.7703, 322.7703,
        46.2403, 25.0403, 3.8703, 341.6003, 320.4303,
        48.8703, 26.3703, 3.8703, 341.3703, 318.8703,
        62.6703, 38.6703, 14.6703, 350.6703, 326.6703,
    ]
    new_base, theta = _read_calibration_constants()

    assert new_base == 60.3
    assert sorted(theta) == list(range(1, 46))
    manual_layer3 = {
        11: (29.9870, 13.0),
        12: (18.6921, 13.0),
        13: (359.0000, 13.0),
        14: (337.9173, 13.0),
        15: (319.7531, 13.0),
    }
    assert theta[3]["theta_horizontal"] == 0.0

    for logic_id, old_azimuth in enumerate(old_horizontal, start=1):
        new_azimuth = theta[logic_id]["theta_horizontal"]
        assert 0.0 <= new_azimuth < 360.0
        if logic_id in manual_layer3:
            expected_azimuth, expected_vertical = manual_layer3[logic_id]
            assert new_azimuth == expected_azimuth
            assert theta[logic_id]["theta_vertical"] == expected_vertical
            continue
        assert abs(new_azimuth - ((old_azimuth - 1.0) % 360.0)) < 1e-9
        assert abs(
            _control_azimuth(59.3, old_azimuth)
            - _control_azimuth(new_base, new_azimuth)
        ) < 1e-9
