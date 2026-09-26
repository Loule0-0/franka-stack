from __future__ import annotations

import json
from pathlib import Path

from franka_runtime import PandaSafetyFilter
from franka_runtime import SafetyConfig
import numpy as np
import pytest

from examples.franka_real import collect_gello


def _observation(joints: np.ndarray) -> dict:
    return {
        "joint_positions": np.concatenate([joints, [0.0]]),
        "gripper_position": np.array([0.0]),
    }


def test_safe_command_limits_acceleration_and_records_shaped_target() -> None:
    initial = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.0])
    safety = PandaSafetyFilter(SafetyConfig(max_joint_step_rad=0.025, max_joint_acceleration_rad_s2=3.0))

    positive_target = np.concatenate([initial + 0.02, [0.5]])
    first = collect_gello._safe_command(positive_target, _observation(initial), safety, dt_s=0.05)  # noqa: SLF001
    np.testing.assert_allclose(first[:7] - initial, np.full(7, 0.0075), atol=1e-7)
    assert first[7] == 0.5

    negative_target = np.concatenate([initial - 0.02, [0.5]])
    second = collect_gello._safe_command(  # noqa: SLF001
        negative_target, _observation(first[:7]), safety, dt_s=0.05
    )
    np.testing.assert_allclose(second[:7], first[:7], atol=1e-7)


def test_safe_command_rejects_nonfinite_gello_target() -> None:
    initial = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.0])
    safety = PandaSafetyFilter()
    target = np.concatenate([initial, [0.0]])
    target[0] = np.nan

    with pytest.raises(ValueError, match="8 finite values"):
        collect_gello._safe_command(target, _observation(initial), safety, dt_s=0.05)  # noqa: SLF001


def test_deadman_release_stops_without_sending_a_hold_command() -> None:
    calls: list[tuple[str, object]] = []

    class Robot:
        def command_joint_state(self, target: np.ndarray) -> None:
            calls.append(("command", target))

        def stop(self, reason: str) -> None:
            calls.append(("stop", reason))

    collect_gello._stop_after_deadman_release(Robot())  # type: ignore[arg-type]  # noqa: SLF001

    assert calls == [("stop", "deadman released")]


def test_deadman_recomputes_pressed_state_after_dropped_release() -> None:
    class Device:
        def read(self):
            raise BlockingIOError

        def active_keys(self) -> list[int]:
            return []

    deadman = object.__new__(collect_gello.Deadman)
    deadman._device = Device()  # noqa: SLF001
    deadman._key_code = 57  # noqa: SLF001
    deadman.enabled = True

    assert deadman.poll() is False


def test_deadman_io_error_fails_closed() -> None:
    class Device:
        def read(self):
            return []

        def active_keys(self) -> list[int]:
            raise OSError("device disconnected")

    deadman = object.__new__(collect_gello.Deadman)
    deadman._device = Device()  # noqa: SLF001
    deadman._key_code = 57  # noqa: SLF001
    deadman.enabled = True

    assert deadman.poll() is False


def test_calibration_device_basename_requires_exact_match(tmp_path: Path) -> None:
    calibration = {
        "calibrated": True,
        "robot_model": "franka_panda",
        "gello_device_basename": "1",
        "joint_ids": [1, 2, 3, 4, 5, 6, 7],
        "joint_offsets": [0.0] * 7,
        "joint_signs": [1] * 7,
        "gripper_config": [8, 0.0, 1.0],
        "start_joints": [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0, 0.0],
        "calibrated_at": "2026-01-01T00:00:00+00:00",
    }
    calibration_path = tmp_path / "calibration.json"
    calibration_path.write_text(json.dumps(calibration), encoding="utf-8")

    with pytest.raises(ValueError, match="does not exactly match"):
        collect_gello._load_agent(  # noqa: SLF001
            calibration_path,
            "/dev/serial/by-id/usb-FTDI_GELLO_1-if00-port0",
        )


def test_calibration_device_basename_must_be_string(tmp_path: Path) -> None:
    calibration = {
        "calibrated": True,
        "robot_model": "franka_panda",
        "gello_device_basename": 1,
        "joint_ids": [1, 2, 3, 4, 5, 6, 7],
        "joint_offsets": [0.0] * 7,
        "joint_signs": [1] * 7,
        "gripper_config": [8, 0.0, 1.0],
        "start_joints": [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0, 0.0],
        "calibrated_at": "2026-01-01T00:00:00+00:00",
    }
    calibration_path = tmp_path / "calibration.json"
    calibration_path.write_text(json.dumps(calibration), encoding="utf-8")

    with pytest.raises(ValueError, match="must be a string"):
        collect_gello._load_agent(  # noqa: SLF001
            calibration_path,
            "/dev/serial/by-id/1",
        )


@pytest.mark.parametrize(
    ("path", "directory", "flag"),
    [
        ("/dev/serial/by-id/../ttyUSB0", "/dev/serial/by-id", "--gello-port"),
        ("/dev/serial/by-id/nested/device", "/dev/serial/by-id", "--gello-port"),
        ("/dev/input/by-id/../event0", "/dev/input/by-id", "--deadman-device"),
    ],
)
def test_stable_device_path_must_be_direct_child(path: str, directory: str, flag: str) -> None:
    with pytest.raises(ValueError, match="direct child"):
        collect_gello._validate_stable_device_path(path, directory=directory, flag=flag)  # noqa: SLF001


def test_stable_device_path_accepts_direct_by_id_child() -> None:
    collect_gello._validate_stable_device_path(  # noqa: SLF001
        "/dev/serial/by-id/usb-FTDI_GELLO-if00-port0",
        directory="/dev/serial/by-id",
        flag="--gello-port",
    )
