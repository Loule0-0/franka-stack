from __future__ import annotations

import numpy as np
import pytest

from franka_runtime.safety import (
    PANDA_JOINT_LIMIT_HIGH,
    PANDA_JOINT_LIMIT_LOW,
    PandaSafetyFilter,
    SafetyConfig,
    SafetyError,
)


def _observation(offset: float = 0.0, gripper: float = 0.25) -> np.ndarray:
    neutral_q = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0])
    return np.concatenate((neutral_q + offset, [gripper]))


def test_panda_joint_limits_are_the_seven_axis_limits() -> None:
    assert PANDA_JOINT_LIMIT_LOW.shape == (7,)
    assert PANDA_JOINT_LIMIT_HIGH.shape == (7,)
    assert np.all(PANDA_JOINT_LIMIT_LOW < PANDA_JOINT_LIMIT_HIGH)
    assert not PANDA_JOINT_LIMIT_LOW.flags.writeable
    assert not PANDA_JOINT_LIMIT_HIGH.flags.writeable


def test_filter_limits_position_step() -> None:
    safety = PandaSafetyFilter(SafetyConfig(max_joint_step_rad=0.03, max_joint_acceleration_rad_s2=1e6))
    measured = _observation()
    action = measured.copy()
    action[:7] += 0.5
    action[7] = 1.0
    command = safety.filter(action, measured, dt_s=0.1)

    np.testing.assert_allclose(command[:7] - measured[:7], 0.03, atol=1e-6)
    assert command[7] == 1.0
    assert np.all(command[:7] <= PANDA_JOINT_LIMIT_HIGH)


@pytest.mark.parametrize(
    "action",
    [
        np.array([10.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0, 0.5]),
        np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0, 1.1]),
    ],
)
def test_finite_out_of_contract_action_is_rejected_not_clipped(action: np.ndarray) -> None:
    safety = PandaSafetyFilter()

    with pytest.raises(SafetyError, match="invalid policy action"):
        safety.filter(action, _observation(), dt_s=0.05)


def test_filter_limits_true_joint_acceleration_across_cycles() -> None:
    safety = PandaSafetyFilter(SafetyConfig(max_joint_step_rad=1.0, max_joint_acceleration_rad_s2=1.0))
    measured = _observation()
    target = measured.copy()
    target[:7] += 0.5
    target[7] = 0.5

    first = safety.filter(target, measured, dt_s=0.1)
    second = safety.filter(target, measured, dt_s=0.1)

    # Starting from zero commanded velocity, dv <= a*dt = 0.1 rad/s,
    # hence the first and second position steps are 0.01 and 0.02 rad.
    np.testing.assert_allclose(first[:7] - measured[:7], 0.01, atol=1e-7)
    np.testing.assert_allclose(second[:7] - measured[:7], 0.02, atol=1e-7)
    np.testing.assert_allclose(safety.previous_joint_velocity, 0.2, atol=1e-7)


def test_missing_action_holds_current_measurement_not_previous_action() -> None:
    safety = PandaSafetyFilter(SafetyConfig(max_joint_step_rad=1.0, max_joint_acceleration_rad_s2=1e6))
    old_measurement = _observation(offset=0.0, gripper=0.1)
    previous_action = old_measurement.copy()
    previous_action[:7] += 0.4
    previous_action[7] = 0.9
    safety.filter(previous_action, old_measurement, dt_s=0.1)
    current_measurement = _observation(offset=-0.2, gripper=0.3)

    held = safety.filter(None, current_measurement, dt_s=0.1)

    np.testing.assert_array_equal(held, current_measurement.astype(np.float32))
    np.testing.assert_array_equal(safety.previous_joint_velocity, np.zeros(7))


def test_nonfinite_policy_action_fails_to_measured_hold() -> None:
    safety = PandaSafetyFilter()
    measured = _observation(offset=0.1, gripper=0.4)
    action = np.array([0.2] * 7 + [np.nan])

    np.testing.assert_array_equal(safety.filter(action, measured, dt_s=0.1), measured.astype(np.float32))


@pytest.mark.parametrize("bad", [np.nan, np.inf])
def test_nonfinite_measurement_cannot_be_used_as_hold(bad: float) -> None:
    safety = PandaSafetyFilter()
    measured = _observation()
    measured[0] = bad

    with pytest.raises(SafetyError, match="invalid measured observation"):
        safety.filter(None, measured, dt_s=0.1)


def test_bad_action_shape_is_an_interface_error() -> None:
    with pytest.raises(SafetyError, match=r"shape \(8,\)"):
        PandaSafetyFilter().filter(np.zeros(7), _observation(), dt_s=0.1)
