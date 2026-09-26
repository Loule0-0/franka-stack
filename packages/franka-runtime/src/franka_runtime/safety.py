"""Stateful, hardware-independent safety filtering for Panda joint targets."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from franka_runtime.contracts import (
    ACTION_DIM,
    GRIPPER_INDEX,
    JOINT_DIM,
    PANDA_JOINT_LIMIT_HIGH,
    PANDA_JOINT_LIMIT_LOW,
    PANDA_JOINT_LIMIT_MARGIN_RAD,
    validate_action,
    validate_observation,
)


class SafetyError(ValueError):
    """Raised when the runtime cannot construct a safe command."""


def _positive_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise SafetyError(f"{name} must be a number")
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise SafetyError(f"{name} must be finite and > 0")
    return result


@dataclass(frozen=True)
class SafetyConfig:
    """Limits applied before a joint-position command reaches a controller."""

    max_joint_step_rad: float = 0.03
    max_joint_acceleration_rad_s2: float = 3.375

    def __post_init__(self) -> None:
        object.__setattr__(self, "max_joint_step_rad", _positive_finite("max_joint_step_rad", self.max_joint_step_rad))
        object.__setattr__(
            self,
            "max_joint_acceleration_rad_s2",
            _positive_finite("max_joint_acceleration_rad_s2", self.max_joint_acceleration_rad_s2),
        )


class PandaSafetyFilter:
    """Limit absolute 8D policy actions against the latest measured state.

    A missing or non-finite policy action fails closed: the output uses the
    *current measured* joint positions and gripper value.  It never replays the
    previous policy action.  A non-finite measured observation raises because
    no trustworthy hold target exists.
    """

    def __init__(self, config: SafetyConfig | None = None) -> None:
        self.config = config or SafetyConfig()
        self._previous_joint_velocity = np.zeros(JOINT_DIM, dtype=np.float64)

    @property
    def previous_joint_velocity(self) -> npt.NDArray[np.float64]:
        return self._previous_joint_velocity.copy()

    def reset(self) -> None:
        self._previous_joint_velocity.fill(0.0)

    def hold(self, measured_observation: object) -> npt.NDArray[np.float32]:
        """Build a hold action from fresh measured q and measured gripper state."""

        try:
            measured = validate_observation(measured_observation)
        except ValueError as exc:
            raise SafetyError(f"cannot hold without a valid measured observation: {exc}") from exc
        self.reset()
        return measured.copy()

    def filter(
        self,
        policy_action: object | None,
        measured_observation: object,
        *,
        dt_s: float,
    ) -> npt.NDArray[np.float32]:
        """Return a finite, limited 8D absolute-position command.

        Position step is limited relative to measured q.  Acceleration is
        limited as the change in commanded joint velocity over ``dt_s``.
        Contract-invalid finite actions raise instead of being clipped toward a
        hardware limit.  Non-finite actions fail closed to measured hold.
        """

        dt = _positive_finite("dt_s", dt_s)
        try:
            measured = validate_observation(measured_observation)
        except ValueError as exc:
            raise SafetyError(f"invalid measured observation: {exc}") from exc

        if policy_action is None:
            return self.hold(measured)

        try:
            raw_action = np.asarray(policy_action)
        except (TypeError, ValueError) as exc:
            raise SafetyError(f"policy action must have shape ({ACTION_DIM},)") from exc
        if raw_action.shape != (ACTION_DIM,):
            raise SafetyError(f"policy action must have shape ({ACTION_DIM},), got {raw_action.shape}")
        if raw_action.dtype.kind not in "iuf":
            raise SafetyError(f"policy action must contain real numbers, got dtype {raw_action.dtype}")
        if not np.isfinite(raw_action).all():
            return self.hold(measured)

        try:
            action = np.asarray(validate_action(raw_action), dtype=np.float64)
        except ValueError as exc:
            raise SafetyError(f"invalid policy action: {exc}") from exc
        measured_q = np.asarray(measured[:JOINT_DIM], dtype=np.float64)
        target_q = action[:JOINT_DIM]

        desired_step = np.clip(
            target_q - measured_q,
            -self.config.max_joint_step_rad,
            self.config.max_joint_step_rad,
        )
        desired_velocity = desired_step / dt
        max_velocity_change = self.config.max_joint_acceleration_rad_s2 * dt
        commanded_velocity = np.clip(
            desired_velocity,
            self._previous_joint_velocity - max_velocity_change,
            self._previous_joint_velocity + max_velocity_change,
        )
        commanded_step = np.clip(
            commanded_velocity * dt,
            -self.config.max_joint_step_rad,
            self.config.max_joint_step_rad,
        )
        command_q = np.clip(
            measured_q + commanded_step,
            PANDA_JOINT_LIMIT_LOW + PANDA_JOINT_LIMIT_MARGIN_RAD,
            PANDA_JOINT_LIMIT_HIGH - PANDA_JOINT_LIMIT_MARGIN_RAD,
        )

        # Store the velocity actually represented by the final, joint-limited command.
        self._previous_joint_velocity = (command_q - measured_q) / dt

        result = np.empty(ACTION_DIM, dtype=np.float32)
        result[:JOINT_DIM] = command_q
        result[GRIPPER_INDEX] = action[GRIPPER_INDEX]
        if not np.isfinite(result).all():  # Defensive invariant; inputs above are already finite.
            return self.hold(measured)
        return result
