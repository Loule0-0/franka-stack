"""Wire-level policy contract for the single-arm Franka runtime.

The state and action vectors intentionally use the same fixed eight dimensions:
seven Panda joint positions in radians, followed by a normalized gripper closed
fraction (0 is fully open, 1 is fully closed).  Actions are absolute joint
position targets, never deltas or velocities.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping

import numpy as np
import numpy.typing as npt

JOINT_DIM = 7
OBSERVATION_DIM = 8
ACTION_DIM = 8
GRIPPER_INDEX = 7

# Franka Emika Panda position limits.  Policy actions use a small conservative
# margin so a learned policy can never deliberately command a mechanical stop.
# Measured observations are allowed across the complete physical range.
PANDA_JOINT_LIMIT_LOW: npt.NDArray[np.float64] = np.array(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973], dtype=np.float64
)
PANDA_JOINT_LIMIT_HIGH: npt.NDArray[np.float64] = np.array(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973], dtype=np.float64
)
PANDA_JOINT_LIMIT_MARGIN_RAD = 0.02
PANDA_JOINT_LIMIT_LOW.setflags(write=False)
PANDA_JOINT_LIMIT_HIGH.setflags(write=False)

JOINT_NAMES = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)

POLICY_SCHEMA_VERSION = "franka-runtime/v1"
ROBOT_MODEL = "franka_panda"
OBSERVATION_SEMANTICS = "joint_position_rad[7]+gripper_closed_fraction[1]"
ACTION_SEMANTICS = "absolute_joint_position_rad[7]+gripper_closed_fraction[1]"

FloatArray = npt.NDArray[np.float32]


class ContractError(ValueError):
    """Raised when policy data does not satisfy the Franka runtime contract."""


def _require_plain_int(name: str, value: object, *, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError(f"{name} must be an integer, got {type(value).__name__}")
    if value < minimum:
        raise ContractError(f"{name} must be >= {minimum}, got {value}")
    return value


def _require_positive_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ContractError(f"{name} must be a number, got {type(value).__name__}")
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ContractError(f"{name} must be finite and > 0, got {value!r}")
    return result


@dataclass(frozen=True)
class PolicyMetadata:
    """The complete metadata handshake required by a Franka policy client.

    ``from_mapping`` rejects unknown keys as well as missing keys.  This is
    deliberate: silently accepting a velocity or delta-action policy as an
    absolute-position policy is unsafe on a physical robot.
    """

    schema_version: str
    robot_model: str
    observation_dim: int
    action_dim: int
    observation_semantics: str
    action_semantics: str
    action_horizon: int
    control_hz: float

    def __post_init__(self) -> None:
        expected_strings = {
            "schema_version": POLICY_SCHEMA_VERSION,
            "robot_model": ROBOT_MODEL,
            "observation_semantics": OBSERVATION_SEMANTICS,
            "action_semantics": ACTION_SEMANTICS,
        }
        for name, expected in expected_strings.items():
            actual = getattr(self, name)
            if actual != expected:
                raise ContractError(f"{name} must be {expected!r}, got {actual!r}")

        observation_dim = _require_plain_int("observation_dim", self.observation_dim, minimum=1)
        action_dim = _require_plain_int("action_dim", self.action_dim, minimum=1)
        if observation_dim != OBSERVATION_DIM:
            raise ContractError(f"observation_dim must be {OBSERVATION_DIM}, got {observation_dim}")
        if action_dim != ACTION_DIM:
            raise ContractError(f"action_dim must be {ACTION_DIM}, got {action_dim}")
        _require_plain_int("action_horizon", self.action_horizon, minimum=1)
        control_hz = _require_positive_finite("control_hz", self.control_hz)
        object.__setattr__(self, "control_hz", control_hz)

    @classmethod
    def create(cls, *, action_horizon: int, control_hz: float) -> PolicyMetadata:
        """Build metadata with the fixed semantic fields filled in."""

        return cls(
            schema_version=POLICY_SCHEMA_VERSION,
            robot_model=ROBOT_MODEL,
            observation_dim=OBSERVATION_DIM,
            action_dim=ACTION_DIM,
            observation_semantics=OBSERVATION_SEMANTICS,
            action_semantics=ACTION_SEMANTICS,
            action_horizon=action_horizon,
            control_hz=control_hz,
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> PolicyMetadata:
        if not isinstance(value, Mapping):
            raise ContractError(f"policy metadata must be a mapping, got {type(value).__name__}")

        expected_keys = {field_name for field_name in cls.__dataclass_fields__}
        actual_keys = set(value)
        missing = sorted(expected_keys - actual_keys)
        unknown = sorted(repr(key) for key in actual_keys - expected_keys)
        if missing or unknown:
            parts = []
            if missing:
                parts.append(f"missing keys: {missing}")
            if unknown:
                parts.append(f"unknown keys: {unknown}")
            raise ContractError("invalid policy metadata (" + "; ".join(parts) + ")")
        return cls(**{key: value[key] for key in expected_keys})

    def to_mapping(self) -> dict[str, str | int | float]:
        return asdict(self)


def validate_policy_metadata(value: Mapping[str, Any] | PolicyMetadata) -> PolicyMetadata:
    """Validate and normalize policy metadata."""

    if isinstance(value, PolicyMetadata):
        return value
    return PolicyMetadata.from_mapping(value)


# The repository intentionally exposes one deployable policy contract.  Keep
# this value next to the validator so training, serving, and robot clients fail
# together if the horizon or control rate is changed deliberately.
EXPECTED_POLICY_METADATA = PolicyMetadata.create(action_horizon=20, control_hz=20.0)


def _numeric_array(name: str, value: object, shape: tuple[int, ...]) -> FloatArray:
    try:
        array = np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise ContractError(f"{name} must be a numeric array with shape {shape}") from exc
    if array.shape != shape:
        raise ContractError(f"{name} must have shape {shape}, got {array.shape}")
    if array.dtype.kind not in "iuf":
        raise ContractError(f"{name} must contain real numbers, got dtype {array.dtype}")
    if not np.isfinite(array).all():
        raise ContractError(f"{name} contains NaN or infinity")
    return np.asarray(array, dtype=np.float32).copy()


def _validate_gripper(name: str, array: FloatArray) -> None:
    gripper = array[..., GRIPPER_INDEX]
    if np.any((gripper < 0.0) | (gripper > 1.0)):
        raise ContractError(f"{name} gripper_closed_fraction must be in [0, 1]")


def _validate_joint_limits(name: str, array: FloatArray, *, margin_rad: float) -> None:
    low = PANDA_JOINT_LIMIT_LOW + margin_rad
    high = PANDA_JOINT_LIMIT_HIGH - margin_rad
    joints = array[..., :JOINT_DIM]
    if np.any((joints < low) | (joints > high)):
        suffix = "" if margin_rad == 0.0 else f" with {margin_rad:g} rad safety margin"
        raise ContractError(f"{name} joint positions exceed Panda limits{suffix}")


def validate_observation(value: object) -> FloatArray:
    """Return a copied float32 observation after strict 8D validation."""

    observation = _numeric_array("observation", value, (OBSERVATION_DIM,))
    _validate_gripper("observation", observation)
    _validate_joint_limits("observation", observation, margin_rad=0.0)
    return observation


def validate_action(value: object) -> FloatArray:
    """Return a copied float32 absolute-position action after validation."""

    action = _numeric_array("action", value, (ACTION_DIM,))
    _validate_gripper("action", action)
    _validate_joint_limits("action", action, margin_rad=PANDA_JOINT_LIMIT_MARGIN_RAD)
    return action


def validate_action_chunk(value: object, *, expected_horizon: int | None = None) -> FloatArray:
    """Validate a non-empty ``[time, 8]`` absolute-position action chunk."""

    try:
        array = np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise ContractError("action chunk must be a numeric [time, 8] array") from exc
    if array.ndim != 2 or array.shape[1:] != (ACTION_DIM,) or array.shape[0] < 1:
        raise ContractError(f"action chunk must have shape [time, {ACTION_DIM}] with time >= 1, got {array.shape}")
    if expected_horizon is not None:
        horizon = _require_plain_int("expected_horizon", expected_horizon, minimum=1)
        if array.shape[0] != horizon:
            raise ContractError(f"action chunk horizon must be {horizon}, got {array.shape[0]}")
    if array.dtype.kind not in "iuf":
        raise ContractError(f"action chunk must contain real numbers, got dtype {array.dtype}")
    if not np.isfinite(array).all():
        raise ContractError("action chunk contains NaN or infinity")
    result = np.asarray(array, dtype=np.float32).copy()
    _validate_gripper("action chunk", result)
    _validate_joint_limits("action chunk", result, margin_rad=PANDA_JOINT_LIMIT_MARGIN_RAD)
    return result


def join_observation(joint_positions_rad: object, gripper_closed_fraction: float) -> FloatArray:
    """Construct the canonical 8D observation from its physical components."""

    joints = _numeric_array("joint_positions_rad", joint_positions_rad, (JOINT_DIM,))
    return validate_observation(np.concatenate((joints, np.asarray([gripper_closed_fraction]))))


def split_observation(observation: object) -> tuple[FloatArray, float]:
    """Return a joint-position copy and scalar gripper closed fraction."""

    normalized = validate_observation(observation)
    return normalized[:JOINT_DIM].copy(), float(normalized[GRIPPER_INDEX])
