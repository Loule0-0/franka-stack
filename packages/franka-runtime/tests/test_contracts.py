from __future__ import annotations

import numpy as np
import pytest

from franka_runtime.contracts import (
    ACTION_SEMANTICS,
    OBSERVATION_SEMANTICS,
    ContractError,
    PolicyMetadata,
    join_observation,
    split_observation,
    validate_action,
    validate_action_chunk,
    validate_observation,
    validate_policy_metadata,
)


def test_policy_metadata_round_trip() -> None:
    metadata = PolicyMetadata.create(action_horizon=15, control_hz=20)

    assert metadata.observation_semantics == OBSERVATION_SEMANTICS
    assert metadata.action_semantics == ACTION_SEMANTICS
    assert metadata.control_hz == 20.0
    assert validate_policy_metadata(metadata.to_mapping()) == metadata


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"action_dim": 7}, "action_dim must be 8"),
        ({"action_horizon": True}, "action_horizon must be an integer"),
        ({"action_semantics": "joint_velocity_rad_s[7]+gripper[1]"}, "action_semantics must be"),
        ({"control_hz": float("nan")}, "control_hz must be finite"),
    ],
)
def test_policy_metadata_rejects_incompatible_values(change: dict[str, object], message: str) -> None:
    raw = PolicyMetadata.create(action_horizon=15, control_hz=20).to_mapping()
    raw.update(change)

    with pytest.raises(ContractError, match=message):
        validate_policy_metadata(raw)


def test_policy_metadata_rejects_missing_and_unknown_keys() -> None:
    raw = PolicyMetadata.create(action_horizon=15, control_hz=20).to_mapping()
    del raw["robot_model"]
    raw["reset_pose"] = [0.0] * 7

    with pytest.raises(ContractError, match="missing keys.*robot_model.*unknown keys.*reset_pose"):
        validate_policy_metadata(raw)


def test_observation_helpers_preserve_8d_semantics() -> None:
    safe_joints = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0])
    observation = join_observation(safe_joints, 0.25)
    joints, gripper = split_observation(observation)

    assert observation.shape == (8,)
    assert observation.dtype == np.float32
    np.testing.assert_array_equal(joints, safe_joints.astype(np.float32))
    assert gripper == pytest.approx(0.25)


@pytest.mark.parametrize(
    "value",
    [
        np.zeros(7),
        np.zeros(9),
        np.array([0.0] * 7 + [1.1]),
        np.array([0.0] * 7 + [np.nan]),
        np.array(["0"] * 8),
    ],
)
def test_observation_validation_is_strict(value: object) -> None:
    with pytest.raises(ContractError):
        validate_observation(value)


def test_action_validation_returns_an_independent_float32_copy() -> None:
    raw = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0, 0.5], dtype=np.float64)
    action = validate_action(raw)
    raw[0] = 2.0

    assert action.dtype == np.float32
    assert action[0] == 0.0


def test_action_chunk_requires_exact_horizon_when_requested() -> None:
    chunk = np.tile(np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0, 0.0]), (3, 1))
    validate_action_chunk(chunk, expected_horizon=3)

    with pytest.raises(ContractError, match="horizon must be 4"):
        validate_action_chunk(chunk, expected_horizon=4)


def test_action_and_chunk_reject_joint_targets_at_hardware_limit() -> None:
    unsafe = np.array([2.8973, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0, 0.5])

    with pytest.raises(ContractError, match="joint positions exceed Panda limits"):
        validate_action(unsafe)
    with pytest.raises(ContractError, match="joint positions exceed Panda limits"):
        validate_action_chunk(np.tile(unsafe, (2, 1)))


@pytest.mark.parametrize("shape", [(0, 8), (3, 7), (3, 9), (8,)])
def test_action_chunk_rejects_non_contract_shapes(shape: tuple[int, ...]) -> None:
    with pytest.raises(ContractError, match="action chunk must have shape"):
        validate_action_chunk(np.zeros(shape, dtype=np.float32))
