from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from franka_runtime.dataset_provenance import LEROBOT_DATA_PATH
from franka_runtime.dataset_provenance import LEROBOT_VIDEO_PATH
from franka_runtime.dataset_provenance import DatasetContentError
from franka_runtime.dataset_provenance import compute_dataset_content_digest
import numpy as np
import pytest

from examples.franka_real import audit_dataset as audit
from examples.franka_real.audit_dataset import sample_indices
from examples.franka_real.audit_dataset import validate_command_safety_sequence
from examples.franka_real.audit_dataset import validate_dataset_counts
from examples.franka_real.audit_dataset import validate_dataset_tasks
from examples.franka_real.audit_dataset import validate_feature_contract
from examples.franka_real.audit_dataset import validate_manifest_contract
from examples.franka_real.audit_dataset import validate_sample


def _features() -> dict:
    return {
        "observation.images.exterior": {"dtype": "image", "shape": (240, 320, 3)},
        "observation.images.wrist": {"dtype": "image", "shape": (240, 320, 3)},
        "observation.state": {"dtype": "float32", "shape": (8,)},
        "action": {"dtype": "float32", "shape": (8,)},
    }


def _sample() -> dict:
    value = np.array([0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0, 0.25], dtype=np.float32)
    return {
        "observation.images.exterior": np.zeros((3, 240, 320), dtype=np.float32),
        "observation.images.wrist": np.zeros((240, 320, 3), dtype=np.uint8),
        "observation.state": value,
        "action": value.copy(),
        "task": "pick up the object",
    }


def _episode(name: str = "episode_000000", frames: int = 10) -> dict:
    return {
        "episode": name,
        "outcome": "complete",
        "raw_frames": frames,
        "frames": frames,
        "duration_s": 0.45,
        "max_frame_gap_s": 0.051,
        "median_period_s": 0.05,
        "p95_period_jitter_s": 0.001,
        "cadence_ratio": 1.0,
    }


def _manifest() -> dict:
    return {
        "schema_id": "franka-runtime/v1",
        "repo_id": "local/franka_gello",
        "task": "pick up the object",
        "fps": 20,
        "color_space": "rgb_uint8_hwc",
        "state": "q7_rad + gripper_closed",
        "action": "absolute_q7_rad + gripper_closed",
        "camera_roles": {"exterior": "base_rgb", "wrist": "wrist_rgb"},
        "episodes": [_episode()],
        "content_digest": {
            "schema_version": "franka-dataset-content/v1",
            "algorithm": "sha256",
            "sha256": "a" * 64,
            "file_count": 5,
            "total_bytes": 1,
        },
    }


def test_contract_and_sample_accept_canonical_data() -> None:
    episodes = validate_manifest_contract(_manifest(), repo_id="local/franka_gello")
    validate_dataset_counts(episodes, num_episodes=1, num_frames=10, metadata_episode_lengths=[10])
    validate_feature_contract(_features())
    validate_sample(_sample(), index=0)


def test_dataset_task_metadata_requires_exact_strings() -> None:
    validate_dataset_tasks(["pick up the object"], expected_task="pick up the object")

    with pytest.raises(ValueError, match="non-empty strings"):
        validate_dataset_tasks([123], expected_task="123")
    with pytest.raises(ValueError, match="exactly manifest task"):
        validate_dataset_tasks(["pick up another object"], expected_task="pick up the object")


@pytest.mark.parametrize(
    ("field", "bad_value"),
    [
        ("schema_id", "franka-runtime/v0"),
        ("repo_id", "local/other"),
        ("task", ""),
        ("fps", 30),
        ("fps", 20.0),
        ("color_space", "bgr_uint8_hwc"),
        ("state", "q7"),
        ("action", "delta_q7"),
        ("camera_roles", {"exterior": "wrist_rgb", "wrist": "base_rgb"}),
        ("episodes", []),
    ],
)
def test_manifest_rejects_every_contract_mismatch(field: str, bad_value: object) -> None:
    manifest = _manifest()
    manifest[field] = bad_value
    with pytest.raises(ValueError, match=f"manifest {field}|manifest episodes"):
        validate_manifest_contract(manifest, repo_id="local/franka_gello")


def test_manifest_rejects_extra_fields_and_bad_episode_summary() -> None:
    manifest = _manifest()
    manifest["legacy"] = True
    with pytest.raises(ValueError, match="fields must exactly match"):
        validate_manifest_contract(manifest, repo_id="local/franka_gello")

    manifest = _manifest()
    manifest["episodes"][0]["frames"] = 9
    with pytest.raises(ValueError, match="raw_frames and frames must match"):
        validate_manifest_contract(manifest, repo_id="local/franka_gello")


@pytest.mark.parametrize(
    ("field", "bad_value"),
    [
        ("max_frame_gap_s", 0.126),
        ("median_period_s", 0.025),
        ("p95_period_jitter_s", 0.031),
        ("cadence_ratio", 0.89),
    ],
)
def test_manifest_rejects_unsafe_cadence(field: str, bad_value: float) -> None:
    manifest = _manifest()
    manifest["episodes"][0][field] = bad_value

    with pytest.raises(ValueError, match=field):
        validate_manifest_contract(manifest, repo_id="local/franka_gello")


def test_manifest_counts_must_match_dataset_metadata() -> None:
    episodes = [_episode("episode_000000", 10), _episode("episode_000001", 12)]
    with pytest.raises(ValueError, match=r"2 episodes.*1"):
        validate_dataset_counts(episodes, num_episodes=1, num_frames=22, metadata_episode_lengths=[22])
    with pytest.raises(ValueError, match=r"22 frames.*21"):
        validate_dataset_counts(episodes, num_episodes=2, num_frames=21, metadata_episode_lengths=[10, 11])
    with pytest.raises(ValueError, match=r"episode lengths total 21.*22"):
        validate_dataset_counts(episodes, num_episodes=2, num_frames=22, metadata_episode_lengths=[10, 11])


def test_sample_rejects_non_finite_action() -> None:
    sample = _sample()
    sample["action"][0] = np.nan
    with pytest.raises(ValueError, match="finite shape"):
        validate_sample(sample, index=3)


def test_sample_uses_physical_state_limits_and_action_margin() -> None:
    sample = _sample()
    sample["observation.state"][0] = -2.8974
    with pytest.raises(ValueError, match="physical joint limits"):
        validate_sample(sample, index=1)

    sample = _sample()
    sample["action"][0] = -2.8973 + 0.019
    with pytest.raises(ValueError, match=r"0.02 rad margin"):
        validate_sample(sample, index=2)


@pytest.mark.parametrize("key", ["observation.state", "action"])
def test_sample_rejects_gripper_outside_unit_interval(key: str) -> None:
    sample = _sample()
    sample[key][7] = 1.01
    with pytest.raises(ValueError, match=r"gripper_closed_fraction in \[0, 1\]"):
        validate_sample(sample, index=4)


def test_sample_indices_cover_endpoints_without_duplicates() -> None:
    assert sample_indices(10, 3).tolist() == [0, 4, 9]
    assert sample_indices(3, None).tolist() == [0, 1, 2]


def _command_sample(command: float) -> dict:
    state = np.zeros(8, dtype=np.float32)
    action = state.copy()
    action[0] = command
    return {"observation.state": state, "action": action}


def test_command_safety_sequence_accepts_contract_and_resets_each_episode() -> None:
    samples = [_command_sample(0.0075), _command_sample(0.0075), _command_sample(-0.0075)]

    validate_command_safety_sequence(samples, episode_lengths=[2, 1])


def test_command_safety_sequence_allows_float32_rounding_at_acceleration_limit() -> None:
    state = np.zeros(8, dtype=np.float32)
    state[0] = 2.8
    first = {"observation.state": state, "action": state.copy()}
    second = {"observation.state": state, "action": state.copy()}
    first["action"][0] = np.float32(state[0] + 0.0075)
    second["action"][0] = np.float32(state[0] + 0.015)

    validate_command_safety_sequence([first, second], episode_lengths=[2])


def test_command_safety_sequence_rejects_step_bypass() -> None:
    with pytest.raises(ValueError, match="commanded joint step"):
        validate_command_safety_sequence([_command_sample(0.035)], episode_lengths=[1])


def test_command_safety_sequence_rejects_acceleration_bypass() -> None:
    samples = [_command_sample(0.0075), _command_sample(-0.0075)]

    with pytest.raises(ValueError, match="commanded joint acceleration"):
        validate_command_safety_sequence(samples, episode_lengths=[2])


def test_command_safety_sequence_rejects_declared_length_mismatch() -> None:
    with pytest.raises(ValueError, match="ended inside episode"):
        validate_command_safety_sequence([_command_sample(0.0)], episode_lengths=[2])

    with pytest.raises(ValueError, match="beyond the declared"):
        validate_command_safety_sequence([_command_sample(0.0)], episode_lengths=[])


def _write_payload(root: Path) -> Path:
    meta = root / "meta"
    meta.mkdir(parents=True)
    info = {
        "codebase_version": "v2.1",
        "chunks_size": 1000,
        "total_episodes": 1,
        "data_path": LEROBOT_DATA_PATH,
        "video_path": LEROBOT_VIDEO_PATH,
        "features": {
            "observation.images.exterior": {"dtype": "image"},
            "observation.state": {"dtype": "float32"},
            "action": {"dtype": "float32"},
        },
    }
    (meta / "info.json").write_text(json.dumps(info), encoding="utf-8")
    (meta / "episodes.jsonl").write_text("{}\n", encoding="utf-8")
    (meta / "episodes_stats.jsonl").write_text("{}\n", encoding="utf-8")
    (meta / "tasks.jsonl").write_text("{}\n", encoding="utf-8")
    parquet = root / "data" / "chunk-000" / "episode_000000.parquet"
    parquet.parent.mkdir(parents=True)
    parquet.write_bytes(b"original parquet")
    return parquet


def test_main_rejects_tampered_payload_before_lerobot_decode(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    parquet = _write_payload(tmp_path)
    manifest = _manifest()
    manifest["content_digest"] = compute_dataset_content_digest(tmp_path).to_mapping()
    (tmp_path / "franka_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    parquet.write_bytes(b"tampered parquet")
    monkeypatch.setattr(
        audit,
        "parse_args",
        lambda: SimpleNamespace(dataset_root=tmp_path, repo_id="local/franka_gello", max_frames=0),
    )

    with pytest.raises(DatasetContentError, match="dataset content digest mismatch"):
        audit.main()
