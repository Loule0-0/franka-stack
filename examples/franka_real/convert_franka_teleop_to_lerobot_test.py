from __future__ import annotations

# The converter deliberately keeps strict path/cadence helpers private. The
# test also targets Python 3.8, where datetime.UTC is unavailable.
# ruff: noqa: SLF001, UP017
import datetime as dt
import json
from pathlib import Path
import pickle

from franka_runtime.dataset_provenance import FRANKA_DATASET_MANIFEST_KEYS
from franka_runtime.dataset_provenance import LEROBOT_DATA_PATH
from franka_runtime.dataset_provenance import LEROBOT_VIDEO_PATH
from franka_runtime.dataset_provenance import verify_dataset_content_digest
import numpy as np
import pytest

from examples.franka_real import convert_franka_teleop_to_lerobot as converter


def _episode(tmp_path: Path, offsets_s: list[float]) -> Path:
    episode = tmp_path / "episode"
    episode.mkdir()
    origin = dt.datetime(2026, 1, 1, tzinfo=dt.timezone.utc)
    for offset_s in offsets_s:
        timestamp = origin + dt.timedelta(seconds=offset_s)
        filename = timestamp.strftime("%Y-%m-%dT%H-%M-%S.%fZ.pkl")
        with (episode / filename).open("wb") as stream:
            pickle.dump(
                {
                    "task": "pick up the block",
                    "joint_positions": np.zeros(8, dtype=np.float32),
                    "control": np.zeros(8, dtype=np.float32),
                },
                stream,
            )
    metadata = {
        "schema_id": converter.SCHEMA_ID,
        "task": "pick up the block",
        "outcome": "complete",
        "recorded_frames": len(offsets_s),
        "control_hz": converter.TARGET_FPS,
        "max_joint_step_rad": converter.MAX_JOINT_STEP_RAD,
        "max_joint_acceleration_rad_s2": converter.MAX_JOINT_ACCELERATION_RAD_S2,
        "camera_ports": {"exterior": 5000, "wrist": 5001},
        "color_space": "rgb_uint8_hwc",
        "gripper_semantics": "closed_fraction_0_open_1_closed",
        "gello_device_sha256": "a" * 64,
        "calibration_sha256": "b" * 64,
    }
    (episode / "episode.json").write_text(json.dumps(metadata), encoding="utf-8")
    return episode


def _load(episode: Path):
    return converter._load_episode(
        episode,
        expected_task="pick up the block",
        max_frame_gap_s=0.125,
        max_period_jitter_s=0.03,
        min_cadence_ratio=0.9,
    )


def test_timestamp_parser_accepts_utc_z_on_python38() -> None:
    timestamp = converter._timestamp_from_path(Path("2026-01-01T00-00-00.125000Z.pkl"))

    assert timestamp == dt.datetime(2026, 1, 1, 0, 0, 0, 125000, tzinfo=dt.timezone.utc)


def test_valid_twenty_hz_jitter_preserves_every_frame(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.049, 0.100, 0.149, 0.200])

    frames, _metadata, cadence = _load(episode)

    assert len(frames) == 5
    assert cadence["cadence_ratio"] == 1.0


def test_sparse_episode_is_rejected_instead_of_compressed(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.05, 0.20])

    with pytest.raises(ValueError, match="max frame gap"):
        _load(episode)


def test_forty_hz_episode_is_rejected_instead_of_stretched(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.025, 0.050, 0.075, 0.100])

    with pytest.raises(ValueError, match="median frame period"):
        _load(episode)


def test_episode_requires_exact_recorder_safety_contract(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.05])
    metadata_path = episode / "episode.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["max_joint_step_rad"] = converter.MAX_JOINT_STEP_RAD * 2
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

    with pytest.raises(ValueError, match="max_joint_step_rad must be"):
        _load(episode)


@pytest.mark.parametrize(
    ("field", "bad_value", "message"),
    [
        ("color_space", "bgr_uint8_hwc", "color_space"),
        ("gripper_semantics", "open_fraction_0_closed_1_open", "gripper_semantics"),
        ("camera_ports", {"exterior": 5000, "wrist": 5000}, "camera ports"),
        ("gello_device_sha256", "not-a-digest", "gello_device_sha256"),
        ("calibration_sha256", "not-a-digest", "calibration_sha256"),
    ],
)
def test_episode_rejects_raw_metadata_semantic_mismatch(
    tmp_path: Path, field: str, bad_value: object, message: str
) -> None:
    episode = _episode(tmp_path, [0.0, 0.05])
    metadata_path = episode / "episode.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata[field] = bad_value
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        _load(episode)


def test_episode_rejects_raw_metadata_schema_drift(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.05])
    metadata_path = episode / "episode.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["legacy"] = True
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

    with pytest.raises(ValueError, match="metadata fields must exactly match"):
        _load(episode)


def test_episode_rejects_non_string_metadata_task(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.05])
    metadata_path = episode / "episode.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["task"] = 123
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")

    with pytest.raises(ValueError, match="task must be a string"):
        converter._load_episode(
            episode,
            expected_task="123",
            max_frame_gap_s=0.125,
            max_period_jitter_s=0.03,
            min_cadence_ratio=0.9,
        )


def test_episode_rejects_non_string_frame_task(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.05])
    frame_path = sorted(episode.glob("*.pkl"))[0]
    with frame_path.open("rb") as stream:
        sample = pickle.load(stream)
    sample["task"] = 123
    with frame_path.open("wb") as stream:
        pickle.dump(sample, stream)

    with pytest.raises(ValueError, match="task must be a string"):
        _load(episode)


def test_episode_rejects_recorded_step_that_bypassed_filter(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.05])
    frame_path = sorted(episode.glob("*.pkl"))[0]
    with frame_path.open("rb") as stream:
        sample = pickle.load(stream)
    sample["control"][0] = converter.MAX_JOINT_STEP_RAD + 0.01
    with frame_path.open("wb") as stream:
        pickle.dump(sample, stream)

    with pytest.raises(ValueError, match="commanded joint step"):
        _load(episode)


def test_episode_rejects_recorded_acceleration_that_bypassed_filter(tmp_path: Path) -> None:
    episode = _episode(tmp_path, [0.0, 0.05])
    first, second = sorted(episode.glob("*.pkl"))
    for frame_path, command in ((first, 0.0075), (second, -0.0075)):
        with frame_path.open("rb") as stream:
            sample = pickle.load(stream)
        sample["control"][0] = command
        with frame_path.open("wb") as stream:
            pickle.dump(sample, stream)

    with pytest.raises(ValueError, match="commanded joint acceleration"):
        _load(episode)


@pytest.mark.parametrize("repo_id", ["raw", "../raw", "/absolute/raw", "owner/name/extra"])
def test_output_requires_canonical_repo_id(tmp_path: Path, repo_id: str) -> None:
    with pytest.raises(ValueError, match="namespace/name"):
        converter._resolve_output_path(dataset_home=tmp_path, repo_id=repo_id, raw_dir=tmp_path / "input")


@pytest.mark.parametrize("raw_suffix", ["owner/name", "owner", "owner/name/raw"])
def test_output_cannot_overlap_raw_recordings(tmp_path: Path, raw_suffix: str) -> None:
    with pytest.raises(ValueError, match="must not overlap"):
        converter._resolve_output_path(
            dataset_home=tmp_path,
            repo_id="owner/name",
            raw_dir=tmp_path / raw_suffix,
        )


def test_output_rejects_filesystem_root_as_dataset_home(tmp_path: Path) -> None:
    filesystem_root = Path(tmp_path.anchor)

    with pytest.raises(ValueError, match="filesystem root"):
        converter._resolve_output_path(
            dataset_home=filesystem_root,
            repo_id="home/data",
            raw_dir=tmp_path / "raw",
        )


def test_overwrite_requires_matching_converter_manifest(tmp_path: Path) -> None:
    target = tmp_path / "owner" / "name"
    target.mkdir(parents=True)

    with pytest.raises(ValueError, match="unowned or incomplete"):
        converter._validate_owned_overwrite_target(target, repo_id="owner/name")

    (target / "franka_manifest.json").write_text(
        json.dumps({"schema_id": converter.SCHEMA_ID, "repo_id": "owner/other"}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="does not exactly identify"):
        converter._validate_owned_overwrite_target(target, repo_id="owner/name")


def test_overwrite_accepts_only_matching_converter_manifest(tmp_path: Path) -> None:
    target = tmp_path / "owner" / "name"
    target.mkdir(parents=True)
    (target / "franka_manifest.json").write_text(
        json.dumps({"schema_id": converter.SCHEMA_ID, "repo_id": "owner/name"}),
        encoding="utf-8",
    )

    converter._validate_owned_overwrite_target(target, repo_id="owner/name")


def test_converter_atomically_publishes_manifest_after_binding_payload(tmp_path: Path) -> None:
    meta = tmp_path / "meta"
    meta.mkdir()
    info = {
        "codebase_version": "v2.1",
        "chunks_size": 1000,
        "total_episodes": 1,
        "data_path": LEROBOT_DATA_PATH,
        "video_path": LEROBOT_VIDEO_PATH,
        "features": {"observation.images.exterior": {"dtype": "image"}},
    }
    (meta / "info.json").write_text(json.dumps(info), encoding="utf-8")
    (meta / "episodes.jsonl").write_text("{}\n", encoding="utf-8")
    (meta / "episodes_stats.jsonl").write_text("{}\n", encoding="utf-8")
    (meta / "tasks.jsonl").write_text("{}\n", encoding="utf-8")
    parquet = tmp_path / "data" / "chunk-000" / "episode_000000.parquet"
    parquet.parent.mkdir(parents=True)
    parquet.write_bytes(b"parquet")

    input_manifest = {
        "schema_id": converter.SCHEMA_ID,
        "repo_id": "local/franka_gello",
        "task": "pick up the block",
        "fps": converter.TARGET_FPS,
        "color_space": "rgb_uint8_hwc",
        "state": "q7_rad + gripper_closed",
        "action": "absolute_q7_rad + gripper_closed",
        "camera_roles": {"exterior": converter.EXTERIOR_KEY, "wrist": converter.WRIST_KEY},
        "episodes": [],
    }
    path = converter._write_final_manifest(tmp_path, input_manifest)

    assert path == tmp_path / "franka_manifest.json"
    assert not (tmp_path / "franka_manifest.json.tmp").exists()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    assert set(manifest) == FRANKA_DATASET_MANIFEST_KEYS
    verify_dataset_content_digest(tmp_path, manifest["content_digest"])


def test_converter_refuses_manifest_schema_drift_before_publication(tmp_path: Path) -> None:
    manifest = dict.fromkeys(FRANKA_DATASET_MANIFEST_KEYS - {"content_digest"})
    manifest["unexpected"] = True

    with pytest.raises(ValueError, match="fields must exactly match"):
        converter._write_final_manifest(tmp_path, manifest)

    assert not (tmp_path / "franka_manifest.json").exists()
