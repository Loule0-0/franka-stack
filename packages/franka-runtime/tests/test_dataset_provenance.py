from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from franka_runtime.dataset_provenance import (
    DATASET_CONTENT_SCHEMA_VERSION,
    LEROBOT_DATA_PATH,
    LEROBOT_VIDEO_PATH,
    DatasetContentDigest,
    DatasetContentError,
    compute_dataset_content_digest,
    verify_dataset_content_digest,
)
from franka_runtime.provenance import ProvenanceError, hash_dataset_manifest


def _write_dataset(root: Path, *, episodes: int = 2, video: bool = False) -> None:
    features = {
        "observation.state": {"dtype": "float32", "shape": [8]},
        "action": {"dtype": "float32", "shape": [8]},
    }
    if video:
        features["observation.images.exterior"] = {"dtype": "video", "shape": [240, 320, 3]}
    else:
        features["observation.images.exterior"] = {"dtype": "image", "shape": [240, 320, 3]}
    info = {
        "codebase_version": "v2.1",
        "chunks_size": 1000,
        "total_episodes": episodes,
        "data_path": LEROBOT_DATA_PATH,
        # Pinned LeRobot writes this default even for image-only datasets.
        "video_path": LEROBOT_VIDEO_PATH,
        "features": features,
    }
    meta = root / "meta"
    meta.mkdir(parents=True)
    (meta / "info.json").write_text(json.dumps(info, sort_keys=True) + "\n", encoding="utf-8")
    (meta / "episodes.jsonl").write_text("{}\n" * episodes, encoding="utf-8")
    (meta / "episodes_stats.jsonl").write_text("{}\n" * episodes, encoding="utf-8")
    (meta / "tasks.jsonl").write_text('{"task_index":0,"task":"pick"}\n', encoding="utf-8")
    for episode_index in range(episodes):
        data_path = root / LEROBOT_DATA_PATH.format(episode_chunk=0, episode_index=episode_index)
        data_path.parent.mkdir(parents=True, exist_ok=True)
        data_path.write_bytes(f"parquet-{episode_index}".encode())
        if video:
            video_path = root / LEROBOT_VIDEO_PATH.format(
                episode_chunk=0,
                episode_index=episode_index,
                video_key="observation.images.exterior",
            )
            video_path.parent.mkdir(parents=True, exist_ok=True)
            video_path.write_bytes(f"mp4-{episode_index}".encode())


def test_content_digest_is_deterministic_and_excludes_root_manifest(tmp_path: Path) -> None:
    _write_dataset(tmp_path)

    first = compute_dataset_content_digest(tmp_path)
    (tmp_path / "franka_manifest.json").write_text("first manifest\n", encoding="utf-8")
    second = compute_dataset_content_digest(tmp_path)
    (tmp_path / "franka_manifest.json").write_text("replacement manifest\n", encoding="utf-8")
    third = compute_dataset_content_digest(tmp_path)

    assert first == second == third
    assert first.file_count == 6
    assert first.schema_version == DATASET_CONTENT_SCHEMA_VERSION


def test_content_digest_binds_raw_bytes_including_line_endings(tmp_path: Path) -> None:
    _write_dataset(tmp_path)
    expected = compute_dataset_content_digest(tmp_path)

    tasks = tmp_path / "meta" / "tasks.jsonl"
    tasks.write_bytes(tasks.read_bytes().replace(b"\n", b"\r\n"))

    actual = compute_dataset_content_digest(tmp_path)
    assert actual.sha256 != expected.sha256
    with pytest.raises(DatasetContentError, match="dataset content digest mismatch"):
        verify_dataset_content_digest(tmp_path, expected)


def test_training_manifest_verifier_checks_bound_payload(tmp_path: Path) -> None:
    _write_dataset(tmp_path, episodes=1)
    manifest = {
        "schema_id": "franka-runtime/v1",
        "repo_id": "local/franka_gello",
        "task": "pick",
        "fps": 20,
        "color_space": "rgb_uint8_hwc",
        "state": "q7_rad + gripper_closed",
        "action": "absolute_q7_rad + gripper_closed",
        "camera_roles": {"exterior": "base_rgb", "wrist": "wrist_rgb"},
        "episodes": [],
        "content_digest": compute_dataset_content_digest(tmp_path).to_mapping(),
    }
    manifest_path = tmp_path / "franka_manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    hash_dataset_manifest(manifest_path, expected_repo_id="local/franka_gello", verify_content=True)
    (tmp_path / "data" / "chunk-000" / "episode_000000.parquet").write_bytes(b"tampered")
    with pytest.raises(ProvenanceError, match="invalid dataset content provenance"):
        hash_dataset_manifest(manifest_path, expected_repo_id="local/franka_gello", verify_content=True)


@pytest.mark.parametrize(
    "relative_path",
    [
        "meta/unreviewed.json",
        "data/chunk-000/episode_999999.parquet",
        "images/observation.images.exterior/frame_000000.png",
    ],
)
def test_content_digest_rejects_unknown_or_temporary_payload(tmp_path: Path, relative_path: str) -> None:
    _write_dataset(tmp_path)
    path = tmp_path.joinpath(*relative_path.split("/"))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"unexpected")

    with pytest.raises(DatasetContentError, match="unknown_files|temporary images"):
        compute_dataset_content_digest(tmp_path)


def test_content_digest_rejects_missing_expected_parquet(tmp_path: Path) -> None:
    _write_dataset(tmp_path, episodes=1)
    (tmp_path / "data" / "chunk-000" / "episode_000000.parquet").unlink()

    with pytest.raises(DatasetContentError, match="missing_files"):
        compute_dataset_content_digest(tmp_path)


@pytest.mark.parametrize(
    ("video", "relative_directory"),
    [
        (False, "meta/unreviewed"),
        (False, "data/chunk-000/unreviewed"),
        (True, "videos/chunk-000/unreviewed"),
    ],
)
def test_content_digest_rejects_unknown_empty_directories(tmp_path: Path, video: bool, relative_directory: str) -> None:
    _write_dataset(tmp_path, episodes=1, video=video)
    tmp_path.joinpath(*relative_directory.split("/")).mkdir(parents=True)

    with pytest.raises(DatasetContentError, match="unknown_directories"):
        compute_dataset_content_digest(tmp_path)


def test_content_digest_rejects_symlinked_payload(tmp_path: Path) -> None:
    _write_dataset(tmp_path, episodes=1)
    target = tmp_path / "outside.parquet"
    target.write_bytes(b"outside")
    payload = tmp_path / "data" / "chunk-000" / "episode_000000.parquet"
    payload.unlink()
    try:
        os.symlink(target, payload)
    except OSError as exc:
        pytest.skip(f"test host cannot create symlinks: {exc}")

    with pytest.raises(DatasetContentError, match="symbolic links"):
        compute_dataset_content_digest(tmp_path)


def test_video_payload_is_allowed_only_for_video_features(tmp_path: Path) -> None:
    _write_dataset(tmp_path, episodes=1, video=True)

    digest = compute_dataset_content_digest(tmp_path)

    assert digest.file_count == 6

    info_path = tmp_path / "meta" / "info.json"
    info = json.loads(info_path.read_text(encoding="utf-8"))
    info["features"]["observation.images.exterior"]["dtype"] = "image"
    info_path.write_text(json.dumps(info), encoding="utf-8")
    with pytest.raises(DatasetContentError, match="videos/ is forbidden"):
        compute_dataset_content_digest(tmp_path)


def test_content_digest_detects_declared_video_byte_mutation(tmp_path: Path) -> None:
    _write_dataset(tmp_path, episodes=1, video=True)
    expected = compute_dataset_content_digest(tmp_path)
    video = tmp_path / LEROBOT_VIDEO_PATH.format(
        episode_chunk=0,
        episode_index=0,
        video_key="observation.images.exterior",
    )
    video.write_bytes(b"Mp4-0")

    with pytest.raises(DatasetContentError, match="dataset content digest mismatch"):
        verify_dataset_content_digest(tmp_path, expected)


@pytest.mark.parametrize("mutation", ["missing", "unknown"])
def test_content_digest_schema_is_closed(tmp_path: Path, mutation: str) -> None:
    _write_dataset(tmp_path, episodes=1)
    value = compute_dataset_content_digest(tmp_path).to_mapping()
    if mutation == "missing":
        del value["total_bytes"]
    else:
        value["unreviewed"] = True

    with pytest.raises(DatasetContentError, match="missing keys|unknown keys"):
        DatasetContentDigest.from_mapping(value)
