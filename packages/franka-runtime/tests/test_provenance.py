from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

from franka_runtime.contracts import EXPECTED_POLICY_METADATA
from franka_runtime.provenance import (
    CHECKPOINT_PROVENANCE_FILENAME,
    EMPTY_SHA256,
    CheckpointProvenance,
    ProvenanceError,
    SourceFingerprint,
    capture_source_fingerprint,
    hash_dataset_manifest,
    hash_model_artifacts,
    is_franka_policy_metadata,
    load_checkpoint_provenance,
    sha256_file,
    verify_checkpoint_provenance,
    write_checkpoint_provenance,
)

REPO_ID = "local/franka_gello"
ASSET_ID = REPO_ID
CONFIG_NAME = "pi05_franka_jointpos"
SOURCE = SourceFingerprint(source_commit="a" * 40, source_diff_sha256=EMPTY_SHA256)


def _write_checkpoint(
    root: Path, *, source_fingerprint: SourceFingerprint = SOURCE
) -> tuple[Path, CheckpointProvenance]:
    checkpoint_dir = root / "checkpoint"
    norm_stats_path = checkpoint_dir / "assets" / ASSET_ID / "norm_stats.json"
    norm_stats_path.parent.mkdir(parents=True)
    norm_stats_path.write_text('{"norm_stats": {}}\n', encoding="utf-8")
    params_path = checkpoint_dir / "params" / "weights"
    params_path.parent.mkdir(parents=True)
    params_path.write_bytes(b"model weights")
    model_artifact_format, model_artifacts_sha256 = hash_model_artifacts(checkpoint_dir)
    provenance = CheckpointProvenance.create(
        train_config_name=CONFIG_NAME,
        dataset_repo_id=REPO_ID,
        asset_id=ASSET_ID,
        model_type="pi05",
        model_action_dim=32,
        model_action_horizon=20,
        policy_metadata=EXPECTED_POLICY_METADATA,
        dataset_manifest_sha256="b" * 64,
        norm_stats_sha256=sha256_file(norm_stats_path),
        model_artifact_format=model_artifact_format,
        model_artifacts_sha256=model_artifacts_sha256,
        source_fingerprint=source_fingerprint,
    )
    write_checkpoint_provenance(checkpoint_dir / "assets", provenance)
    return checkpoint_dir, provenance


def _verification_kwargs(source_fingerprint: SourceFingerprint = SOURCE) -> dict[str, object]:
    return {
        "train_config_name": CONFIG_NAME,
        "dataset_repo_id": REPO_ID,
        "asset_id": ASSET_ID,
        "model_type": "pi05",
        "model_action_dim": 32,
        "model_action_horizon": 20,
        "policy_metadata": EXPECTED_POLICY_METADATA,
        "source_fingerprint": source_fingerprint,
    }


def _git(repository: Path, *args: str) -> str:
    result = subprocess.run(["git", "-C", str(repository), *args], check=True, capture_output=True)
    return result.stdout.decode("utf-8").strip()


def test_checkpoint_provenance_round_trip_and_verification(tmp_path: Path) -> None:
    checkpoint_dir, provenance = _write_checkpoint(tmp_path)

    assert load_checkpoint_provenance(checkpoint_dir) == provenance
    assert verify_checkpoint_provenance(checkpoint_dir, **_verification_kwargs()) == provenance


def test_missing_checkpoint_provenance_fails_closed(tmp_path: Path) -> None:
    with pytest.raises(ProvenanceError, match="missing Franka checkpoint provenance"):
        verify_checkpoint_provenance(tmp_path, **_verification_kwargs())


def test_non_franka_metadata_does_not_opt_in() -> None:
    assert not is_franka_policy_metadata(None)
    assert not is_franka_policy_metadata({"reset_pose": [0.0] * 7})
    assert is_franka_policy_metadata(EXPECTED_POLICY_METADATA.to_mapping())


@pytest.mark.parametrize("mutation", ["missing", "unknown"])
def test_checkpoint_provenance_rejects_open_schema(tmp_path: Path, mutation: str) -> None:
    checkpoint_dir, _ = _write_checkpoint(tmp_path)
    path = checkpoint_dir / "assets" / CHECKPOINT_PROVENANCE_FILENAME
    value = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "missing":
        del value["transform_id"]
    else:
        value["unreviewed_override"] = True
    path.write_text(json.dumps(value), encoding="utf-8")

    with pytest.raises(ProvenanceError, match="missing keys|unknown keys"):
        load_checkpoint_provenance(checkpoint_dir)


def test_checkpoint_provenance_rejects_velocity_policy_metadata(tmp_path: Path) -> None:
    checkpoint_dir, _ = _write_checkpoint(tmp_path)
    path = checkpoint_dir / "assets" / CHECKPOINT_PROVENANCE_FILENAME
    value = json.loads(path.read_text(encoding="utf-8"))
    value["policy_metadata"]["action_semantics"] = "normalized_joint_velocity[7]+gripper[1]"
    path.write_text(json.dumps(value), encoding="utf-8")

    with pytest.raises(ProvenanceError, match="invalid policy_metadata.*action_semantics"):
        load_checkpoint_provenance(checkpoint_dir)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"train_config_name": "pi05_franka_other"}, "train_config_name"),
        ({"dataset_repo_id": "local/other"}, "dataset_repo_id"),
        ({"asset_id": "franka"}, "asset_id"),
        ({"model_action_horizon": 19}, "model_action_horizon"),
        ({"dataset_manifest_sha256": "c" * 64}, "dataset_manifest_sha256"),
        (
            {"source_fingerprint": SourceFingerprint("c" * 40, EMPTY_SHA256)},
            "source_commit",
        ),
    ],
)
def test_checkpoint_provenance_rejects_expected_value_mismatch(
    tmp_path: Path, change: dict[str, object], message: str
) -> None:
    checkpoint_dir, _ = _write_checkpoint(tmp_path)
    kwargs = _verification_kwargs()
    kwargs.update(change)

    with pytest.raises(ProvenanceError, match=message):
        verify_checkpoint_provenance(checkpoint_dir, **kwargs)


def test_checkpoint_provenance_rejects_modified_norm_stats(tmp_path: Path) -> None:
    checkpoint_dir, _ = _write_checkpoint(tmp_path)
    norm_stats_path = checkpoint_dir / "assets" / ASSET_ID / "norm_stats.json"
    norm_stats_path.write_text('{"norm_stats": {"action": "tampered"}}\n', encoding="utf-8")

    with pytest.raises(ProvenanceError, match="normalization statistics do not match"):
        verify_checkpoint_provenance(checkpoint_dir, **_verification_kwargs())


def test_checkpoint_provenance_rejects_modified_model_artifact(tmp_path: Path) -> None:
    checkpoint_dir, _ = _write_checkpoint(tmp_path)
    (checkpoint_dir / "params" / "weights").write_bytes(b"different model weights")

    with pytest.raises(ProvenanceError, match="model artifacts do not match"):
        verify_checkpoint_provenance(checkpoint_dir, **_verification_kwargs())


def test_checkpoint_provenance_rejects_safetensors_injection(tmp_path: Path) -> None:
    checkpoint_dir, _ = _write_checkpoint(tmp_path)
    (checkpoint_dir / "model.safetensors").write_bytes(b"injected model")

    with pytest.raises(ProvenanceError, match="exactly one model artifact"):
        verify_checkpoint_provenance(checkpoint_dir, **_verification_kwargs())


def test_safetensors_checkpoint_artifact_hash_is_deterministic(tmp_path: Path) -> None:
    checkpoint_dir = tmp_path / "checkpoint"
    checkpoint_dir.mkdir()
    (checkpoint_dir / "model.safetensors").write_bytes(b"model")

    first = hash_model_artifacts(checkpoint_dir)
    second = hash_model_artifacts(checkpoint_dir)

    assert first == second
    assert first[0] == "safetensors/v1"


def test_dataset_manifest_hash_binds_schema_and_repo(tmp_path: Path) -> None:
    manifest = tmp_path / "franka_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_id": "franka-runtime/v1",
                "repo_id": REPO_ID,
                "task": "pick up the object",
                "fps": 20,
                "color_space": "rgb_uint8_hwc",
                "state": "q7_rad + gripper_closed",
                "action": "absolute_q7_rad + gripper_closed",
                "camera_roles": {"exterior": "base_rgb", "wrist": "wrist_rgb"},
                "episodes": [],
                "content_digest": {
                    "schema_version": "franka-dataset-content/v1",
                    "algorithm": "sha256",
                    "sha256": "c" * 64,
                    "file_count": 1,
                    "total_bytes": 0,
                },
            }
        ),
        encoding="utf-8",
    )

    assert hash_dataset_manifest(manifest, expected_repo_id=REPO_ID) == sha256_file(manifest)
    with pytest.raises(ProvenanceError, match="repo_id"):
        hash_dataset_manifest(manifest, expected_repo_id="local/other")


def test_source_fingerprint_is_clean_then_detects_tracked_and_untracked_changes(tmp_path: Path) -> None:
    repository = tmp_path / "repository"
    repository.mkdir()
    _git(repository, "init")
    _git(repository, "config", "user.email", "test@example.invalid")
    _git(repository, "config", "user.name", "Test User")
    tracked = repository / "tracked.txt"
    tracked.write_text("initial\n", encoding="utf-8")
    _git(repository, "add", "tracked.txt")
    _git(repository, "commit", "-m", "initial")

    clean = capture_source_fingerprint(tracked)
    assert clean.source_diff_sha256 == EMPTY_SHA256

    tracked.write_text("dirty\n", encoding="utf-8")
    tracked_dirty = capture_source_fingerprint(repository)
    assert tracked_dirty.source_commit == clean.source_commit
    assert tracked_dirty.source_diff_sha256 != EMPTY_SHA256
    assert tracked_dirty.source_diff_sha256 != clean.source_diff_sha256

    tracked.write_text("initial\n", encoding="utf-8")
    (repository / "untracked.txt").write_text("new source\n", encoding="utf-8")
    untracked_dirty = capture_source_fingerprint(repository)
    assert untracked_dirty.source_diff_sha256 != EMPTY_SHA256
    assert untracked_dirty.source_diff_sha256 != tracked_dirty.source_diff_sha256


def test_source_fingerprint_binds_submodule_to_superproject(tmp_path: Path) -> None:
    child_source = tmp_path / "child-source"
    child_source.mkdir()
    _git(child_source, "init")
    _git(child_source, "config", "user.email", "test@example.invalid")
    _git(child_source, "config", "user.name", "Test User")
    (child_source / "policy.py").write_text("POLICY = 1\n", encoding="utf-8")
    _git(child_source, "add", "policy.py")
    _git(child_source, "commit", "-m", "child")

    parent = tmp_path / "parent"
    parent.mkdir()
    _git(parent, "init")
    _git(parent, "config", "user.email", "test@example.invalid")
    _git(parent, "config", "user.name", "Test User")
    _git(parent, "-c", "protocol.file.allow=always", "submodule", "add", str(child_source), "third_party/openpi")
    _git(parent, "commit", "-am", "parent")

    child_file = parent / "third_party" / "openpi" / "policy.py"
    clean = capture_source_fingerprint(child_file)
    assert clean.source_commit == _git(child_file.parent, "rev-parse", "HEAD").strip()
    assert clean.source_diff_sha256 != EMPTY_SHA256

    (parent / "README.md").write_text("parent change\n", encoding="utf-8")
    parent_dirty = capture_source_fingerprint(child_file)
    assert parent_dirty.source_commit == clean.source_commit
    assert parent_dirty.source_diff_sha256 != clean.source_diff_sha256

    (parent / "README.md").unlink()
    child_file.write_text("POLICY = 2\n", encoding="utf-8")
    child_dirty = capture_source_fingerprint(child_file)
    assert child_dirty.source_commit == clean.source_commit
    assert child_dirty.source_diff_sha256 not in {clean.source_diff_sha256, parent_dirty.source_diff_sha256}


def test_source_diff_mismatch_blocks_checkpoint(tmp_path: Path) -> None:
    repository = tmp_path / "repository"
    repository.mkdir()
    _git(repository, "init")
    _git(repository, "config", "user.email", "test@example.invalid")
    _git(repository, "config", "user.name", "Test User")
    tracked = repository / "tracked.txt"
    tracked.write_text("initial\n", encoding="utf-8")
    _git(repository, "add", "tracked.txt")
    _git(repository, "commit", "-m", "initial")
    clean = capture_source_fingerprint(repository)
    checkpoint_dir, _ = _write_checkpoint(tmp_path, source_fingerprint=clean)

    tracked.write_text("changed after training\n", encoding="utf-8")
    dirty = capture_source_fingerprint(repository)
    with pytest.raises(ProvenanceError, match="source_diff_sha256"):
        verify_checkpoint_provenance(checkpoint_dir, **_verification_kwargs(dirty))
