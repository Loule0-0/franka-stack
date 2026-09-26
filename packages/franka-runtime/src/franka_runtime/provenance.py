"""Strict provenance contract for deployable Franka checkpoints."""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping

from franka_runtime.contracts import (
    ACTION_DIM,
    POLICY_SCHEMA_VERSION,
    ContractError,
    PolicyMetadata,
    validate_policy_metadata,
)
from franka_runtime.dataset_provenance import (
    FRANKA_DATASET_MANIFEST_KEYS,
    DatasetContentDigest,
    verify_dataset_content_digest,
)

CHECKPOINT_PROVENANCE_FILENAME = "franka_provenance.json"
CHECKPOINT_PROVENANCE_SCHEMA_VERSION = "franka-checkpoint-provenance/v2"
FRANKA_TRANSFORM_ID = "panda-absolute-q8-arm-delta-v1"
FRANKA_MODEL_TYPE = "pi05"
ORBAX_MODEL_ARTIFACT_FORMAT = "orbax-params/v1"
SAFETENSORS_MODEL_ARTIFACT_FORMAT = "safetensors/v1"
EMPTY_SHA256 = hashlib.sha256(b"").hexdigest()

_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_GIT_COMMIT_PATTERN = re.compile(r"^[0-9a-f]{40}(?:[0-9a-f]{24})?$")
_NAME_PART_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


class ProvenanceError(ContractError):
    """Raised when a Franka checkpoint cannot prove its training provenance."""


def _require_nonempty_string(name: str, value: object) -> str:
    if not isinstance(value, str) or not value:
        raise ProvenanceError(f"{name} must be a non-empty string")
    return value


def _require_plain_positive_int(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ProvenanceError(f"{name} must be a positive integer")
    return value


def _require_sha256(name: str, value: object) -> str:
    if not isinstance(value, str) or _SHA256_PATTERN.fullmatch(value) is None:
        raise ProvenanceError(f"{name} must be a lowercase SHA-256 hex digest")
    return value


def _require_safe_identifier(name: str, value: object, *, exact_parts: int | None = None) -> str:
    result = _require_nonempty_string(name, value)
    if "\\" in result or result.startswith("/") or result.endswith("/"):
        raise ProvenanceError(f"{name} must be a safe relative POSIX identifier")
    parts = result.split("/")
    if exact_parts is not None and len(parts) != exact_parts:
        raise ProvenanceError(f"{name} must contain exactly {exact_parts} path components")
    if any(_NAME_PART_PATTERN.fullmatch(part) is None for part in parts):
        raise ProvenanceError(f"{name} contains an invalid path component")
    return result


@dataclass(frozen=True)
class SourceFingerprint:
    """The exact Git source state used by a training or serving process."""

    source_commit: str
    source_diff_sha256: str

    def __post_init__(self) -> None:
        if not isinstance(self.source_commit, str) or _GIT_COMMIT_PATTERN.fullmatch(self.source_commit) is None:
            raise ProvenanceError("source_commit must be a lowercase 40- or 64-character Git object id")
        _require_sha256("source_diff_sha256", self.source_diff_sha256)


@dataclass(frozen=True)
class CheckpointProvenance:
    """The complete, closed schema stored beside Franka normalization stats."""

    schema_version: str
    train_config_name: str
    dataset_repo_id: str
    asset_id: str
    model_type: str
    model_action_dim: int
    model_action_horizon: int
    transform_id: str
    policy_metadata: PolicyMetadata
    dataset_manifest_sha256: str
    norm_stats_sha256: str
    model_artifact_format: str
    model_artifacts_sha256: str
    source_commit: str
    source_diff_sha256: str

    def __post_init__(self) -> None:
        if self.schema_version != CHECKPOINT_PROVENANCE_SCHEMA_VERSION:
            raise ProvenanceError(
                f"schema_version must be {CHECKPOINT_PROVENANCE_SCHEMA_VERSION!r}, got {self.schema_version!r}"
            )
        _require_safe_identifier("train_config_name", self.train_config_name, exact_parts=1)
        _require_safe_identifier("dataset_repo_id", self.dataset_repo_id, exact_parts=2)
        _require_safe_identifier("asset_id", self.asset_id)
        if self.model_type != FRANKA_MODEL_TYPE:
            raise ProvenanceError(f"model_type must be {FRANKA_MODEL_TYPE!r}, got {self.model_type!r}")
        model_action_dim = _require_plain_positive_int("model_action_dim", self.model_action_dim)
        model_action_horizon = _require_plain_positive_int("model_action_horizon", self.model_action_horizon)
        if model_action_dim < ACTION_DIM:
            raise ProvenanceError(f"model_action_dim must be at least {ACTION_DIM}, got {model_action_dim}")
        if self.transform_id != FRANKA_TRANSFORM_ID:
            raise ProvenanceError(f"transform_id must be {FRANKA_TRANSFORM_ID!r}, got {self.transform_id!r}")
        if not isinstance(self.policy_metadata, PolicyMetadata):
            raise ProvenanceError("policy_metadata must be validated PolicyMetadata")
        if model_action_horizon != self.policy_metadata.action_horizon:
            raise ProvenanceError(
                "model_action_horizon must equal policy_metadata.action_horizon "
                f"({self.policy_metadata.action_horizon}), got {model_action_horizon}"
            )
        _require_sha256("dataset_manifest_sha256", self.dataset_manifest_sha256)
        _require_sha256("norm_stats_sha256", self.norm_stats_sha256)
        if self.model_artifact_format not in {
            ORBAX_MODEL_ARTIFACT_FORMAT,
            SAFETENSORS_MODEL_ARTIFACT_FORMAT,
        }:
            raise ProvenanceError(f"unsupported model_artifact_format: {self.model_artifact_format!r}")
        _require_sha256("model_artifacts_sha256", self.model_artifacts_sha256)
        SourceFingerprint(self.source_commit, self.source_diff_sha256)

    @classmethod
    def create(
        cls,
        *,
        train_config_name: str,
        dataset_repo_id: str,
        asset_id: str,
        model_type: str,
        model_action_dim: int,
        model_action_horizon: int,
        policy_metadata: Mapping[str, Any] | PolicyMetadata,
        dataset_manifest_sha256: str,
        norm_stats_sha256: str,
        model_artifact_format: str,
        model_artifacts_sha256: str,
        source_fingerprint: SourceFingerprint,
    ) -> CheckpointProvenance:
        return cls(
            schema_version=CHECKPOINT_PROVENANCE_SCHEMA_VERSION,
            train_config_name=train_config_name,
            dataset_repo_id=dataset_repo_id,
            asset_id=asset_id,
            model_type=model_type,
            model_action_dim=model_action_dim,
            model_action_horizon=model_action_horizon,
            transform_id=FRANKA_TRANSFORM_ID,
            policy_metadata=validate_policy_metadata(policy_metadata),
            dataset_manifest_sha256=dataset_manifest_sha256,
            norm_stats_sha256=norm_stats_sha256,
            model_artifact_format=model_artifact_format,
            model_artifacts_sha256=model_artifacts_sha256,
            source_commit=source_fingerprint.source_commit,
            source_diff_sha256=source_fingerprint.source_diff_sha256,
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> CheckpointProvenance:
        if not isinstance(value, Mapping):
            raise ProvenanceError(f"checkpoint provenance must be a mapping, got {type(value).__name__}")
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
            raise ProvenanceError("invalid checkpoint provenance (" + "; ".join(parts) + ")")
        try:
            policy_metadata = validate_policy_metadata(value["policy_metadata"])
        except ContractError as exc:
            raise ProvenanceError(f"invalid policy_metadata: {exc}") from exc
        return cls(
            schema_version=value["schema_version"],
            train_config_name=value["train_config_name"],
            dataset_repo_id=value["dataset_repo_id"],
            asset_id=value["asset_id"],
            model_type=value["model_type"],
            model_action_dim=value["model_action_dim"],
            model_action_horizon=value["model_action_horizon"],
            transform_id=value["transform_id"],
            policy_metadata=policy_metadata,
            dataset_manifest_sha256=value["dataset_manifest_sha256"],
            norm_stats_sha256=value["norm_stats_sha256"],
            model_artifact_format=value["model_artifact_format"],
            model_artifacts_sha256=value["model_artifacts_sha256"],
            source_commit=value["source_commit"],
            source_diff_sha256=value["source_diff_sha256"],
        )

    def to_mapping(self) -> dict[str, object]:
        result = asdict(self)
        result["policy_metadata"] = self.policy_metadata.to_mapping()
        return result


def is_franka_policy_metadata(value: object) -> bool:
    """Return whether metadata opts into the strict Franka runtime contract."""

    return isinstance(value, PolicyMetadata) or (
        isinstance(value, Mapping) and value.get("schema_version") == POLICY_SCHEMA_VERSION
    )


def validate_dataset_repo_id(value: object) -> str:
    """Validate a two-component LeRobot repository id."""

    return _require_safe_identifier("dataset_repo_id", value, exact_parts=2)


def validate_asset_id(value: object) -> str:
    """Validate a checkpoint-relative normalization-stat asset id."""

    return _require_safe_identifier("asset_id", value)


def sha256_file(path: os.PathLike[str] | str) -> str:
    """Hash a regular file without loading it all into memory."""

    resolved = Path(path)
    if not resolved.is_file():
        raise ProvenanceError(f"required provenance input is not a regular file: {resolved}")
    digest = hashlib.sha256()
    with resolved.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hash_model_artifacts(checkpoint_dir: os.PathLike[str] | str) -> tuple[str, str]:
    """Hash the exact model artifact selected by ``create_trained_policy``.

    A checkpoint must contain exactly one supported representation. Rejecting a
    directory that contains both prevents an injected ``model.safetensors``
    from silently taking precedence over the provenance-bound Orbax tree.
    """

    root = Path(checkpoint_dir)
    params_dir = root / "params"
    safetensors_path = root / "model.safetensors"
    has_params = params_dir.is_dir()
    has_safetensors = safetensors_path.is_file()
    if has_params == has_safetensors:
        raise ProvenanceError("Franka checkpoint must contain exactly one model artifact: params/ or model.safetensors")

    if has_safetensors:
        if safetensors_path.is_symlink():
            raise ProvenanceError("Franka model.safetensors must not be a symbolic link")
        files = [safetensors_path]
        artifact_format = SAFETENSORS_MODEL_ARTIFACT_FORMAT
    else:
        if params_dir.is_symlink():
            raise ProvenanceError("Franka params directory must not be a symbolic link")
        files = sorted(
            (path for path in params_dir.rglob("*") if path.is_file() or path.is_symlink()),
            key=lambda path: path.relative_to(root).as_posix(),
        )
        if not files:
            raise ProvenanceError("Franka params directory contains no model artifact files")
        if any(path.is_symlink() for path in files):
            raise ProvenanceError("Franka params model artifacts must not contain symbolic links")
        artifact_format = ORBAX_MODEL_ARTIFACT_FORMAT

    digest = hashlib.sha256()
    digest.update(b"franka-model-artifacts-v1\0")
    for path in files:
        relative_path = path.relative_to(root).as_posix().encode("utf-8")
        _update_length_prefixed(digest, relative_path)
        size = path.stat().st_size
        digest.update(size.to_bytes(8, byteorder="big"))
        bytes_read = 0
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                bytes_read += len(chunk)
                digest.update(chunk)
        if bytes_read != size:
            raise ProvenanceError(f"model artifact changed while hashing: {path}")
    return artifact_format, digest.hexdigest()


def hash_dataset_manifest(
    path: os.PathLike[str] | str,
    *,
    expected_repo_id: str,
    verify_content: bool = False,
) -> str:
    """Validate a closed manifest, optionally verify its payload, and hash its exact bytes."""

    expected_repo_id = validate_dataset_repo_id(expected_repo_id)
    manifest_path = Path(path)
    try:
        raw = manifest_path.read_bytes()
    except OSError as exc:
        raise ProvenanceError(f"cannot read dataset manifest {manifest_path}: {exc}") from exc
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProvenanceError(f"dataset manifest is not valid UTF-8 JSON: {manifest_path}") from exc
    if not isinstance(value, dict):
        raise ProvenanceError("dataset manifest must be a JSON object")
    actual_keys = set(value)
    if actual_keys != FRANKA_DATASET_MANIFEST_KEYS:
        missing = sorted(FRANKA_DATASET_MANIFEST_KEYS - actual_keys)
        unknown = sorted(repr(key) for key in actual_keys - FRANKA_DATASET_MANIFEST_KEYS)
        raise ProvenanceError(
            f"dataset manifest fields must exactly match the contract; missing={missing}, unknown={unknown}"
        )
    if value.get("schema_id") != POLICY_SCHEMA_VERSION:
        raise ProvenanceError(
            f"dataset manifest schema_id must be {POLICY_SCHEMA_VERSION!r}, got {value.get('schema_id')!r}"
        )
    if value.get("repo_id") != expected_repo_id:
        raise ProvenanceError(f"dataset manifest repo_id must be {expected_repo_id!r}, got {value.get('repo_id')!r}")
    try:
        content_digest = DatasetContentDigest.from_mapping(value["content_digest"])
        if verify_content:
            verify_dataset_content_digest(manifest_path.parent, content_digest)
    except ContractError as exc:
        raise ProvenanceError(f"invalid dataset content provenance: {exc}") from exc
    return hashlib.sha256(raw).hexdigest()


def _run_git(repository: Path, *args: str) -> bytes:
    try:
        result = subprocess.run(
            ["git", "-c", "core.quotepath=false", "-C", str(repository), *args],
            check=True,
            capture_output=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        detail = ""
        if isinstance(exc, subprocess.CalledProcessError):
            detail = exc.stderr.decode("utf-8", errors="replace").strip()
        suffix = f": {detail}" if detail else ""
        raise ProvenanceError(f"cannot inspect Git source provenance{suffix}") from exc
    return result.stdout


def _update_length_prefixed(digest: Any, value: bytes) -> None:
    digest.update(len(value).to_bytes(8, byteorder="big"))
    digest.update(value)


def _source_diff_sha256(repository: Path) -> str:
    tracked_diff = _run_git(
        repository, "diff", "--binary", "--no-color", "--no-ext-diff", "--no-textconv", "HEAD", "--"
    )
    untracked_paths = sorted(
        path for path in _run_git(repository, "ls-files", "--others", "--exclude-standard", "-z").split(b"\0") if path
    )
    if not tracked_diff and not untracked_paths:
        return EMPTY_SHA256

    digest = hashlib.sha256()
    digest.update(b"franka-source-diff-v1\0")
    _update_length_prefixed(digest, tracked_diff)
    for raw_relative_path in untracked_paths:
        relative_path = Path(os.fsdecode(raw_relative_path))
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise ProvenanceError(f"Git returned unsafe untracked path: {relative_path}")
        full_path = repository / relative_path
        _update_length_prefixed(digest, raw_relative_path)
        if full_path.is_symlink():
            digest.update(b"symlink\0")
            _update_length_prefixed(digest, os.fsencode(os.readlink(full_path)))
        elif full_path.is_file():
            digest.update(b"file\0")
            size = full_path.stat().st_size
            digest.update(size.to_bytes(8, byteorder="big"))
            bytes_read = 0
            with full_path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    bytes_read += len(chunk)
                    digest.update(chunk)
            if bytes_read != size:
                raise ProvenanceError(f"untracked source file changed while hashing: {relative_path}")
        else:
            raise ProvenanceError(f"unsupported untracked source path: {relative_path}")
    return digest.hexdigest()


def capture_source_fingerprint(start_path: os.PathLike[str] | str) -> SourceFingerprint:
    """Capture the repository and, for a submodule, its parent checkout state."""

    start = Path(start_path).resolve()
    if start.is_file():
        start = start.parent
    root_bytes = _run_git(start, "rev-parse", "--show-toplevel")
    repository = Path(os.fsdecode(root_bytes.strip())).resolve()
    commit = _run_git(repository, "rev-parse", "--verify", "HEAD").decode("ascii").strip()
    repository_diff = _source_diff_sha256(repository)

    superproject_bytes = _run_git(repository, "rev-parse", "--show-superproject-working-tree").strip()
    if not superproject_bytes:
        return SourceFingerprint(source_commit=commit, source_diff_sha256=repository_diff)

    superproject = Path(os.fsdecode(superproject_bytes)).resolve()
    superproject_commit = _run_git(superproject, "rev-parse", "--verify", "HEAD").decode("ascii").strip()
    superproject_diff = _source_diff_sha256(superproject)
    try:
        submodule_path = repository.relative_to(superproject).as_posix()
    except ValueError as exc:
        raise ProvenanceError("Git reported a superproject that does not contain the source repository") from exc
    digest = hashlib.sha256()
    digest.update(b"franka-source-superproject-v1\0")
    for value in (
        submodule_path.encode("utf-8"),
        repository_diff.encode("ascii"),
        superproject_commit.encode("ascii"),
        superproject_diff.encode("ascii"),
    ):
        _update_length_prefixed(digest, value)
    return SourceFingerprint(source_commit=commit, source_diff_sha256=digest.hexdigest())


def write_checkpoint_provenance(assets_dir: os.PathLike[str] | str, value: CheckpointProvenance) -> Path:
    """Atomically write the canonical checkpoint provenance JSON."""

    directory = Path(assets_dir)
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / CHECKPOINT_PROVENANCE_FILENAME
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(json.dumps(value.to_mapping(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(destination)
    return destination


def load_checkpoint_provenance(checkpoint_dir: os.PathLike[str] | str) -> CheckpointProvenance:
    """Load and strictly validate a checkpoint-root provenance manifest."""

    path = Path(checkpoint_dir) / "assets" / CHECKPOINT_PROVENANCE_FILENAME
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ProvenanceError(f"missing Franka checkpoint provenance: {path}") from exc
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProvenanceError(f"cannot read Franka checkpoint provenance {path}: {exc}") from exc
    return CheckpointProvenance.from_mapping(value)


def verify_checkpoint_provenance(
    checkpoint_dir: os.PathLike[str] | str,
    *,
    train_config_name: str,
    dataset_repo_id: str,
    asset_id: str,
    model_type: str,
    model_action_dim: int,
    model_action_horizon: int,
    policy_metadata: Mapping[str, Any] | PolicyMetadata,
    source_fingerprint: SourceFingerprint,
    dataset_manifest_sha256: str | None = None,
) -> CheckpointProvenance:
    """Fail unless a checkpoint exactly matches the selected Franka runtime."""

    provenance = load_checkpoint_provenance(checkpoint_dir)
    expected_metadata = validate_policy_metadata(policy_metadata)
    expected_values: dict[str, object] = {
        "train_config_name": train_config_name,
        "dataset_repo_id": dataset_repo_id,
        "asset_id": asset_id,
        "model_type": model_type,
        "model_action_dim": model_action_dim,
        "model_action_horizon": model_action_horizon,
        "transform_id": FRANKA_TRANSFORM_ID,
        "policy_metadata": expected_metadata,
        "source_commit": source_fingerprint.source_commit,
        "source_diff_sha256": source_fingerprint.source_diff_sha256,
    }
    if dataset_manifest_sha256 is not None:
        expected_values["dataset_manifest_sha256"] = _require_sha256("dataset_manifest_sha256", dataset_manifest_sha256)
    for field_name, expected in expected_values.items():
        actual = getattr(provenance, field_name)
        if actual != expected:
            raise ProvenanceError(
                f"checkpoint provenance mismatch for {field_name}: expected {expected!r}, got {actual!r}"
            )

    norm_stats_path = Path(checkpoint_dir) / "assets" / provenance.asset_id / "norm_stats.json"
    actual_norm_stats_sha256 = sha256_file(norm_stats_path)
    if actual_norm_stats_sha256 != provenance.norm_stats_sha256:
        raise ProvenanceError(
            "checkpoint normalization statistics do not match provenance: "
            f"expected {provenance.norm_stats_sha256}, got {actual_norm_stats_sha256}"
        )
    actual_model_artifact_format, actual_model_artifacts_sha256 = hash_model_artifacts(checkpoint_dir)
    if actual_model_artifact_format != provenance.model_artifact_format:
        raise ProvenanceError(
            "checkpoint model artifact format does not match provenance: "
            f"expected {provenance.model_artifact_format}, got {actual_model_artifact_format}"
        )
    if actual_model_artifacts_sha256 != provenance.model_artifacts_sha256:
        raise ProvenanceError(
            "checkpoint model artifacts do not match provenance: "
            f"expected {provenance.model_artifacts_sha256}, got {actual_model_artifacts_sha256}"
        )
    return provenance
