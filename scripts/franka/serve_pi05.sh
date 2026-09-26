#!/usr/bin/env bash
set -Eeuo pipefail

PROGRAM_NAME="${0##*/}"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --project-root PATH --config NAME --checkpoint PATH --dataset-repo-id ID [options]

Serve an explicitly selected checkpoint. The listener binds to loopback by
default; use an SSH tunnel from the robot PC.

Options:
  --host ADDRESS          Bind address (default: 127.0.0.1).
  --port N                TCP port (default: 8000).
  --default-prompt TEXT   Fallback language prompt.
  --dataset-repo-id ID    Exact LeRobot dataset ID used for norm stats.
  --allow-non-loopback    Required if --host is not a loopback address.
  --record                Record policy requests and outputs.
  --state-root PATH       Persistent cache/temp root (recommended).
  -h, --help              Show this help.

Environment equivalents include FRANKA_PROJECT_ROOT, FRANKA_CONFIG,
FRANKA_CHECKPOINT_DIR, and FRANKA_DATASET_REPO_ID.
EOF
}

project_root="${FRANKA_PROJECT_ROOT:-}"
config="${FRANKA_CONFIG:-}"
checkpoint="${FRANKA_CHECKPOINT_DIR:-}"
dataset_repo_id="${FRANKA_DATASET_REPO_ID:-}"
host="${POLICY_HOST:-127.0.0.1}"
port="${POLICY_PORT:-8000}"
default_prompt="${POLICY_PROMPT:-}"
state_root="${FRANKA_SERVE_STATE_ROOT:-}"
allow_non_loopback=false
record=false

while (($#)); do
    case "$1" in
        --project-root) project_root="${2:?missing value for --project-root}"; shift 2 ;;
        --config) config="${2:?missing value for --config}"; shift 2 ;;
        --checkpoint) checkpoint="${2:?missing value for --checkpoint}"; shift 2 ;;
        --dataset-repo-id) dataset_repo_id="${2:?missing value for --dataset-repo-id}"; shift 2 ;;
        --host) host="${2:?missing value for --host}"; shift 2 ;;
        --port) port="${2:?missing value for --port}"; shift 2 ;;
        --default-prompt) default_prompt="${2:?missing value for --default-prompt}"; shift 2 ;;
        --state-root) state_root="${2:?missing value for --state-root}"; shift 2 ;;
        --allow-non-loopback) allow_non_loopback=true; shift ;;
        --record) record=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

openpi_root="$project_root/third_party/openpi"
[[ "$project_root" == /* && -f "$project_root/pyproject.toml" && -f "$openpi_root/pyproject.toml" ]] || {
    echo "ERROR: --project-root must be an absolute recursive checkout path" >&2
    exit 2
}
[[ -n "$config" ]] || { echo "ERROR: --config is required" >&2; exit 2; }
[[ -n "$checkpoint" ]] || { echo "ERROR: --checkpoint is required" >&2; exit 2; }
[[ "$dataset_repo_id" =~ ^[A-Za-z0-9._-]+/[A-Za-z0-9._-]+$ ]] || {
    echo "ERROR: --dataset-repo-id must be an explicit namespace/name ID" >&2
    exit 2
}
[[ "$port" =~ ^[0-9]+$ ]] && ((port >= 1 && port <= 65535)) || { echo "ERROR: invalid port: $port" >&2; exit 2; }

case "$host" in
    127.*|::1|localhost) ;;
    *)
        [[ "$allow_non_loopback" == true ]] || {
            echo "ERROR: non-loopback bind requires --allow-non-loopback; plaintext WebSocket has no authentication" >&2
            exit 2
        }
        ;;
esac

if [[ "$checkpoint" != *://* ]]; then
    [[ "$checkpoint" == /* ]] || { echo "ERROR: local checkpoint path must be absolute" >&2; exit 2; }
    [[ -d "$checkpoint" ]] || { echo "ERROR: checkpoint directory not found: $checkpoint" >&2; exit 1; }
fi
command -v uv >/dev/null 2>&1 || { echo "ERROR: uv not found" >&2; exit 3; }
export FRANKA_DATASET_REPO_ID="$dataset_repo_id"

if [[ -n "$state_root" ]]; then
    [[ "$state_root" == /* ]] || { echo "ERROR: --state-root must be absolute" >&2; exit 2; }
    case "$state_root" in
        /tmp|/tmp/*|/var/tmp|/var/tmp/*) echo "ERROR: --state-root must be persistent" >&2; exit 2 ;;
    esac
    mkdir -p "$state_root/cache" "$state_root/tmp"
    export XDG_CACHE_HOME="$state_root/cache"
    export UV_CACHE_DIR="$state_root/cache/uv"
    export TMPDIR="$state_root/tmp"
    export TEMP="$TMPDIR"
    export TMP="$TMPDIR"
fi

args=(
    python "$openpi_root/scripts/serve_policy.py"
    --host "$host"
    --port "$port"
)
[[ -n "$default_prompt" ]] && args+=(--default-prompt "$default_prompt")
[[ "$record" == true ]] && args+=(--record)
args+=(policy:checkpoint --policy.config "$config" --policy.dir "$checkpoint")

if [[ "$record" == true && -z "$state_root" ]]; then
    echo "ERROR: --record requires --state-root so recordings are written to persistent storage" >&2
    exit 2
fi

echo "Serving ${config} from ${checkpoint} on ${host}:${port} with dataset ${dataset_repo_id}"
if [[ "$host" == "127.0.0.1" || "$host" == "localhost" || "$host" == "::1" ]]; then
    echo "Connect the robot PC through an SSH tunnel; do not expose this plaintext service publicly."
fi
[[ -n "$state_root" ]] && cd "$state_root"
exec uv run --project "$openpi_root" "${args[@]}"
