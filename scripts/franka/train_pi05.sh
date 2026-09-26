#!/usr/bin/env bash
set -Eeuo pipefail

PROGRAM_NAME="${0##*/}"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --project-root PATH --dataset-home PATH \\
  --dataset-repo-id ID --checkpoint-root PATH --state-root PATH \\
  --experiment NAME [options]

Compute dataset normalization statistics, then full-finetune pi0.5. Paths are
explicit so data, stats, and checkpoints stay out of Git and temporary storage.

Options:
  --config NAME           Training config (default: pi05_franka_jointpos).
  --base-checkpoint PATH  Base params path/URI (default: official pi05_base).
  --num-gpus N            Visible GPUs and FSDP devices (default: 4).
  --gpu-devices CSV       Explicit CUDA device list (default: 0..N-1).
  --skip-stats            Reuse existing normalization statistics.
  --max-stat-frames N     Limit frames used for normalization statistics.
  --num-train-steps N     Override the config's positive training-step count.
  --save-interval N       Override the config's positive checkpoint interval.
  --batch-size N          Override the config's positive global batch size.
  --num-workers N         Override the config's non-negative loader worker count.
  --resume                Resume the existing experiment.
  --overwrite             Explicitly replace the existing experiment.
  --disable-wandb         Disable Weights & Biases logging.
  -h, --help              Show this help.

Environment equivalents include FRANKA_PROJECT_ROOT, HF_LEROBOT_HOME,
FRANKA_DATASET_REPO_ID, FRANKA_CHECKPOINT_ROOT, FRANKA_TRAIN_STATE_ROOT,
FRANKA_CONFIG, FRANKA_BASE_CHECKPOINT, FRANKA_EXPERIMENT, and FRANKA_NUM_GPUS.
EOF
}

project_root="${FRANKA_PROJECT_ROOT:-}"
dataset_home="${HF_LEROBOT_HOME:-}"
dataset_repo_id="${FRANKA_DATASET_REPO_ID:-}"
checkpoint_root="${FRANKA_CHECKPOINT_ROOT:-}"
state_root="${FRANKA_TRAIN_STATE_ROOT:-}"
experiment="${FRANKA_EXPERIMENT:-}"
config="${FRANKA_CONFIG:-pi05_franka_jointpos}"
base_checkpoint="${FRANKA_BASE_CHECKPOINT:-gs://openpi-assets/checkpoints/pi05_base/params}"
num_gpus="${FRANKA_NUM_GPUS:-4}"
gpu_devices="${CUDA_VISIBLE_DEVICES:-}"
skip_stats=false
max_stat_frames=""
num_train_steps=""
save_interval=""
batch_size=""
num_workers=""
run_mode=""
wandb_enabled=true

while (($#)); do
    case "$1" in
        --project-root) project_root="${2:?missing value for --project-root}"; shift 2 ;;
        --dataset-home) dataset_home="${2:?missing value for --dataset-home}"; shift 2 ;;
        --dataset-repo-id) dataset_repo_id="${2:?missing value for --dataset-repo-id}"; shift 2 ;;
        --checkpoint-root) checkpoint_root="${2:?missing value for --checkpoint-root}"; shift 2 ;;
        --state-root) state_root="${2:?missing value for --state-root}"; shift 2 ;;
        --experiment) experiment="${2:?missing value for --experiment}"; shift 2 ;;
        --config) config="${2:?missing value for --config}"; shift 2 ;;
        --base-checkpoint) base_checkpoint="${2:?missing value for --base-checkpoint}"; shift 2 ;;
        --num-gpus) num_gpus="${2:?missing value for --num-gpus}"; shift 2 ;;
        --gpu-devices) gpu_devices="${2:?missing value for --gpu-devices}"; shift 2 ;;
        --skip-stats) skip_stats=true; shift ;;
        --max-stat-frames) max_stat_frames="${2:?missing value for --max-stat-frames}"; shift 2 ;;
        --num-train-steps) num_train_steps="${2:?missing value for --num-train-steps}"; shift 2 ;;
        --save-interval) save_interval="${2:?missing value for --save-interval}"; shift 2 ;;
        --batch-size) batch_size="${2:?missing value for --batch-size}"; shift 2 ;;
        --num-workers) num_workers="${2:?missing value for --num-workers}"; shift 2 ;;
        --resume)
            [[ -z "$run_mode" ]] || { echo "ERROR: --resume and --overwrite are mutually exclusive" >&2; exit 2; }
            run_mode="resume"
            shift
            ;;
        --overwrite)
            [[ -z "$run_mode" ]] || { echo "ERROR: --resume and --overwrite are mutually exclusive" >&2; exit 2; }
            run_mode="overwrite"
            shift
            ;;
        --disable-wandb) wandb_enabled=false; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

require_persistent_absolute_path() {
    local label="$1"
    local path="$2"
    [[ -n "$path" ]] || { echo "ERROR: ${label} is required" >&2; exit 2; }
    [[ "$path" == /* ]] || { echo "ERROR: ${label} must be absolute: $path" >&2; exit 2; }
    case "$path" in
        /tmp|/tmp/*|/var/tmp|/var/tmp/*)
            echo "ERROR: ${label} must not be temporary: $path" >&2
            exit 2
            ;;
    esac
}

require_persistent_absolute_path "project root" "$project_root"
require_persistent_absolute_path "dataset home" "$dataset_home"
require_persistent_absolute_path "checkpoint root" "$checkpoint_root"
require_persistent_absolute_path "training state root" "$state_root"
[[ "$dataset_repo_id" =~ ^[A-Za-z0-9._-]+/[A-Za-z0-9._-]+$ ]] || {
    echo "ERROR: --dataset-repo-id must be an explicit namespace/name ID" >&2
    exit 2
}
[[ "$experiment" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ && "$experiment" != "." && "$experiment" != ".." ]] || {
    echo "ERROR: --experiment must be a single safe path component" >&2
    exit 2
}
[[ "$num_gpus" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: --num-gpus must be positive" >&2; exit 2; }
if [[ -n "$max_stat_frames" ]]; then
    [[ "$max_stat_frames" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: --max-stat-frames must be positive" >&2; exit 2; }
fi
for value_name in num_train_steps save_interval batch_size; do
    value="${!value_name}"
    if [[ -n "$value" && ! "$value" =~ ^[1-9][0-9]*$ ]]; then
        echo "ERROR: --${value_name//_/-} must be positive" >&2
        exit 2
    fi
done
if [[ -n "$num_workers" && ! "$num_workers" =~ ^[0-9]+$ ]]; then
    echo "ERROR: --num-workers must be non-negative" >&2
    exit 2
fi
openpi_root="$project_root/third_party/openpi"
[[ -f "$project_root/pyproject.toml" && -f "$openpi_root/pyproject.toml" && -f "$openpi_root/uv.lock" ]] || {
    echo "ERROR: invalid project root or uninitialized OpenPI submodule: $project_root" >&2
    exit 1
}
[[ -d "$dataset_home/$dataset_repo_id" ]] || { echo "ERROR: dataset not found: $dataset_home/$dataset_repo_id" >&2; exit 1; }
command -v uv >/dev/null 2>&1 || { echo "ERROR: uv is not on PATH; run bootstrap_gpu_server.sh first" >&2; exit 3; }
command -v nvidia-smi >/dev/null 2>&1 || { echo "ERROR: nvidia-smi not found" >&2; exit 3; }

gpu_listing="$(nvidia-smi --query-gpu=index --format=csv,noheader)"
available_gpus=0
declare -A available_gpu_devices=()
while IFS= read -r gpu_index; do
    if [[ -n "$gpu_index" ]]; then
        [[ "$gpu_index" =~ ^[0-9]+$ ]] || { echo "ERROR: unexpected GPU index from nvidia-smi: $gpu_index" >&2; exit 1; }
        available_gpu_devices[$gpu_index]=1
        available_gpus=$((available_gpus + 1))
    fi
done <<<"$gpu_listing"
((available_gpus >= num_gpus)) || { echo "ERROR: requested ${num_gpus} GPUs, found ${available_gpus}" >&2; exit 1; }

if [[ -z "$gpu_devices" ]]; then
    for ((gpu_index = 0; gpu_index < num_gpus; gpu_index++)); do
        [[ -z "$gpu_devices" ]] || gpu_devices+=,
        gpu_devices+="$gpu_index"
    done
fi
IFS=',' read -r -a selected_gpus <<<"$gpu_devices"
((${#selected_gpus[@]} == num_gpus)) || {
    echo "ERROR: --gpu-devices selects ${#selected_gpus[@]} devices, expected ${num_gpus}" >&2
    exit 2
}
declare -A seen_gpu_devices=()
for gpu_index in "${selected_gpus[@]}"; do
    [[ "$gpu_index" =~ ^[0-9]+$ ]] || { echo "ERROR: invalid GPU index: $gpu_index" >&2; exit 2; }
    [[ -n "${available_gpu_devices[$gpu_index]+present}" ]] || { echo "ERROR: GPU index is unavailable: $gpu_index" >&2; exit 2; }
    [[ -z "${seen_gpu_devices[$gpu_index]+present}" ]] || { echo "ERROR: duplicate GPU index: $gpu_index" >&2; exit 2; }
    seen_gpu_devices[$gpu_index]=1
done

echo "Current GPU occupancy (inspect before the training process allocates memory):"
nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv

mkdir -p "$checkpoint_root" "$state_root/assets" "$state_root/cache" "$state_root/tmp"
export HF_LEROBOT_HOME="$dataset_home"
export FRANKA_DATASET_REPO_ID="$dataset_repo_id"
export FRANKA_BASE_CHECKPOINT="$base_checkpoint"
export CUDA_VISIBLE_DEVICES="$gpu_devices"
export XDG_CACHE_HOME="$state_root/cache"
export UV_CACHE_DIR="$state_root/cache/uv"
export TMPDIR="$state_root/tmp"
export TEMP="$TMPDIR"
export TMP="$TMPDIR"

echo "Auditing every frame in ${dataset_home}/${dataset_repo_id} before training"
(
    cd "$state_root"
    uv run --project "$openpi_root" python "$project_root/examples/franka_real/audit_dataset.py" \
        --dataset-root "$dataset_home/$dataset_repo_id" \
        --repo-id "$dataset_repo_id" \
        --max-frames 0
)

if [[ "$skip_stats" != true ]]; then
    stats_args=(python "$openpi_root/scripts/compute_norm_stats.py" --config-name "$config")
    if [[ -n "$max_stat_frames" ]]; then
        stats_args+=(--max-frames "$max_stat_frames")
    fi
    echo "Computing normalization statistics from ${dataset_home}/${dataset_repo_id}"
    (
        cd "$state_root"
        uv run --project "$openpi_root" "${stats_args[@]}"
    )
else
    echo "Skipping normalization-stat computation; the selected config must resolve valid stats."
fi

train_args=(
    python "$openpi_root/scripts/train.py" "$config"
    --exp-name "$experiment"
    --assets-base-dir "$state_root/assets"
    --checkpoint-base-dir "$checkpoint_root"
    --fsdp-devices "$num_gpus"
)
[[ "$wandb_enabled" == false ]] && train_args+=(--no-wandb-enabled)
case "$run_mode" in
    resume) train_args+=(--resume) ;;
    overwrite) train_args+=(--overwrite) ;;
esac
[[ -n "$num_train_steps" ]] && train_args+=(--num-train-steps "$num_train_steps")
[[ -n "$save_interval" ]] && train_args+=(--save-interval "$save_interval")
[[ -n "$batch_size" ]] && train_args+=(--batch-size "$batch_size")
[[ -n "$num_workers" ]] && train_args+=(--num-workers "$num_workers")

echo "Starting ${config}/${experiment} on CUDA devices ${CUDA_VISIBLE_DEVICES}"
(
    cd "$state_root"
    uv run --project "$openpi_root" "${train_args[@]}"
)
