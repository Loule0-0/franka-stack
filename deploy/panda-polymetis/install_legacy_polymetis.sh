#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=versions.env
source "$SCRIPT_DIR/versions.env"
PROGRAM_NAME="${0##*/}"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --source-root PATH --env-prefix PATH [options]

Install the pinned, historical Panda/Polymetis source stack. This helper is
deliberately limited to Ubuntu 20.04 x86_64 and Python 3.8. It does not claim
hardware validation and does not start a robot server. An existing environment
prefix is synchronized from the pinned environment.yml with --prune.

Options:
  --source-root PATH       Persistent fairo checkout destination.
  --env-prefix PATH        Persistent conda/mamba environment prefix.
  --solver COMMAND         mamba or conda (auto-detected by default).
  --clone-only             Only clone, pin, and initialize libfranka.
  -h, --help               Show this help.
EOF
}

source_root="${PANDA_POLYMETIS_SOURCE_ROOT:-}"
env_prefix="${PANDA_POLYMETIS_ENV_PREFIX:-}"
solver=""
clone_only=false

while (($#)); do
    case "$1" in
        --source-root) source_root="${2:?missing value for --source-root}"; shift 2 ;;
        --env-prefix) env_prefix="${2:?missing value for --env-prefix}"; shift 2 ;;
        --solver) solver="${2:?missing value for --solver}"; shift 2 ;;
        --clone-only) clone_only=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

require_persistent_absolute_path() {
    local label="$1"
    local path="$2"
    [[ "$path" == /* ]] || { echo "ERROR: ${label} must be an absolute path" >&2; exit 2; }
    case "$path" in
        /tmp|/tmp/*|/var/tmp|/var/tmp/*) echo "ERROR: ${label} must be persistent" >&2; exit 2 ;;
    esac
}
require_persistent_absolute_path "source root" "$source_root"
require_persistent_absolute_path "environment prefix" "$env_prefix"

for command_name in git realpath; do
    command -v "$command_name" >/dev/null 2>&1 || { echo "ERROR: required command not found: $command_name" >&2; exit 3; }
done
source_root="$(realpath -m -- "$source_root")"
env_prefix="$(realpath -m -- "$env_prefix")"
case "$env_prefix" in
    /|/bin|/bin/*|/boot|/boot/*|/dev|/dev/*|/etc|/etc/*|/home|/home/data|/lib|/lib/*|/lib64|/lib64/*|/opt|/proc|/proc/*|/root|/root/*|/run|/run/*|/sbin|/sbin/*|/sys|/sys/*|/usr|/usr/*|/var|/var/*)
        echo "ERROR: environment prefix is a system or shared root: $env_prefix" >&2
        exit 2
        ;;
esac
case "$env_prefix/" in
    "$source_root/"*) echo "ERROR: environment prefix must not be inside the source checkout" >&2; exit 2 ;;
esac
case "$source_root/" in
    "$env_prefix/"*) echo "ERROR: source checkout must not be inside the environment prefix" >&2; exit 2 ;;
esac
if [[ "$clone_only" != true ]]; then
    [[ "$(uname -m)" == "x86_64" ]] || { echo "ERROR: this historical stack is only gated for x86_64" >&2; exit 1; }
    [[ -r /etc/os-release ]] || { echo "ERROR: cannot identify the operating system" >&2; exit 1; }
    # shellcheck source=/dev/null
    source /etc/os-release
    [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "20.04" ]] || {
        echo "ERROR: pinned Polymetis baseline requires Ubuntu 20.04; found ${ID:-unknown} ${VERSION_ID:-unknown}" >&2
        exit 1
    }
fi

if [[ -e "$source_root" && ! -d "$source_root/.git" ]]; then
    echo "ERROR: source root exists but is not a Git checkout: $source_root" >&2
    exit 1
fi
created_checkout=false
if [[ ! -d "$source_root/.git" ]]; then
    mkdir -p "$(dirname "$source_root")"
    git clone --filter=blob:none --sparse "$FAIRO_REPOSITORY" "$source_root"
    git -C "$source_root" sparse-checkout set polymetis
    created_checkout=true
fi

origin_url="$(git -C "$source_root" remote get-url origin)"
[[ "$origin_url" == "$FAIRO_REPOSITORY" || "$origin_url" == "git@github.com:facebookresearch/fairo.git" ]] || {
    echo "ERROR: unexpected fairo origin: $origin_url" >&2
    exit 1
}
current_commit="$(git -C "$source_root" rev-parse HEAD 2>/dev/null || true)"
if [[ "$current_commit" != "$FAIRO_COMMIT" ]]; then
    if [[ "$created_checkout" != true ]]; then
        [[ -z "$(git -C "$source_root" status --porcelain)" ]] || {
            echo "ERROR: fairo checkout is dirty; refusing to switch commits" >&2
            exit 1
        }
    fi
    git -C "$source_root" fetch --depth 1 origin "$FAIRO_COMMIT"
    git -C "$source_root" switch --detach "$FAIRO_COMMIT"
fi

libfranka_path="polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka"
git -C "$source_root" submodule update --init --recursive "$libfranka_path"
actual_libfranka="$(git -C "$source_root/$libfranka_path" rev-parse HEAD)"
[[ "$actual_libfranka" == "$LIBFRANKA_COMMIT" ]] || {
    echo "ERROR: bundled libfranka is ${actual_libfranka}, expected ${LIBFRANKA_COMMIT}" >&2
    exit 1
}
bash "$SCRIPT_DIR/apply_fail_closed_patch.sh" --source-root "$source_root"

echo "Pinned legacy source is ready:"
echo "  fairo: $(git -C "$source_root" rev-parse HEAD)"
echo "  libfranka: ${actual_libfranka} (${LIBFRANKA_VERSION})"
if [[ "$clone_only" == true ]]; then
    echo "Source-only preparation does not validate the build host or physical robot."
    exit 0
fi

if [[ -z "$solver" ]]; then
    if command -v mamba >/dev/null 2>&1; then
        solver="mamba"
    elif command -v conda >/dev/null 2>&1; then
        solver="conda"
    else
        echo "ERROR: mamba or conda is required for the historical dependency set" >&2
        exit 3
    fi
fi
command -v "$solver" >/dev/null 2>&1 || { echo "ERROR: solver not found: $solver" >&2; exit 3; }
command -v sha256sum >/dev/null 2>&1 || { echo "ERROR: sha256sum is required for build provenance" >&2; exit 3; }

environment_file="$source_root/polymetis/polymetis/environment.yml"
[[ -f "$environment_file" ]] || { echo "ERROR: pinned environment file not found: $environment_file" >&2; exit 1; }
environment_sha256="$(sha256sum "$environment_file" | awk '{print $1}')"
if [[ ! -x "$env_prefix/bin/python" ]]; then
    mkdir -p "$(dirname "$env_prefix")"
    "$solver" env create --prefix "$env_prefix" --file "$environment_file"
else
    conda_history="$env_prefix/conda-meta/history"
    [[ -f "$conda_history" && ! -L "$conda_history" ]] || {
        echo "ERROR: existing prefix is not an identifiable conda environment: $env_prefix" >&2
        exit 1
    }
    "$solver" env update --prefix "$env_prefix" --file "$environment_file" --prune
fi

verify_legacy_python_versions() {
    python_version="$("$env_prefix/bin/python" -c 'import platform; print(platform.python_version())')"
    [[ "$python_version" == "${POLYMETIS_PYTHON_VERSION}."* ]] || {
        echo "ERROR: legacy environment must use Python ${POLYMETIS_PYTHON_VERSION}, found ${python_version}" >&2
        exit 1
    }
    torch_version="$("$env_prefix/bin/python" -c 'import torch; print(torch.__version__.split("+", 1)[0])')"
    [[ "$torch_version" == "1.13.1" ]] || {
        echo "ERROR: legacy environment must use PyTorch 1.13.1, found ${torch_version}" >&2
        exit 1
    }
    numpy_version="$("$env_prefix/bin/python" -c 'import numpy; print(numpy.__version__)')"
    [[ "$numpy_version" == "1.23.5" ]] || {
        echo "ERROR: legacy environment must use NumPy 1.23.5, found ${numpy_version}" >&2
        exit 1
    }
}
verify_legacy_python_versions

project_dir="$source_root/polymetis/polymetis"
"$solver" run --prefix "$env_prefix" env \
    PREFIX="$env_prefix" \
    PYTHON="$env_prefix/bin/python" \
    BUILD_FRANKA=ON \
    BUILD_TESTS=OFF \
    BUILD_DOCS=OFF \
    BUILD_ALLEGRO=OFF \
    DEV_PYTHON=ON \
    bash -c 'cd "$1" && ./install.sh' _ "$project_dir"

arm_binary="$env_prefix/bin/franka_panda_client"
hand_binary="$env_prefix/bin/franka_hand_client"
for installed_binary in "$arm_binary" "$hand_binary"; do
    [[ -x "$installed_binary" ]] || {
        echo "ERROR: build completed without installed client: $installed_binary" >&2
        exit 1
    }
done
verify_legacy_python_versions
"$env_prefix/bin/python" -m pip check
provenance_file="$source_root/.franka_real_pi05_build_provenance"
[[ ! -L "$provenance_file" && ( ! -e "$provenance_file" || -f "$provenance_file" ) ]] || {
    echo "ERROR: build provenance destination must be absent or a regular file: $provenance_file" >&2
    exit 1
}
provenance_tmp="$(mktemp --tmpdir="$source_root" .franka_real_pi05_build_provenance.tmp.XXXXXX)"
cleanup_provenance_tmp() {
    [[ -z "${provenance_tmp:-}" || ! -e "$provenance_tmp" ]] || rm -f -- "$provenance_tmp"
}
trap cleanup_provenance_tmp EXIT
{
    printf 'FAIRO_COMMIT=%s\n' "$FAIRO_COMMIT"
    printf 'LIBFRANKA_COMMIT=%s\n' "$actual_libfranka"
    printf 'PATCH_SHA256=%s\n' "$(sha256sum "$SCRIPT_DIR/fail_closed.patch" | awk '{print $1}')"
    printf 'ENVIRONMENT_SHA256=%s\n' "$environment_sha256"
    printf 'PYTHON_VERSION=%s\n' "$python_version"
    printf 'PYTORCH_VERSION=%s\n' "$torch_version"
    printf 'NUMPY_VERSION=%s\n' "$numpy_version"
    printf 'ARM_BINARY_PATH=%s\n' "$arm_binary"
    printf 'ARM_BINARY_SHA256=%s\n' "$(sha256sum "$arm_binary" | awk '{print $1}')"
    printf 'HAND_BINARY_PATH=%s\n' "$hand_binary"
    printf 'HAND_BINARY_SHA256=%s\n' "$(sha256sum "$hand_binary" | awk '{print $1}')"
    printf 'BUILT_AT_UTC=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} >"$provenance_tmp"
chmod 0444 "$provenance_tmp"
mv -T -- "$provenance_tmp" "$provenance_file"
provenance_tmp=""
trap - EXIT

echo "Legacy Polymetis build completed with the fail-closed patch, but physical-robot compatibility is NOT certified."
echo "Build provenance: $provenance_file"
echo "Run check_legacy_stack.sh, then perform read-only and low-speed validation before motion."
