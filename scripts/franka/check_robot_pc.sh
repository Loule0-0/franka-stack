#!/usr/bin/env bash
set -Eeuo pipefail

PROGRAM_NAME="${0##*/}"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --interface IFACE [options]

Read-only, strict audit for the Franka control PC. Every required check is run;
the script exits non-zero if any check fails.

Options:
  --interface IFACE       Dedicated FCI interface (or FRANKA_ROBOT_NIC).
  --host-cidr CIDR        Expected host address (default: 172.16.0.1/24).
  --robot-ip IP           Robot address (default: 172.16.0.2).
  --min-rtprio N          Minimum realtime priority (default: 99).
  --min-memlock-kib N     Minimum locked memory in KiB (default: 102400).
  --ping-count N          ICMP probes (default: 3).
  --ping-timeout N        Timeout per probe in seconds (default: 1).
  -h, --help              Show this help.

Environment equivalents:
  FRANKA_ROBOT_NIC, ROBOT_PC_FCI_CIDR, ROBOT_FCI_IP,
  FRANKA_MIN_RTPRIO, FRANKA_MIN_MEMLOCK_KIB, FRANKA_PING_COUNT,
  FRANKA_PING_TIMEOUT.
EOF
}

interface="${FRANKA_ROBOT_NIC:-}"
host_cidr="${ROBOT_PC_FCI_CIDR:-172.16.0.1/24}"
robot_ip="${ROBOT_FCI_IP:-172.16.0.2}"
min_rtprio="${FRANKA_MIN_RTPRIO:-99}"
min_memlock_kib="${FRANKA_MIN_MEMLOCK_KIB:-102400}"
ping_count="${FRANKA_PING_COUNT:-3}"
ping_timeout="${FRANKA_PING_TIMEOUT:-1}"

while (($#)); do
    case "$1" in
        --interface)
            [[ $# -ge 2 ]] || { echo "ERROR: --interface needs a value" >&2; exit 2; }
            interface="$2"
            shift 2
            ;;
        --host-cidr)
            [[ $# -ge 2 ]] || { echo "ERROR: --host-cidr needs a value" >&2; exit 2; }
            host_cidr="$2"
            shift 2
            ;;
        --robot-ip)
            [[ $# -ge 2 ]] || { echo "ERROR: --robot-ip needs a value" >&2; exit 2; }
            robot_ip="$2"
            shift 2
            ;;
        --min-rtprio)
            [[ $# -ge 2 ]] || { echo "ERROR: --min-rtprio needs a value" >&2; exit 2; }
            min_rtprio="$2"
            shift 2
            ;;
        --min-memlock-kib)
            [[ $# -ge 2 ]] || { echo "ERROR: --min-memlock-kib needs a value" >&2; exit 2; }
            min_memlock_kib="$2"
            shift 2
            ;;
        --ping-count)
            [[ $# -ge 2 ]] || { echo "ERROR: --ping-count needs a value" >&2; exit 2; }
            ping_count="$2"
            shift 2
            ;;
        --ping-timeout)
            [[ $# -ge 2 ]] || { echo "ERROR: --ping-timeout needs a value" >&2; exit 2; }
            ping_timeout="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ -n "$interface" ]] || { echo "ERROR: --interface (or FRANKA_ROBOT_NIC) is required" >&2; exit 2; }
for value in "$min_rtprio" "$min_memlock_kib" "$ping_count" "$ping_timeout"; do
    [[ "$value" =~ ^[0-9]+$ ]] || { echo "ERROR: numeric option has invalid value: $value" >&2; exit 2; }
done

for command_name in ip ping python3 uname grep awk; do
    command -v "$command_name" >/dev/null 2>&1 || {
        echo "ERROR: required command not found: $command_name" >&2
        exit 3
    }
done

host_ip="$({ HOST_CIDR="$host_cidr" ROBOT_IP="$robot_ip" python3 - <<'PY'
import ipaddress
import os

host = ipaddress.ip_interface(os.environ["HOST_CIDR"])
robot = ipaddress.ip_address(os.environ["ROBOT_IP"])
if host.version != 4 or robot.version != 4:
    raise SystemExit("FCI addresses must be IPv4")
if host.ip == robot:
    raise SystemExit("host and robot addresses must differ")
if robot not in host.network:
    raise SystemExit(f"robot {robot} is outside host network {host.network}")
print(host.ip)
PY
} 2>&1)" || { echo "ERROR: invalid FCI addressing: $host_ip" >&2; exit 2; }

failures=0
pass() { printf '[PASS] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*" >&2; failures=$((failures + 1)); }

kernel_release="$(uname -r)"
printf '[INFO] kernel: %s\n' "$kernel_release"

if [[ -r /sys/kernel/realtime ]] && [[ "$(< /sys/kernel/realtime)" == "1" ]]; then
    pass "/sys/kernel/realtime reports 1"
else
    fail "/sys/kernel/realtime is absent or does not report 1"
fi

kernel_config=""
if [[ -r "/boot/config-${kernel_release}" ]]; then
    kernel_config="/boot/config-${kernel_release}"
elif [[ -r /proc/config.gz ]] && command -v zgrep >/dev/null 2>&1; then
    kernel_config="/proc/config.gz"
fi

if [[ -z "$kernel_config" ]]; then
    fail "kernel config is not readable (/boot/config-${kernel_release} or /proc/config.gz)"
elif [[ "$kernel_config" == *.gz ]]; then
    if zgrep -Eq '^CONFIG_PREEMPT_RT(_FULL)?=y$' "$kernel_config"; then
        pass "kernel config enables PREEMPT_RT"
    else
        fail "kernel config does not enable PREEMPT_RT"
    fi
elif grep -Eq '^CONFIG_PREEMPT_RT(_FULL)?=y$' "$kernel_config"; then
    pass "kernel config enables PREEMPT_RT"
else
    fail "kernel config does not enable PREEMPT_RT"
fi

rtprio="$(ulimit -r)"
if [[ "$rtprio" == "unlimited" ]] || { [[ "$rtprio" =~ ^[0-9]+$ ]] && ((rtprio >= min_rtprio)); }; then
    pass "realtime priority limit is ${rtprio} (required: ${min_rtprio})"
else
    fail "realtime priority limit is ${rtprio} (required: ${min_rtprio})"
fi

memlock="$(ulimit -l)"
if [[ "$memlock" == "unlimited" ]] || { [[ "$memlock" =~ ^[0-9]+$ ]] && ((memlock >= min_memlock_kib)); }; then
    pass "locked-memory limit is ${memlock} KiB (required: ${min_memlock_kib})"
else
    fail "locked-memory limit is ${memlock} KiB (required: ${min_memlock_kib})"
fi

governor_files=(/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor)
if ((${#governor_files[@]} == 0)) || [[ ! -e "${governor_files[0]}" ]]; then
    fail "CPU frequency governor files are unavailable"
else
    governor_failures=0
    for governor_file in "${governor_files[@]}"; do
        governor="$(< "$governor_file")"
        if [[ "$governor" != "performance" ]]; then
            printf '[FAIL] %s is %s, expected performance\n' "$governor_file" "$governor" >&2
            governor_failures=$((governor_failures + 1))
        fi
    done
    if ((governor_failures == 0)); then
        pass "all CPU frequency governors are performance"
    else
        failures=$((failures + governor_failures))
    fi
fi

if ! ip link show dev "$interface" >/dev/null 2>&1; then
    fail "network interface does not exist: $interface"
else
    pass "network interface exists: $interface"
    link_state="$(< "/sys/class/net/${interface}/operstate")"
    if [[ "$link_state" == "up" ]]; then
        pass "interface ${interface} link state is up"
    else
        fail "interface ${interface} link state is ${link_state}"
    fi

    if ip -4 -o address show dev "$interface" | awk '{print $4}' | grep -Fxq "$host_cidr"; then
        pass "interface ${interface} owns ${host_cidr}"
    else
        fail "interface ${interface} does not own ${host_cidr}"
    fi

    route_line="$(ip -4 route get "$robot_ip" 2>&1 | head -n 1 || true)"
    read -r -a route_fields <<<"$route_line"
    route_device=""
    route_source=""
    for ((index = 0; index < ${#route_fields[@]}; index++)); do
        if [[ "${route_fields[$index]}" == "dev" ]] && ((index + 1 < ${#route_fields[@]})); then
            route_device="${route_fields[$((index + 1))]}"
        elif [[ "${route_fields[$index]}" == "src" ]] && ((index + 1 < ${#route_fields[@]})); then
            route_source="${route_fields[$((index + 1))]}"
        fi
    done
    if [[ "$route_device" == "$interface" && "$route_source" == "$host_ip" ]]; then
        pass "route to ${robot_ip} uses ${interface} with source ${host_ip}"
    else
        fail "route to ${robot_ip} is wrong: ${route_line:-no route}"
    fi

    if ping -I "$interface" -c "$ping_count" -W "$ping_timeout" "$robot_ip" >/dev/null 2>&1; then
        pass "robot responds to ICMP on ${interface}: ${robot_ip}"
    else
        fail "robot did not respond to ICMP on ${interface}: ${robot_ip}"
    fi
fi

if ((failures > 0)); then
    printf '[SUMMARY] audit failed: %d check(s) failed\n' "$failures" >&2
    exit 1
fi

echo "[SUMMARY] robot PC audit passed"
