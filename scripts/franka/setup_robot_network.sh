#!/usr/bin/env bash
set -Eeuo pipefail

PROGRAM_NAME="${0##*/}"

usage() {
    cat <<EOF
Usage: ${PROGRAM_NAME} --interface IFACE [options] [--apply]

Configure one dedicated NetworkManager connection for the Franka FCI link.
The default is a dry run. No network state changes unless --apply is present.

Options:
  --interface IFACE       Dedicated physical interface (or FRANKA_ROBOT_NIC).
  --host-cidr CIDR        Host address (default: 172.16.0.1/24).
  --robot-ip IP           Robot address (default: 172.16.0.2).
  --connection-name NAME  NetworkManager profile name (default: franka-fci-IFACE).
  --ping-count N          Post-apply probes (default: 3).
  --ping-timeout N        Timeout per probe in seconds (default: 1).
  --apply                 Apply and verify the configuration.
  -h, --help              Show this help.
EOF
}

interface="${FRANKA_ROBOT_NIC:-}"
host_cidr="${ROBOT_PC_FCI_CIDR:-172.16.0.1/24}"
robot_ip="${ROBOT_FCI_IP:-172.16.0.2}"
connection_name=""
ping_count="${FRANKA_PING_COUNT:-3}"
ping_timeout="${FRANKA_PING_TIMEOUT:-1}"
apply=false

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
        --connection-name)
            [[ $# -ge 2 ]] || { echo "ERROR: --connection-name needs a value" >&2; exit 2; }
            connection_name="$2"
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
        --apply)
            apply=true
            shift
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
[[ "$interface" != "lo" ]] || { echo "ERROR: loopback cannot be the FCI interface" >&2; exit 2; }
[[ "$interface" != */* ]] || { echo "ERROR: invalid interface name: $interface" >&2; exit 2; }
[[ "$ping_count" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: invalid --ping-count" >&2; exit 2; }
[[ "$ping_timeout" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: invalid --ping-timeout" >&2; exit 2; }

connection_name="${connection_name:-franka-fci-${interface}}"
[[ -n "$connection_name" && "$connection_name" != */* ]] || {
    echo "ERROR: invalid NetworkManager connection name" >&2
    exit 2
}

for command_name in ip python3; do
    command -v "$command_name" >/dev/null 2>&1 || { echo "ERROR: required command not found: $command_name" >&2; exit 3; }
done
ip link show dev "$interface" >/dev/null 2>&1 || { echo "ERROR: interface not found: $interface" >&2; exit 1; }

HOST_CIDR="$host_cidr" ROBOT_IP="$robot_ip" python3 - <<'PY'
import ipaddress
import os

host = ipaddress.ip_interface(os.environ["HOST_CIDR"])
robot = ipaddress.ip_address(os.environ["ROBOT_IP"])
if host.version != 4 or robot.version != 4:
    raise SystemExit("FCI addresses must be IPv4")
if host.ip == robot:
    raise SystemExit("host and robot addresses must differ")
if host.network.prefixlen != 24:
    raise SystemExit("this deployment requires a dedicated /24 FCI network")
if robot not in host.network:
    raise SystemExit(f"robot {robot} is outside host network {host.network}")
if host.ip.is_unspecified or host.ip.is_loopback or robot.is_unspecified or robot.is_loopback:
    raise SystemExit("FCI addresses must be unicast, non-loopback addresses")
PY

print_command() {
    printf '  '
    printf '%q ' "$@"
    printf '\n'
}

echo "Mode: $([[ "$apply" == true ]] && echo APPLY || echo DRY-RUN)"
echo "Interface: ${interface}"
echo "Connection: ${connection_name}"
echo "Host address: ${host_cidr}"
echo "Robot address: ${robot_ip}"

if [[ "$apply" != true ]]; then
    echo "Commands that would run:"
    print_command nmcli connection add type ethernet ifname "$interface" con-name "$connection_name"
    print_command nmcli connection modify "$connection_name" connection.interface-name "$interface" ipv4.method manual ipv4.addresses "$host_cidr" ipv4.gateway "" ipv4.dns "" ipv4.never-default yes ipv6.method disabled connection.autoconnect yes
    print_command nmcli connection up "$connection_name"
    echo "Dry run only. Re-run with --apply after checking the interface name."
    exit 0
fi

((EUID == 0)) || { echo "ERROR: --apply must run as root (use sudo)" >&2; exit 1; }
for command_name in nmcli ping; do
    command -v "$command_name" >/dev/null 2>&1 || { echo "ERROR: required command not found: $command_name" >&2; exit 3; }
done

if nmcli -g NAME connection show "$connection_name" >/dev/null 2>&1; then
    existing_interface="$(nmcli -g connection.interface-name connection show "$connection_name")"
    if [[ "$existing_interface" != "$interface" ]]; then
        echo "ERROR: existing profile ${connection_name} belongs to ${existing_interface}; refusing to modify it" >&2
        exit 1
    fi
else
    nmcli connection add type ethernet ifname "$interface" con-name "$connection_name"
fi

nmcli connection modify "$connection_name" \
    connection.interface-name "$interface" \
    ipv4.method manual \
    ipv4.addresses "$host_cidr" \
    ipv4.gateway "" \
    ipv4.dns "" \
    ipv4.never-default yes \
    ipv6.method disabled \
    connection.autoconnect yes
nmcli connection up "$connection_name"

host_ip="${host_cidr%/*}"
ip -4 -o address show dev "$interface" | awk '{print $4}' | grep -Fxq "$host_cidr" || {
    echo "ERROR: ${interface} does not own ${host_cidr} after apply" >&2
    exit 1
}
route_line="$(ip -4 route get "$robot_ip" | head -n 1)"
[[ " $route_line " == *" dev ${interface} "* && " $route_line " == *" src ${host_ip} "* ]] || {
    echo "ERROR: unexpected route after apply: $route_line" >&2
    exit 1
}
ping -I "$interface" -c "$ping_count" -W "$ping_timeout" "$robot_ip" >/dev/null || {
    echo "ERROR: robot does not answer at ${robot_ip}; check FCI enablement, cable, and robot address" >&2
    exit 1
}

echo "FCI network configured and verified. No default route or DNS was assigned."
