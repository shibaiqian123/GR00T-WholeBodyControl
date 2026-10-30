#!/usr/bin/env bash
# Psi0-style robot launcher: reuse the existing SONIC C++ deployment path.
# Reference: physical-superintelligence-lab/Psi0 real/scripts/deploy_psi0-sonic-rtc-robot.sh
set -euo pipefail

usage() {
    cat <<'HELP'
Usage: deploy_dit4dit_sonic_robot.sh [--interface NAME] [--dry-run]
  --interface NAME  Robot interface (default: enx6c1ff7b8d821).
  --dry-run         Check paths/interface and print command; do not start control.
  -h, --help        Show this help.

Uses SONIC V1.1 encoder/decoder and matching observation config.
Input: existing ZMQ pose protocol, 64D token, tcp://127.0.0.1:5556.
Port 5556 and topic pose are the existing C++ defaults.
Body state/history -> decoder -> existing 29-joint DDS control path.
No camera, VLA server/client or gripper service is started here.
The existing deployment confirmation and manual control startup remain in place.
HELP
}
fail() { printf 'Error: %s\n' "$*" >&2; exit 2; }

interface=enx6c1ff7b8d821
dry_run=0
while (( $# )); do
    case "$1" in
        --interface)
            (( $# >= 2 )) && [[ -n "$2" && "$2" != -* ]] ||
                fail '--interface requires a network interface name'
            interface="$2"
            shift 2
            ;;
        --dry-run) dry_run=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "Unknown argument: $1" ;;
    esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../.." && pwd)"
deploy_dir="$repo_root/gear_sonic_deploy"
checkpoint=policy/sonic_v1_1/model
obs_config=policy/sonic_v1_1/observation_config.yaml
for file in deploy.sh "${checkpoint}_encoder.onnx" "${checkpoint}_decoder.onnx" "$obs_config"; do
    [[ -s "$deploy_dir/$file" && -r "$deploy_dir/$file" ]] ||
        fail "Required file missing, empty or unreadable: $deploy_dir/$file"
done
command -v ip >/dev/null 2>&1 || fail 'ip command is required to verify the network interface'
ip link show dev "$interface" >/dev/null 2>&1 ||
    fail "Network interface does not exist: $interface"

cd -- "$deploy_dir"
command=(bash ./deploy.sh
    --cp "$checkpoint"
    --obs-config "$obs_config"
    --input-type zmq
    --zmq-host 127.0.0.1
    "$interface")
printf 'SONIC V1.1 | interface=%s | input=127.0.0.1:5556 topic=pose\n' "$interface"
printf 'Working directory: %s\n' "$deploy_dir"
printf 'Command:'
printf ' %q' "${command[@]}"
printf '\n'
if (( dry_run )); then
    printf 'DRY RUN: no controller started; no motor commands sent.\n'
    exit 0
fi
exec "${command[@]}"
