#!/bin/bash

set -eu

usage() {
    echo "Usage: $0 KACHAKA_IP_ADRESS [Option]"
    echo "  -d    daemonize"
    exit 1
}

if [[ $# -lt 1 ]]; then
    usage
fi

USER_ID="$(id -u)"
GROUP_ID="$(id -g)"
GRPC_PORT=26400

export USER_ID
export GROUP_ID

KACHAKA_IP=$1

API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}" docker compose -f docker-compose-ros2-bridge.yaml up "${@:2}" ros2_bridge
