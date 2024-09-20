#!/bin/bash

set -eu

export USER_ID="$(id -u)"
export GROUP_ID="$(id -g)"

xhost +local:docker

if [[ $# -lt 1 ]]; then
    echo "please specify mode (odometry|mapping|localization)"
    exit
else
    export MODE=$1
fi

if [[ $# -lt 2 ]]; then
    docker compose up
else
    docker compose up $2
fi
