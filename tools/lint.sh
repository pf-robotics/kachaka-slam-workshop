#!/bin/bash

set -eu

EXEC_COMMAND="pysen run lint"
while getopts i OPT; do
  case $OPT in
    i)
      EXEC_COMMAND="pysen run format && pysen run lint" ;;
    *)
      echo "[usage] lint.sh [-i]"
      exit 1
  esac
done

IMAGE_NAME=slam-workshop-lint
SCRIPT_DIR=$(dirname $(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd))

echo $SCRIPT_DIR

echo "build $IMAGE_NAME"
docker build --target slam-workshop-lint -t $IMAGE_NAME $SCRIPT_DIR

USER_ID="$(id -u)"
GROUP_ID="$(id -g)"

docker run -it --rm \
  --user "${USER_ID}:${GROUP_ID}" \
  --volume "${SCRIPT_DIR}:/ws" \
  $IMAGE_NAME \
  bash -c "(cd /ws; git config --local --add safe.directory /ws; ${EXEC_COMMAND})"
