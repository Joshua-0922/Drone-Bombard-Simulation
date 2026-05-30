#!/bin/bash
set -e

echo "[ENTRYPOINT] Isaac Lab container"
command -v nvidia-smi &>/dev/null && nvidia-smi -L || true

export ISAACLAB_PATH=${ISAACLAB_PATH:-/workspace/isaaclab}
export ACCEPT_EULA=Y
export PRIVACY_CONSENT=Y

exec "$@"
