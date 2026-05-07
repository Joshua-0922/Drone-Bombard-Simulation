#!/usr/bin/env bash

set -eo pipefail

source /workspace/ros2_ws/source_container_env.sh
exec ros2 launch mission_manager drone_mission.launch.py "$@"
