# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Environment

All development happens **inside a Docker container**. The host VM at `/opt/drone-bombard` is for git operations only. The container mounts:
- `ros2_ws/` → `/workspace/ros2_ws` (ROS2 packages)
- `gazebo_models/` → `/workspace/gazebo_models` (Gazebo models/worlds)

Docker image is built automatically via GitHub Actions on push to `main`. Pull the image; never build locally.

```bash
# Pull latest image
docker pull us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest

# Start container (first time)
xhost +local:docker
docker run -itd --gpus all --net=host --privileged --ipc=host \
  --name drone-bombard-dev \
  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws:/workspace/ros2_ws \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models:/workspace/gazebo_models \
  -v ~/.cache:/root/.cache \
  us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest /bin/bash

# Reconnect to existing container
xhost +local:docker && docker start -ai drone-bombard-dev
```

## Build Commands (inside container)

```bash
# Build all ROS2 packages
cd /workspace/ros2_ws
colcon build
source install/setup.bash

# Build a single package
colcon build --packages-select <package_name>
source install/setup.bash
```

## Running the Full Simulation

Three terminals are needed, all inside the container (`docker exec -it drone-bombard-dev bash`):

**Terminal 1** – uXRCE-DDS bridge (PX4 ↔ ROS2):
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2** – PX4 SITL + Gazebo (minimize GUI immediately, it's slow):
```bash
cd /opt/PX4-Autopilot
make px4_sitl gazebo-classic
```

**Terminal 3** – All ROS2 nodes:
```bash
cd /workspace/ros2_ws
source install/setup.bash
ros2 launch mission_manager drone_mission.launch.py
```

## System Architecture

The system is an autonomous drone that **cruises**, **detects** an X-marker target via downward camera, **tracks** it, then **drops** a payload using ballistics timing.

### Mission State Machine (`mission_manager`)

FSM states managed by `mission_manager_node`:
- `TAKEOFF` → climbs to 10m altitude
- `CRUISE` → flies north-east at 1 m/s until target detected
- `TRACKING` → delegates velocity control to `rl_navigation`
- `DROP` → hovers after drop confirmed

State transitions are driven by `/target/pixel_coords` (vision detection) and `/payload/drop_cmd` (ballistics).

### ROS2 Package Overview

| Package | Type | Role |
|---------|------|------|
| `mission_manager` | Python | FSM commander; publishes `/drone/cmd/position` or `/drone/cmd/velocity` |
| `drone_controller` | Python | PX4 bridge; converts ENU→NED commands to `/fmu/in/trajectory_setpoint` |
| `vision_detection` | C++ (CMake) | YOLOv8 inference; publishes `/target/pixel_coords` |
| `rl_navigation` | Python | Tracking controller; sends velocity commands and triggers payload detach |
| `drop_calculator` | Python | Ballistics engine; publishes `/payload/drop_cmd` |
| `gazebo_ros_link_attacher` | C++ (CMake) | Gazebo plugin; exposes `/attach` and `/detach` services for payload joint |
| `px4_msgs` | CMake | PX4 message type definitions (synced with PX4 v1.15.4) |

### Key Topics

| Topic | Type | Flow |
|-------|------|------|
| `/target/pixel_coords` | `geometry_msgs/Point` | vision_detection → mission_manager, rl_navigation, drop_calculator (x=u, y=v, z=confidence) |
| `/mission/state` | `std_msgs/String` | mission_manager → rl_navigation |
| `/drone/cmd/position` | `geometry_msgs/Vector3` | mission_manager → drone_controller |
| `/drone/cmd/velocity` | `geometry_msgs/Twist` | rl_navigation → drone_controller |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | drone_controller → PX4 |
| `/payload/drop_cmd` | `std_msgs/Bool` | drop_calculator → mission_manager |
| `/drone/payload/drop_cmd_raw` | `std_msgs/Bool` | rl_navigation → drop_calculator (False = drop event) |
| `/vision/detections` | `vision_detection/DetectionResult` | vision_detection → monitoring |

### Coordinate Systems

- **PX4 uses NED** (North-East-Down): altitude is negative-z, east is +y
- **ROS2 uses ENU**: altitude is +z
- `drone_controller` translates: ENU `(x, y, z)` → NED `(x, -y, -z)` for position; similar for velocity

### Payload Drop Mechanism

`rl_navigation` calls `ros2 service call /attach` on startup to weld the payload, then calls `/detach` (via `gazebo_ros_link_attacher`) when the target is detected in TRACKING mode. It signals the drop event by publishing `False` on `/drone/payload/drop_cmd_raw`.

### YOLO Model

- Model file: `/workspace/ros2_ws/yolo_workspace/runs/train/drone_bombard_train2/weights/best.pt` (inside container)
- Root copy: `drone_bombard_best.pt` in repo root
- YOLOv8n trained on X-marker images; mAP@0.5 = 99.5%
- Inference rate: 10 Hz

### Docker Image Contents

Built from `drone_drop_system/docker/Dockerfile`:
- Ubuntu 22.04 + CUDA 12.6.2
- ROS2 Humble (desktop)
- Gazebo Classic + `ros-humble-gazebo-ros-pkgs`
- PX4 Autopilot v1.15.4 at `/opt/PX4-Autopilot`
- uXRCE-DDS-Agent at `/opt/Micro-XRCE-DDS-Agent`
- px4_msgs pre-built at `/root/ros2_ws` (separate from workspace volume)
- Python deps from `drone_drop_system/docker/requirements.txt`
