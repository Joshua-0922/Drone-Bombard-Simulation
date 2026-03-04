# Running the Drone Bombard Simulation (Gazebo Harmonic)

This runbook covers the `feature/migration-harmonic` branch. For the stable Gazebo Classic
version see `main` and the instructions in `CLAUDE.md`.

---

## Prerequisites

### 1. Pull the Docker image

```bash
docker pull us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest
```

### 2. Start (or reconnect to) the container

```bash
xhost +local:docker

# First time
docker run -itd --gpus all --net=host --privileged --ipc=host \
  --name drone-bombard-harmonic \
  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws:/workspace/ros2_ws \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models:/workspace/gazebo_models \
  -v ~/.cache:/root/.cache \
  us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest /bin/bash

# Reconnect to existing container
xhost +local:docker && docker start -ai drone-bombard-harmonic
```

### 3. One-time PX4 build (inside container)

Run once after the container is created; the build artifacts persist in `/opt/PX4-Autopilot`.

```bash
cd /opt/PX4-Autopilot && DONT_RUN=1 make px4_sitl gz_x500
```

### 4. Build the ROS2 workspace (inside container)

```bash
cd /workspace/ros2_ws
colcon build
source install/setup.bash
```

---

## Running the Simulation

Open **two terminals** inside the container (`docker exec -it drone-bombard-harmonic bash`
in each new tab after the container is running).

### Terminal 1 — Gazebo + PX4 SITL + DDS bridge + Vision

```bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch path_generation gz_harmonic_sitl.launch.py
```

This single launch file starts everything in the correct order:
- Gazebo Harmonic with the `x_marker_world` world
- PX4 SITL (x500_bombard model with `DetachableJoint` payload — auto-attached at startup)
- uXRCE-DDS bridge (PX4 ↔ ROS2)
- `xmarker_detector` vision node at t=12 s (after Gazebo + PX4 + bridge are fully initialised)

**Optional flags:**

| Flag | Default | Description |
|------|---------|-------------|
| `headless:=true` | `false` | Run Gazebo without GUI (faster, for CI) |
| `enable_vision:=false` | `true` | Skip launching the YOLO vision node |

```bash
# Example: headless run without vision
ros2 launch path_generation gz_harmonic_sitl.launch.py headless:=true enable_vision:=false
```

### Terminal 2 — Mission nodes

```bash
source /workspace/ros2_ws/install/setup.bash
ros2 launch mission_manager drone_mission.launch.py
```

This launches the four mission nodes:
- `mission_manager_node` — FSM (TAKEOFF → CRUISE → TRACKING → DROP)
- `drone_controller` — ENU→NED PX4 bridge
- `rl_navigation_node` — velocity tracking controller; triggers payload detach
- `drop_calculator` — post-drop referee; publishes miss distance to `/rl/drop_error`

---

## Functional Test Checks

Run these in a third terminal (`docker exec -it drone-bombard-harmonic bash`) while the
simulation is running.

```bash
# 1. Confirm payload is attached (DetachableJoint topic should exist)
gz topic -l | grep drop

# 2. Confirm vision node is publishing detections
ros2 topic echo /target/pixel_coords --once

# 3. Confirm drop_calculator is listening for drop events
ros2 topic echo /rl/drop_error

# 4. Check mission state machine state
ros2 topic echo /mission/state --once

# 5. Simulate a vision detection (for testing without a real target in view)
cd /workspace/ros2_ws
python3 system_tester.py
```

---

## World and Model Defaults

| Parameter | Value |
|-----------|-------|
| Gazebo world | `x_marker_world` (spawns red X-marker at (11, 10, 0)) |
| Drone model | `x500_bombard` (with `DetachableJoint` payload at (0, 0, 0.14)) |
| X-marker colour | Solid red (1.0 0.0 0.0) — no PBR |
| Drop target | (11, 10) in world frame |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Gazebo opens but drone doesn't arm | DDS bridge not ready | Wait 15 s after Terminal 1 starts before launching Terminal 2 |
| `xmarker_detector` crashes immediately | YOLO model missing | Ensure `best.pt` is at `/workspace/ros2_ws/yolo_workspace/runs/train/drone_bombard_train2/weights/best.pt` |
| PX4 SITL exits with build error | PX4 not pre-built | Run the one-time `DONT_RUN=1 make px4_sitl gz_x500` step |
| `/rl/drop_error` never publishes | Payload didn't hit ground | Check `/drone/payload/drop_cmd_raw` — should see `False` after detach |
| Two `xmarker_detector` processes running | Old `drone_mission.launch.py` | Ensure you are on the latest `feature/migration-harmonic` and have rebuilt the workspace |
