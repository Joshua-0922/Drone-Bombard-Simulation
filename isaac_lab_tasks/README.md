# Isaac Lab Tasks — Drone Bombard

GPU-native RL training (no ROS2). Gazebo stack remains in `ros2_ws/` for deployment fallback.

## Quick start (Docker + GPU)

```bash
cd Drone-Bombard-Simulation
docker build -f docker/Dockerfile.isaac -t drone-bombard-isaac .

# Smoke (num_envs=1)
docker run --gpus all --rm \
  -v $(pwd)/isaac_lab_tasks:/workspace/isaac_lab_tasks \
  drone-bombard-isaac \
  /isaac-sim/python.sh /workspace/isaac_lab_tasks/drone_bombard/scripts/smoke_test.py --headless

# Train
docker run --gpus all --rm \
  -v $(pwd)/isaac_lab_tasks:/workspace/isaac_lab_tasks \
  -v $(pwd)/logs:/workspace/logs \
  drone-bombard-isaac \
  --task Isaac-DroneDrop-v0 --num_envs 1024 --headless --total_timesteps 5000000

# FPS benchmark
docker run --gpus all --rm \
  -v $(pwd)/isaac_lab_tasks:/workspace/isaac_lab_tasks \
  drone-bombard-isaac \
  /isaac-sim/python.sh /workspace/isaac_lab_tasks/drone_bombard/scripts/benchmark_envs.py \
  --env_counts 1,64,256,1024 --headless
```

## Task IDs

| ID | Description |
|----|-------------|
| `Isaac-DroneDrop-v0` | Training (default 4096 envs in cfg; override via `--num_envs`) |
| `Isaac-DroneDrop-Play-v0` | Play / eval (4 envs) |

## Docs

- [docs/ASSETS.md](docs/ASSETS.md) — USD Path A/B
- [docs/PAYLOAD_ATTACHMENT.md](docs/PAYLOAD_ATTACHMENT.md) — drop mechanism
