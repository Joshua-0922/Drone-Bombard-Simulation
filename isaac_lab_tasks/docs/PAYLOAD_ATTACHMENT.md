# Payload Attachment (Gazebo DetachableJoint → Isaac Lab)

## Modes

| Mode | Description | When to use |
|------|-------------|-------------|
| `kinematic_sync` (default) | Each physics step: copy drone pose+velocity to payload; disable gravity while attached | `num_envs >= 64`, production RL |
| `fixed_joint` | Create USD `UsdPhysics.FixedJoint` between Drone and Payload; break on drop | Smoke tests, `num_envs <= 16` |

## Drop sequence

1. CCIP `d_impact <= auto_drop_threshold` → `detach_payload()`
2. Payload falls under PhysX gravity
3. `payload_landed` when `z <= 0.04` and `dropped=True`
4. Reset → `reattach_payload_on_reset()` + pose teleport

## Fallback

If `fixed_joint` prim creation fails at init, env falls back to `kinematic_sync` automatically.

## Gazebo parity note

Gazebo uses `gz::sim::systems::DetachableJoint` (topic-triggered joint destroy). Isaac
kinematic_sync is faster for GPU batches but does not model payload inertia while attached.
For ablation, compare free-fall trajectories at drop from 5 m altitude.
