# Isaac Lab Assets (Path A — training default)

## Drone (`assets/drone.py`)

- **USD:** `/Isaac/Robots/Crazyflie/cf2x.usd` (placeholder; x500 SDF has no URDF in repo)
- **Control:** No motor joints — `actuators={}`; thrust via `set_external_force_and_torque` in PD controller
- **Mass:** 2.0 kg (x500 nominal)
- **Max total thrust:** ~34 N (4 × 8.5 N) — clipped in `mdp/actions.py`

## Payload (`assets/payload.py`)

- **Spawn:** Procedural `CylinderCfg` r=0.03 m, h=0.005 m, mass=0.1 kg (matches Gazebo `payload_cylinder`)
- **Mount offset:** `PAYLOAD_OFFSET_Z = -0.15` m below drone COM

## Target (`assets/target_marker.py`)

- **Spawn:** Kinematic cuboid 1.5 × 1.5 × 0.01 m (Gazebo `x_marker` footprint)
- **Collision:** disabled (visual aim point only)

## Path B (future): SDF → URDF → USD

1. Export `gazebo_models/x500_bombard/model.sdf` and `payload_cylinder` to URDF
2. Isaac Sim URDF Importer → `isaac_lab_tasks/drone_bombard/assets/usd/`
3. Disable rotor drives; keep force-based control in `PdVelocityController`
