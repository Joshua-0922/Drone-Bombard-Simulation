"""Rule-based CCIP baseline: fly straight at fixed speed, drop when d_impact ≤ threshold.

Provides a no-RL reference performance for comparison against Phase 1 SAC policy.
Runs N episodes and writes mean_drop_error, CEP50, success_rate as JSON + stdout.

Usage:
    ros2 run rl_navigation baseline_ccip --config <path/to/hyperparams.yaml>
    ros2 run rl_navigation baseline_ccip --episodes 50 --speed 5.0
"""

import argparse
import json
import math
import os

import numpy as np

from rl_navigation.drone_drop_env import DroneDropEnv

DEFAULT_EPISODES = 50
DEFAULT_APPROACH_SPEED = 5.0   # m/s horizontal approach speed
DEFAULT_OUTPUT_DIR = '/workspace/ros2_ws/rl_eval_results/baseline_ccip'


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Rule-based CCIP baseline (no RL policy).')
    parser.add_argument('--episodes', type=int, default=DEFAULT_EPISODES,
                        help='Number of episodes (default: %(default)s)')
    parser.add_argument('--speed', type=float, default=DEFAULT_APPROACH_SPEED,
                        help='Approach speed in m/s (default: %(default)s)')
    parser.add_argument('--config', type=str, default=None,
                        help='Path to hyperparams.yaml')
    parser.add_argument('--output-dir', type=str, default=DEFAULT_OUTPUT_DIR,
                        help='Output directory for results (default: %(default)s)')
    return parser.parse_args()


def _rule_action(obs, env, approach_speed):
    """Compute rule-based velocity command: fly toward target at fixed speed.

    obs layout (17-dim):
      [0-2]  pos ENU / 50m
      [3-5]  vel ENU / 15m
      [13]   rel_dx = (pos_x - target_x) / 50m
      [14]   rel_dy = (pos_y - target_y) / 50m

    Returns action array (5,) in [-1, 1].
    """
    # Recover relative displacement to target from obs
    rel_dx = float(obs[13]) * 50.0   # pos_x - target_x  (negative = target is ahead in +x)
    rel_dy = float(obs[14]) * 50.0   # pos_y - target_y

    # Direction from drone to target (negate rel_d)
    dx_to_target = -rel_dx
    dy_to_target = -rel_dy
    dist = math.sqrt(dx_to_target ** 2 + dy_to_target ** 2)

    if dist > 0.01:
        ux = dx_to_target / dist
        uy = dy_to_target / dist
    else:
        ux, uy = 0.0, 0.0

    # Velocity commands (normalised by action_vx_scale=15, action_vy_scale=5)
    vx_cmd = ux * approach_speed / 15.0
    vy_cmd = uy * approach_speed / 5.0

    # Clamp to [-1, 1]
    vx_cmd = max(-1.0, min(1.0, vx_cmd))
    vy_cmd = max(-1.0, min(1.0, vy_cmd))

    # No altitude change, no yaw, no manual drop
    return np.array([vx_cmd, vy_cmd, 0.0, 0.0, 0.0], dtype=np.float32)


def main(args=None):
    cli = _parse_args()
    os.makedirs(cli.output_dir, exist_ok=True)

    print('=== Rule-Based CCIP Baseline ===')
    print(f'  Episodes : {cli.episodes}')
    print(f'  Speed    : {cli.speed} m/s')
    print(f'  Output   : {cli.output_dir}')

    env = DroneDropEnv(config_path=cli.config)

    drop_errors = []
    episode_rewards = []

    for ep in range(cli.episodes):
        obs, _ = env.reset()
        done = False
        ep_reward = 0.0
        ep_drop_error = None

        while not done:
            action = _rule_action(obs, env, cli.speed)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward
            done = terminated or truncated

            if 'drop_error_actual_m' in info:
                ep_drop_error = info['drop_error_actual_m']

        episode_rewards.append(ep_reward)
        if ep_drop_error is not None:
            drop_errors.append(ep_drop_error)

        print(f'  Episode {ep + 1:3d}/{cli.episodes}: '
              f'reward={ep_reward:7.2f}  '
              f'drop_error={ep_drop_error if ep_drop_error is not None else "N/A"}')

    errors = np.array(drop_errors) if drop_errors else np.array([float('nan')])
    summary = {
        'n_episodes': cli.episodes,
        'n_drops': len(drop_errors),
        'mean_drop_error_m': float(np.nanmean(errors)),
        'std_drop_error_m': float(np.nanstd(errors)),
        'cep50_m': float(np.nanpercentile(errors, 50)),
        'success_rate_05m': float(np.nanmean(errors <= 0.5)),
        'mean_episode_reward': float(np.mean(episode_rewards)),
        'approach_speed_mps': cli.speed,
    }

    json_path = os.path.join(cli.output_dir, 'baseline_ccip_summary.json')
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)

    print(f'\n=== Results ===')
    print(f'  Drops recorded   : {summary["n_drops"]} / {cli.episodes}')
    print(f'  Mean miss dist   : {summary["mean_drop_error_m"]:.3f} m')
    print(f'  CEP50            : {summary["cep50_m"]:.3f} m')
    print(f'  Success rate     : {summary["success_rate_05m"]:.3f}')
    print(f'  Saved to         : {json_path}')

    env.close()


if __name__ == '__main__':
    main()
