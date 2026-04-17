"""Evaluation script for trained SAC drone drop policy."""

import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')  # headless rendering inside container
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from stable_baselines3 import SAC

from rl_navigation.drone_drop_env import DroneDropEnv

DEFAULT_EPISODES = 20
DEFAULT_OUTPUT_DIR = '/workspace/ros2_ws/rl_eval_results'


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Evaluate a trained SAC drone drop model.')
    parser.add_argument(
        '--model', type=str, required=True,
        help='Path to .zip model file')
    parser.add_argument(
        '--episodes', type=int, default=DEFAULT_EPISODES,
        help='Number of evaluation episodes (default: %(default)s)')
    parser.add_argument(
        '--output-dir', type=str, default=DEFAULT_OUTPUT_DIR,
        help='Directory for evaluation outputs (default: %(default)s)')
    parser.add_argument(
        '--config', type=str, default=None,
        help='Path to hyperparams.yaml (uses env default if omitted)')
    return parser.parse_args()


def _run_evaluation(model, env, n_episodes):
    """Run n_episodes and collect per-episode metrics."""
    drop_errors = []
    drop_speeds = []
    episode_rewards = []
    trajectories = []   # list of (x_arr, y_arr) per episode

    for ep in range(n_episodes):
        obs, _ = env.reset()
        done = False
        ep_reward = 0.0
        ep_drop_error = None
        ep_drop_speed = None
        xs, ys = [], []

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward
            done = terminated or truncated

            # Record trajectory (ENU x=East, y=North)
            x = float(obs[0]) * 50.0
            y = float(obs[1]) * 50.0
            xs.append(x)
            ys.append(y)

            if 'drop_error_actual_m' in info:
                ep_drop_error = info['drop_error_actual_m']
            # Estimate drop speed from velocity obs
            vx = float(obs[3]) * 15.0
            vy = float(obs[4]) * 15.0
            ep_drop_speed = float(np.sqrt(vx**2 + vy**2))

        episode_rewards.append(ep_reward)
        trajectories.append((np.array(xs), np.array(ys)))
        if ep_drop_error is not None:
            drop_errors.append(ep_drop_error)
            drop_speeds.append(ep_drop_speed)

        print(f'  Episode {ep + 1:3d}/{n_episodes}: '
              f'reward={ep_reward:7.2f}  '
              f'drop_error={ep_drop_error if ep_drop_error is not None else "N/A"}')

    return drop_errors, drop_speeds, episode_rewards, trajectories


def _write_report(output_dir, drop_errors, drop_speeds, episode_rewards):
    """Write evaluation_report.md and evaluation_summary.json."""
    errors = np.array(drop_errors) if drop_errors else np.array([float('nan')])
    speeds = np.array(drop_speeds) if drop_speeds else np.array([float('nan')])

    summary = {
        'n_episodes': len(episode_rewards),
        'n_drops': len(drop_errors),
        'mean_drop_error_m': float(np.nanmean(errors)),
        'std_drop_error_m': float(np.nanstd(errors)),
        'min_drop_error_m': float(np.nanmin(errors)),
        'max_drop_error_m': float(np.nanmax(errors)),
        'cep50_m': float(np.nanpercentile(errors, 50)),
        'success_rate_05m': float(np.nanmean(errors <= 0.5)),
        'mean_drop_speed_mps': float(np.nanmean(speeds)),
        'mean_episode_reward': float(np.mean(episode_rewards)),
        'std_episode_reward': float(np.std(episode_rewards)),
    }

    # JSON
    json_path = os.path.join(output_dir, 'evaluation_summary.json')
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)

    # Markdown
    md_path = os.path.join(output_dir, 'evaluation_report.md')
    with open(md_path, 'w') as f:
        f.write('# Drone Drop SAC Evaluation Report\n\n')
        f.write('## Metrics\n\n')
        f.write('| Metric | Value |\n')
        f.write('|--------|-------|\n')
        f.write(f'| Episodes evaluated | {summary["n_episodes"]} |\n')
        f.write(f'| Episodes with drop | {summary["n_drops"]} |\n')
        f.write(f'| Mean miss distance (m) | {summary["mean_drop_error_m"]:.3f} |\n')
        f.write(f'| Std miss distance (m)  | {summary["std_drop_error_m"]:.3f} |\n')
        f.write(f'| Min miss distance (m)  | {summary["min_drop_error_m"]:.3f} |\n')
        f.write(f'| Max miss distance (m)  | {summary["max_drop_error_m"]:.3f} |\n')
        f.write(f'| CEP50 (m)              | {summary["cep50_m"]:.3f} |\n')
        f.write(f'| Success rate (≤0.5m)   | {summary["success_rate_05m"]:.3f} |\n')
        f.write(f'| Mean drop speed (m/s)  | {summary["mean_drop_speed_mps"]:.2f} |\n')
        f.write(f'| Mean episode reward    | {summary["mean_episode_reward"]:.2f} |\n')
        f.write(f'| Std episode reward     | {summary["std_episode_reward"]:.2f} |\n')

    return summary


def _plot_drop_error_hist(output_dir, drop_errors):
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(drop_errors, bins=10, edgecolor='black')
    ax.set_xlabel('Miss Distance (m)')
    ax.set_ylabel('Count')
    ax.set_title('Drop Error Distribution')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'drop_error_dist.png'), dpi=120)
    plt.close(fig)


def _plot_episode_rewards(output_dir, episode_rewards):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(episode_rewards, marker='o', markersize=3)
    ax.set_xlabel('Episode')
    ax.set_ylabel('Total Reward')
    ax.set_title('Per-Episode Rewards')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'episode_rewards.png'), dpi=120)
    plt.close(fig)


def _plot_trajectory(output_dir, trajectories):
    """Plot 2D overhead trajectory of the final episode."""
    if not trajectories:
        return
    xs, ys = trajectories[-1]
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xs, ys, 'b-', linewidth=1.5, label='Drone path')
    ax.plot(xs[0], ys[0], 'gs', markersize=10, label='Start')
    if len(xs) > 1:
        ax.plot(xs[-1], ys[-1], 'r^', markersize=10, label='End')
    # Mark target
    ax.plot(11.0, 10.0, 'rx', markersize=15, markeredgewidth=3, label='Target')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Trajectory (top view, final episode)')
    ax.legend()
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'trajectory_top.png'), dpi=120)
    plt.close(fig)


def _plot_speed_vs_accuracy(output_dir, drop_speeds, drop_errors):
    if not drop_speeds:
        return
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(drop_speeds, drop_errors, alpha=0.7)
    ax.set_xlabel('Drop Speed (m/s)')
    ax.set_ylabel('Miss Distance (m)')
    ax.set_title('Drop Speed vs Miss Distance')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'speed_vs_accuracy.png'), dpi=120)
    plt.close(fig)


def main(args=None):
    """Entry point: ros2 run rl_navigation evaluate --model <path>."""
    cli = _parse_args()

    os.makedirs(cli.output_dir, exist_ok=True)

    print(f'=== SAC Drone Drop Evaluation ===')
    print(f'  Model     : {cli.model}')
    print(f'  Episodes  : {cli.episodes}')
    print(f'  Output dir: {cli.output_dir}')

    env = DroneDropEnv(config_path=cli.config)
    model = SAC.load(cli.model, env=env)

    print('\nRunning evaluation episodes...')
    drop_errors, drop_speeds, episode_rewards, trajectories = \
        _run_evaluation(model, env, cli.episodes)

    print('\nGenerating outputs...')
    summary = _write_report(cli.output_dir, drop_errors, drop_speeds, episode_rewards)

    if drop_errors:
        _plot_drop_error_hist(cli.output_dir, drop_errors)
    _plot_episode_rewards(cli.output_dir, episode_rewards)
    _plot_trajectory(cli.output_dir, trajectories)
    if drop_speeds:
        _plot_speed_vs_accuracy(cli.output_dir, drop_speeds, drop_errors)

    print(f'\nResults saved to {cli.output_dir}')
    print(f'  Mean miss distance : {summary["mean_drop_error_m"]:.3f} m')
    print(f'  Mean drop speed    : {summary["mean_drop_speed_mps"]:.2f} m/s')
    print(f'  Mean episode reward: {summary["mean_episode_reward"]:.2f}')

    env.close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
