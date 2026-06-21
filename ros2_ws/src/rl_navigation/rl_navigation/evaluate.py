"""Evaluation script for trained SAC drone drop policy.

v13 (terminal-reward env) note:
    The env TERMINATES when the drone reaches the success radius (no ballistic
    payload drop is modelled), so "miss distance / CEP / drop speed" are not real
    quantities here. The meaningful metrics are success rate, steps-to-reach,
    final/closest d_xy, deterministic episode reward, and the outcome breakdown.

    d_xy is recovered directly from the observation: obs[12], obs[13] are the
    target-relative offset normalised by POS_SCALE, so
        d_xy = POS_SCALE * sqrt(obs[12]^2 + obs[13]^2).
    (The old script read info['drop_error_actual_m'], a key the current env never
    emits → every miss-distance column came out NaN.)
"""

import argparse
import json
import math
import os
from collections import Counter

import matplotlib
matplotlib.use('Agg')  # headless rendering inside container
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from stable_baselines3 import SAC

from rl_navigation.drone_drop_env import DroneDropEnv, POS_SCALE

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


def _obs_d_xy(obs):
    """Recover horizontal distance-to-target (m) from a normalised observation."""
    return POS_SCALE * math.hypot(float(obs[12]), float(obs[13]))


def _run_evaluation(model, env, n_episodes):
    """Run n_episodes deterministically and collect per-episode metrics."""
    episodes = []          # list of per-episode dicts
    trajectories = []      # list of (x_arr, y_arr) per episode

    for ep in range(n_episodes):
        obs, _ = env.reset()
        done = False
        ep_reward = 0.0
        steps = 0
        min_d_xy = _obs_d_xy(obs)
        final_d_xy = min_d_xy
        outcome = 'unknown'
        success = False
        xs, ys = [], []

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward
            steps += 1
            done = terminated or truncated

            d_xy = _obs_d_xy(obs)
            min_d_xy = min(min_d_xy, d_xy)
            final_d_xy = d_xy

            # Record trajectory (ENU x=East, y=North)
            xs.append(float(obs[0]) * POS_SCALE)
            ys.append(float(obs[1]) * POS_SCALE)

            if done:
                success = bool(info.get('success', False))
                outcome = info.get('truncate_reason',
                                   'success' if success else 'unknown')

        episodes.append({
            'reward': ep_reward,
            'steps': steps,
            'success': success,
            'outcome': outcome,
            'final_d_xy': final_d_xy,
            'min_d_xy': min_d_xy,
        })
        trajectories.append((np.array(xs), np.array(ys)))

        print(f'  Episode {ep + 1:3d}/{n_episodes}: '
              f'reward={ep_reward:8.2f}  '
              f'steps={steps:4d}  '
              f'{"SUCCESS" if success else "FAIL   "}  '
              f'final_d_xy={final_d_xy:5.2f}m  '
              f'min_d_xy={min_d_xy:5.2f}m  '
              f'[{outcome}]')

    return episodes, trajectories


def _summarise(episodes):
    """Aggregate per-episode dicts into a summary dict."""
    n = len(episodes)
    rewards = np.array([e['reward'] for e in episodes]) if n else np.array([np.nan])
    successes = [e for e in episodes if e['success']]
    succ_steps = np.array([e['steps'] for e in successes]) if successes \
        else np.array([np.nan])
    final_d = np.array([e['final_d_xy'] for e in episodes]) if n else np.array([np.nan])
    min_d = np.array([e['min_d_xy'] for e in episodes]) if n else np.array([np.nan])
    outcomes = Counter(e['outcome'] for e in episodes)

    return {
        'n_episodes': n,
        'n_success': len(successes),
        'success_rate': (len(successes) / n) if n else float('nan'),
        'mean_steps_to_success': float(np.nanmean(succ_steps)),
        'median_steps_to_success': float(np.nanmedian(succ_steps)),
        'mean_final_d_xy_m': float(np.nanmean(final_d)),
        'mean_closest_d_xy_m': float(np.nanmean(min_d)),
        'min_closest_d_xy_m': float(np.nanmin(min_d)),
        'mean_episode_reward': float(np.nanmean(rewards)),
        'std_episode_reward': float(np.nanstd(rewards)),
        'outcomes': dict(outcomes),
    }


def _write_report(output_dir, summary):
    """Write evaluation_report.md and evaluation_summary.json."""
    json_path = os.path.join(output_dir, 'evaluation_summary.json')
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)

    md_path = os.path.join(output_dir, 'evaluation_report.md')
    with open(md_path, 'w') as f:
        f.write('# Drone Drop SAC Evaluation Report\n\n')
        f.write('> v13 terminal-reward env: episode ends at the success radius '
                '(no ballistic drop modelled). Metrics below are approach/'
                'navigation metrics, not CEP.\n\n')
        f.write('## Metrics\n\n')
        f.write('| Metric | Value |\n')
        f.write('|--------|-------|\n')
        f.write(f'| Episodes evaluated | {summary["n_episodes"]} |\n')
        f.write(f'| Successes (reached radius) | {summary["n_success"]} |\n')
        f.write(f'| **Success rate** | **{summary["success_rate"]:.3f}** |\n')
        f.write(f'| Mean steps-to-success | {summary["mean_steps_to_success"]:.1f} |\n')
        f.write(f'| Median steps-to-success | {summary["median_steps_to_success"]:.1f} |\n')
        f.write(f'| Mean final d_xy (m) | {summary["mean_final_d_xy_m"]:.3f} |\n')
        f.write(f'| Mean closest d_xy (m) | {summary["mean_closest_d_xy_m"]:.3f} |\n')
        f.write(f'| Best closest d_xy (m) | {summary["min_closest_d_xy_m"]:.3f} |\n')
        f.write(f'| Mean episode reward | {summary["mean_episode_reward"]:.2f} |\n')
        f.write(f'| Std episode reward | {summary["std_episode_reward"]:.2f} |\n')
        f.write('\n## Outcome breakdown\n\n')
        f.write('| Outcome | Count |\n|---------|-------|\n')
        for reason, count in sorted(summary['outcomes'].items(),
                                    key=lambda kv: -kv[1]):
            f.write(f'| {reason} | {count} |\n')


def _plot_episode_rewards(output_dir, episodes):
    fig, ax = plt.subplots(figsize=(10, 4))
    rewards = [e['reward'] for e in episodes]
    colors = ['tab:green' if e['success'] else 'tab:red' for e in episodes]
    ax.scatter(range(len(rewards)), rewards, c=colors, s=30, zorder=3)
    ax.plot(rewards, color='gray', linewidth=1, zorder=2)
    ax.axhline(0, color='black', linewidth=0.6)
    ax.set_xlabel('Episode')
    ax.set_ylabel('Total Reward')
    ax.set_title('Per-Episode Rewards (green=success, red=fail)')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'episode_rewards.png'), dpi=120)
    plt.close(fig)


def _plot_closest_d_xy_hist(output_dir, episodes):
    vals = [e['min_d_xy'] for e in episodes]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(vals, bins=12, edgecolor='black')
    ax.set_xlabel('Closest d_xy to target (m)')
    ax.set_ylabel('Count')
    ax.set_title('Closest Approach Distribution')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'closest_d_xy_dist.png'), dpi=120)
    plt.close(fig)


def _plot_outcomes(output_dir, summary):
    items = sorted(summary['outcomes'].items(), key=lambda kv: -kv[1])
    if not items:
        return
    labels, counts = zip(*items)
    colors = ['tab:green' if l == 'success' else 'tab:red' for l in labels]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.bar(labels, counts, color=colors, edgecolor='black')
    ax.set_ylabel('Count')
    ax.set_title('Episode Outcome Breakdown')
    plt.xticks(rotation=30, ha='right')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'outcome_breakdown.png'), dpi=120)
    plt.close(fig)


def _plot_trajectory(output_dir, trajectories, target_xy):
    """Plot 2D overhead trajectory of the final episode."""
    if not trajectories:
        return
    xs, ys = trajectories[-1]
    if len(xs) == 0:
        return
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xs, ys, 'b-', linewidth=1.5, label='Drone path')
    ax.plot(xs[0], ys[0], 'gs', markersize=10, label='Start')
    if len(xs) > 1:
        ax.plot(xs[-1], ys[-1], 'r^', markersize=10, label='End')
    ax.plot(target_xy[0], target_xy[1], 'rx', markersize=15,
            markeredgewidth=3, label='Target')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Trajectory (top view, final episode)')
    ax.legend()
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'trajectory_top.png'), dpi=120)
    plt.close(fig)


def main(args=None):
    """Entry point: ros2 run rl_navigation evaluate --model <path>."""
    cli = _parse_args()

    os.makedirs(cli.output_dir, exist_ok=True)

    print('=== SAC Drone Drop Evaluation ===')
    print(f'  Model     : {cli.model}')
    print(f'  Episodes  : {cli.episodes}')
    print(f'  Output dir: {cli.output_dir}')

    env = DroneDropEnv(config_path=cli.config)
    model = SAC.load(cli.model, env=env)
    target_xy = (env._cfg_target_x, env._cfg_target_y)

    print('\nRunning evaluation episodes...')
    episodes, trajectories = _run_evaluation(model, env, cli.episodes)

    print('\nGenerating outputs...')
    summary = _summarise(episodes)
    _write_report(cli.output_dir, summary)
    _plot_episode_rewards(cli.output_dir, episodes)
    _plot_closest_d_xy_hist(cli.output_dir, episodes)
    _plot_outcomes(cli.output_dir, summary)
    _plot_trajectory(cli.output_dir, trajectories, target_xy)

    print(f'\nResults saved to {cli.output_dir}')
    print(f'  Success rate         : {summary["success_rate"]:.3f} '
          f'({summary["n_success"]}/{summary["n_episodes"]})')
    print(f'  Mean steps-to-success: {summary["mean_steps_to_success"]:.1f}')
    print(f'  Mean closest d_xy    : {summary["mean_closest_d_xy_m"]:.3f} m')
    print(f'  Mean episode reward  : {summary["mean_episode_reward"]:.2f}')
    print(f'  Outcomes             : {summary["outcomes"]}')

    env.close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
