"""SAC training script for drone fly-by drop policy."""

import argparse
import os

import rclpy
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from rl_navigation.drone_drop_env import DroneDropEnv

DEFAULT_TIMESTEPS = 500_000
DEFAULT_CHECKPOINT_FREQ = 5_000
DEFAULT_LOG_DIR = '/workspace/ros2_ws/rl_logs/sac_drop'
DEFAULT_CHECKPOINT_DIR = '/workspace/ros2_ws/rl_checkpoints'


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Train SAC policy for drone fly-by drop.')
    parser.add_argument(
        '--timesteps', type=int, default=DEFAULT_TIMESTEPS,
        help='Total environment steps to train (default: %(default)s)')
    parser.add_argument(
        '--checkpoint-freq', type=int, default=DEFAULT_CHECKPOINT_FREQ,
        help='Save checkpoint every N steps (default: %(default)s)')
    parser.add_argument(
        '--resume', type=str, default=None,
        metavar='PATH',
        help='Path to existing .zip checkpoint to resume training')
    parser.add_argument(
        '--log-dir', type=str, default=DEFAULT_LOG_DIR,
        help='TensorBoard log directory (default: %(default)s)')
    parser.add_argument(
        '--checkpoint-dir', type=str, default=DEFAULT_CHECKPOINT_DIR,
        help='Directory for saved checkpoints (default: %(default)s)')
    return parser.parse_args()


def main(args=None):
    """Entry point: ros2 run rl_navigation train_sac."""
    cli = _parse_args()

    os.makedirs(cli.log_dir, exist_ok=True)
    os.makedirs(cli.checkpoint_dir, exist_ok=True)

    print('=== SAC Drone Drop Training ===')
    print(f'  Timesteps  : {cli.timesteps:,}')
    print(f'  Log dir    : {cli.log_dir}')
    print(f'  Checkpoints: {cli.checkpoint_dir}')
    if cli.resume:
        print(f'  Resuming   : {cli.resume}')

    env = DroneDropEnv()

    checkpoint_callback = CheckpointCallback(
        save_freq=cli.checkpoint_freq,
        save_path=cli.checkpoint_dir,
        name_prefix='sac_drop',
        verbose=1,
    )

    if cli.resume:
        print(f'Loading existing model from {cli.resume} ...')
        model = SAC.load(
            cli.resume,
            env=env,
            tensorboard_log=cli.log_dir,
        )
    else:
        model = SAC(
            'MlpPolicy',
            env,
            learning_rate=3e-4,
            buffer_size=100_000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            learning_starts=1_000,
            policy_kwargs=dict(net_arch=[256, 256]),
            tensorboard_log=cli.log_dir,
            verbose=1,
        )

    print('Starting training...')
    model.learn(
        total_timesteps=cli.timesteps,
        callback=checkpoint_callback,
        reset_num_timesteps=(cli.resume is None),
    )

    final_path = os.path.join(cli.checkpoint_dir, 'sac_drop_final')
    model.save(final_path)
    print(f'Training complete. Final model saved to {final_path}.zip')

    env.close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
