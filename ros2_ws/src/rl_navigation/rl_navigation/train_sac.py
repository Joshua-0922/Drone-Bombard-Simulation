"""SAC training script for drone fly-by drop policy."""

import argparse
import glob as _glob
import os
import shutil
import signal
import sys
import time

import numpy as np
import rclpy
import yaml
import wandb
from ament_index_python.packages import get_package_share_directory
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback, CallbackList, CheckpointCallback)
from stable_baselines3.common.vec_env import SubprocVecEnv
from wandb.integration.sb3 import WandbCallback

from rl_navigation.drone_drop_env import DroneDropEnv

_DEFAULT_CONFIG = os.path.join(
    get_package_share_directory('rl_navigation'), 'config', 'hyperparams.yaml')

# Will be set in main() so the SIGTERM handler can reference them
_model = None
_checkpoint_dir = None


def _emergency_save(signum, frame):
    """SIGTERM handler: persist model + replay buffer before Spot VM dies."""
    global _model, _checkpoint_dir
    if _model is not None and _checkpoint_dir is not None:
        path = os.path.join(_checkpoint_dir, 'sac_drop_preempt')
        _model.save(path)
        _model.save_replay_buffer(path + '_replay')
        if wandb.run:
            wandb.save(path + '.zip')
            wandb.finish()
    sys.exit(0)


class WandbMetricsCallback(BaseCallback):
    """Forward SB3 rollout/train metrics + env custom metrics to WandB."""

    def __init__(self):
        super().__init__()
        self._ep_drop_errors: list = []
        self._step_d_xy: list = []
        self._step_rew_drop: list = []
        self._total_episodes: int = 0
        self._total_drops: int = 0
        self._total_auto_drops: int = 0
        self._total_success: int = 0
        self._total_jackpot: int = 0

    def _on_step(self):
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [])
        for info, done in zip(infos, dones):
            if 'd_xy' in info:
                self._step_d_xy.append(info['d_xy'])
            if 'rew_drop' in info:
                self._step_rew_drop.append(info['rew_drop'])

            if done:
                self._total_episodes += 1
                if 'drop_error_actual_m' in info:
                    self._ep_drop_errors.append(info['drop_error_actual_m'])
                    self._total_drops += 1
                    if info.get('drop_trigger') != 'random':
                        self._total_auto_drops += 1
                    if info.get('is_success'):
                        self._total_success += 1
                    if info.get('jackpot'):
                        self._total_jackpot += 1
        return True

    def _on_rollout_end(self):
        if not wandb.run:
            return
        log_dict = {}
        for key in ('rollout/ep_rew_mean', 'rollout/ep_len_mean',
                     'train/actor_loss', 'train/critic_loss',
                     'train/ent_coef', 'train/ent_coef_loss',
                     'train/learning_rate', 'train/n_updates',
                     'time/fps', 'time/episodes', 'time/total_timesteps'):
            val = self.logger.name_to_value.get(key)
            if val is not None:
                log_dict[key] = val

        def _mean(lst):
            return sum(lst) / len(lst) if lst else None

        if self._step_d_xy:
            log_dict['env/d_xy'] = _mean(self._step_d_xy)
            self._step_d_xy.clear()
        if self._step_rew_drop:
            log_dict['env/rew_drop'] = _mean(self._step_rew_drop)
            self._step_rew_drop.clear()

        if self._ep_drop_errors:
            log_dict['env/drop_error'] = _mean(self._ep_drop_errors)
            self._ep_drop_errors.clear()

        log_dict['env/total_episodes'] = self._total_episodes
        log_dict['env/total_drops'] = self._total_drops
        log_dict['env/total_auto_drops'] = self._total_auto_drops
        log_dict['env/total_success'] = self._total_success
        log_dict['env/total_jackpot'] = self._total_jackpot

        if log_dict:
            # NOTE: time/total_timesteps 는 위 L123 fetch 에서 이미 들어가지만,
            # 그게 None 일 경우 대비 fallback 으로 num_timesteps 사용.
            log_dict.setdefault('time/total_timesteps', self.num_timesteps)
            wandb.log(log_dict, step=self.num_timesteps)


class BestModelCallback(BaseCallback):
    """Save model when mean episode reward reaches a new best.

    Checks every ``eval_freq`` steps using the rolling ``ep_rew_mean``
    from the SB3 logger (same metric printed in the rollout table).
    No separate eval env needed — uses training rollout stats.
    """

    def __init__(self, save_path, eval_freq=10_000, verbose=1):
        super().__init__(verbose)
        self._save_path = save_path
        self._eval_freq = eval_freq
        self._best_mean_reward = -float('inf')
        os.makedirs(save_path, exist_ok=True)

    def _on_step(self):
        if self.num_timesteps % self._eval_freq != 0:
            return True

        ep_rew = self.logger.name_to_value.get('rollout/ep_rew_mean')
        if ep_rew is None:
            return True

        if ep_rew > self._best_mean_reward:
            self._best_mean_reward = ep_rew
            path = os.path.join(self._save_path, 'best_model')
            self.model.save(path)
            if self.verbose:
                print(f'[BestModel] New best mean reward: {ep_rew:.2f} '
                      f'at {self.num_timesteps} steps → saved {path}.zip')
            if wandb.run:
                wandb.log({
                    'eval/best_mean_reward': ep_rew,
                    'eval/best_model_step': self.num_timesteps,
                }, step=self.num_timesteps)
        return True


class MilestoneArchiveCallback(BaseCallback):
    """Archive model weights (no replay buffer) at fixed step intervals.

    Archived files are saved to a separate directory that is exempt from
    the rolling checkpoint cleanup logic.
    """

    def __init__(self, archive_dir, archive_freq=50_000, verbose=1):
        super().__init__(verbose)
        self._archive_dir = archive_dir
        self._archive_freq = archive_freq
        os.makedirs(archive_dir, exist_ok=True)

    def _on_step(self):
        if self.num_timesteps % self._archive_freq != 0:
            return True

        name = f'sac_drop_milestone_{self.num_timesteps}_steps'
        path = os.path.join(self._archive_dir, name)
        # Save weights only (no replay buffer) to conserve disk
        self.model.save(path)
        if self.verbose:
            size_mb = os.path.getsize(path + '.zip') / (1024 * 1024)
            print(f'[Archive] Milestone at {self.num_timesteps} steps '
                  f'→ {path}.zip ({size_mb:.1f} MB)')
        return True


class CleanupOldCheckpointsCallback(BaseCallback):
    """Keep only the N most-recent periodic checkpoints to cap disk usage."""

    def __init__(self, checkpoint_dir, name_prefix, keep_last=5, verbose=0):
        super().__init__(verbose)
        self._dir = checkpoint_dir
        self._prefix = name_prefix
        self._keep = keep_last

    def _on_step(self):
        files = sorted(_glob.glob(
            os.path.join(self._dir, f'{self._prefix}_*_steps.zip')))
        for old in files[:-self._keep]:
            os.remove(old)
            if self.verbose:
                print(f'[Cleanup] Deleted old checkpoint: {old}')
        return True


class DropEpisodeRecorderCallback(BaseCallback):
    """Save full trajectory data for episodes where a drop was triggered.

    Each drop episode is stored as a compressed .npz containing the action
    sequence (for replay), observations, rewards, and per-step metrics.
    An index.csv is maintained for quick scanning.
    """

    def __init__(self, save_dir, verbose=0):
        super().__init__(verbose)
        self._save_dir = save_dir
        self._best_dir = os.path.join(save_dir, 'best_drops')
        os.makedirs(save_dir, exist_ok=True)
        os.makedirs(self._best_dir, exist_ok=True)
        self._buffers = {}
        self._drop_count = 0
        self._best_drop_error = float('inf')
        self._index_path = os.path.join(save_dir, 'index.csv')
        if not os.path.exists(self._index_path):
            with open(self._index_path, 'w') as f:
                f.write('filename,timestep,drop_error_m,is_success,'
                        'episode_reward,n_steps\n')

    def _get_buf(self, i):
        if i not in self._buffers:
            self._buffers[i] = {
                'obs': [], 'act': [], 'rew': [],
                'd_xy': [], 'd_impact': [],
            }
        return self._buffers[i]

    def _on_step(self):
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [])
        actions = self.locals.get('actions')
        rewards = self.locals.get('rewards')
        new_obs = self.locals.get('new_obs')

        for i, (info, done) in enumerate(zip(infos, dones)):
            buf = self._get_buf(i)
            buf['act'].append(actions[i].copy())
            buf['rew'].append(float(rewards[i]))
            buf['obs'].append(new_obs[i].copy())
            buf['d_xy'].append(info.get('d_xy', float('nan')))
            buf['d_impact'].append(info.get('d_impact', float('nan')))

            if done:
                # Replace last obs with terminal observation (DummyVecEnv
                # auto-resets and overwrites new_obs with the reset obs)
                terminal_obs = info.get('terminal_observation')
                if terminal_obs is not None:
                    buf['obs'][-1] = terminal_obs.copy()

                if 'drop_error_actual_m' in info:
                    self._drop_count += 1
                    drop_error = info['drop_error_actual_m']
                    is_success = info.get('is_success', False)
                    ep_reward = info.get('episode_reward', sum(buf['rew']))
                    n_steps = len(buf['act'])

                    fname = (f'drop_{self._drop_count:04d}'
                             f'_step{self.num_timesteps}'
                             f'_err{drop_error:.2f}m.npz')

                    np.savez_compressed(
                        os.path.join(self._save_dir, fname),
                        actions=np.array(buf['act'], dtype=np.float32),
                        observations=np.array(buf['obs'], dtype=np.float32),
                        rewards=np.array(buf['rew'], dtype=np.float32),
                        d_xy=np.array(buf['d_xy'], dtype=np.float32),
                        d_impact=np.array(buf['d_impact'], dtype=np.float32),
                        drop_error=np.float32(drop_error),
                        is_success=np.bool_(is_success),
                        episode_reward=np.float32(ep_reward),
                        timestep=np.int64(self.num_timesteps),
                        n_steps=np.int32(n_steps),
                    )

                    with open(self._index_path, 'a') as f:
                        f.write(f'{fname},{self.num_timesteps},'
                                f'{drop_error:.4f},{is_success},'
                                f'{ep_reward:.2f},{n_steps}\n')

                    if (drop_error < self._best_drop_error
                            and info.get('drop_trigger') == 'auto'):
                        self._best_drop_error = drop_error
                        best_path = os.path.join(
                            self._best_dir,
                            f'best_err{drop_error:.2f}m_step{self.num_timesteps}')
                        self.model.save(best_path)
                        if self.verbose:
                            print(f'[DropRecorder] NEW BEST (auto)! {drop_error:.3f}m '
                                  f'→ model saved to {best_path}.zip')

                    if self.verbose:
                        print(f'[DropRecorder] Saved {fname} '
                              f'(err={drop_error:.3f}m, rew={ep_reward:.1f},'
                              f' {n_steps} steps)')

                self._buffers[i] = {
                    'obs': [], 'act': [], 'rew': [],
                    'd_xy': [], 'd_impact': [],
                }
        return True


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Train SAC policy for drone fly-by drop.')
    parser.add_argument(
        '--config', type=str,
        default=_DEFAULT_CONFIG,
        help='Path to hyperparams.yaml (default: package config/hyperparams.yaml)')
    parser.add_argument(
        '--timesteps', type=int, default=None,
        help='Override total_timesteps from config')
    parser.add_argument(
        '--resume', type=str, default=None, metavar='PATH',
        help='Path to existing .zip checkpoint to resume training')
    parser.add_argument(
        '--checkpoint-dir', type=str, default=None,
        help='Override checkpoint directory from config')
    return parser.parse_args()


def main(args=None):
    """Entry point: ros2 run rl_navigation train_sac."""
    global _model, _checkpoint_dir

    signal.signal(signal.SIGTERM, _emergency_save)

    cli = _parse_args()

    # --- Load config ---
    config_path = os.path.realpath(cli.config)
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    cfg_train = cfg.get('training', {})
    cfg_sac = cfg.get('sac', {})
    cfg_wandb = cfg.get('wandb', {})

    total_timesteps = cli.timesteps or cfg_train.get('total_timesteps', 500_000)
    checkpoint_dir = cli.checkpoint_dir or cfg_train.get(
        'checkpoint_dir', '/workspace/ros2_ws/rl_checkpoints')
    checkpoint_freq = cfg_train.get('checkpoint_freq', 5_000)
    max_checkpoints_kept = cfg_train.get('max_checkpoints_kept', 5)
    eval_freq = cfg_train.get('eval_freq', 10_000)   # L6: read from yaml
    _checkpoint_dir = checkpoint_dir

    best_model_dir = os.path.join(checkpoint_dir, 'best_model')
    archive_dir = os.path.join(checkpoint_dir, 'archive')
    drop_episodes_dir = os.path.join(checkpoint_dir, 'drop_episodes')
    os.makedirs(checkpoint_dir, exist_ok=True)

    # --- Prune stale WandB offline-run directories (older than 7 days) ---
    _wandb_dir = os.path.join(os.getcwd(), 'wandb')
    if os.path.isdir(_wandb_dir):
        cutoff = time.time() - 7 * 86400
        for entry in os.scandir(_wandb_dir):
            if entry.is_dir() and entry.stat().st_mtime < cutoff:
                shutil.rmtree(entry.path, ignore_errors=True)

    # --- WandB ---
    wandb.init(
        project=cfg_wandb.get('project', 'drone-bombard-sac'),
        entity=cfg_wandb.get('entity') or None,
        name=cfg_wandb.get('run_name') or None,
        tags=cfg_wandb.get('tags', []),
        config=cfg,
        resume='allow',   # re-attaches to existing run after preemption
    )

    # ---- WandB x축 자동 매핑 (2026-05-23 추가) ----
    # 모든 누적 count metric (env/total_*) 의 x축을 env/total_episodes 로 자동 설정.
    # 대시보드 UI 수동 조작 불필요. rate 그래프의 binary 진동 문제 해결.
    wandb.define_metric('env/total_episodes')
    wandb.define_metric('env/total_*', step_metric='env/total_episodes')

    num_envs = cfg_train.get('num_envs', 1)

    print('=== SAC Drone Drop Training ===')
    print(f'  Config     : {config_path}')
    print(f'  Timesteps  : {total_timesteps:,}')
    print(f'  Checkpoints: {checkpoint_dir}')
    print(f'  Best model : {best_model_dir}')
    print(f'  Archive    : {archive_dir}')
    print(f'  Drop eps   : {drop_episodes_dir}')
    print(f'  Device     : {cfg_sac.get("device", "cuda")}')
    print(f'  Num envs   : {num_envs}')
    print(f'  WandB run  : {wandb.run.name}')
    if cli.resume:
        print(f'  Resuming   : {cli.resume}')

    if num_envs > 1:
        stagger_secs = cfg_train.get('env_stagger_secs', 10)

        def _make_env(rank, cfg_path):
            def _init():
                # Stagger init to prevent CPU spike and PX4 lockstep timeouts
                time.sleep(rank * stagger_secs)
                return DroneDropEnv(config_path=cfg_path, instance_id=rank)
            return _init
        env = SubprocVecEnv([_make_env(i, config_path) for i in range(num_envs)])
    else:
        env = DroneDropEnv(config_path=config_path, instance_id=0)

    # --- Callbacks ---
    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=checkpoint_dir,
        name_prefix='sac_drop',
        save_replay_buffer=False,  # only SIGTERM handler saves replay buffer
        verbose=1,
    )
    cleanup_callback = CleanupOldCheckpointsCallback(
        checkpoint_dir, 'sac_drop', keep_last=max_checkpoints_kept, verbose=1)
    best_model_callback = BestModelCallback(
        save_path=best_model_dir, eval_freq=eval_freq, verbose=1)
    milestone_callback = MilestoneArchiveCallback(
        archive_dir=archive_dir, archive_freq=50_000, verbose=1)
    wandb_callback = WandbCallback(
        gradient_save_freq=cfg_wandb.get('log_freq', 100),
        model_save_path=checkpoint_dir,
        verbose=2,
    )
    wandb_metrics_callback = WandbMetricsCallback()
    drop_recorder_callback = DropEpisodeRecorderCallback(
        save_dir=drop_episodes_dir, verbose=1)
    callbacks = CallbackList([
        checkpoint_callback, cleanup_callback,
        best_model_callback, milestone_callback,
        wandb_callback, wandb_metrics_callback,
        drop_recorder_callback])

    # --- Model ---
    if cli.resume:
        print(f'Loading existing model from {cli.resume} ...')
        _model = SAC.load(
            cli.resume,
            env=env,
            device=cfg_sac.get('device', 'cuda'),
            tensorboard_log=None,
        )
        # Reload replay buffer if preempt checkpoint exists.
        # Skip if num_envs changed — buffer n_envs must match current env.
        replay_path = cli.resume.replace('.zip', '_replay.pkl')
        if os.path.exists(replay_path) and num_envs <= 1:
            print(f'Loading replay buffer from {replay_path} ...')
            _model.load_replay_buffer(replay_path)
        elif os.path.exists(replay_path):
            print(f'Skipping replay buffer (num_envs={num_envs}, '
                  f'buffer was single-env — shape mismatch)')
    else:
        _model = SAC(
            'MlpPolicy',
            env,
            learning_rate=cfg_sac.get('learning_rate', 3e-4),
            buffer_size=cfg_sac.get('buffer_size', 100_000),
            batch_size=cfg_sac.get('batch_size', 256),
            tau=cfg_sac.get('tau', 0.005),
            gamma=cfg_sac.get('gamma', 0.99),
            learning_starts=cfg_sac.get('learning_starts', 1_000),
            gradient_steps=cfg_sac.get('gradient_steps', 1),
            policy_kwargs=dict(net_arch=cfg_sac.get('net_arch', [256, 256])),
            device=cfg_sac.get('device', 'cuda'),
            tensorboard_log=None,
            verbose=1,
        )

    print('Starting training...')
    _model.learn(
        total_timesteps=total_timesteps,
        callback=callbacks,
        reset_num_timesteps=(cli.resume is None),
    )

    # --- Save final model ---
    final_path = os.path.join(checkpoint_dir, 'sac_drop_final')
    _model.save(final_path)
    print(f'Training complete. Final model saved to {final_path}.zip')

    # --- Upload final model as WandB artifact ---
    if cfg_wandb.get('save_model_artifact', True) and wandb.run:
        artifact = wandb.Artifact('sac_drop_final', type='model')
        artifact.add_file(final_path + '.zip')
        wandb.log_artifact(artifact)
        print('Final model uploaded to WandB as artifact.')

    wandb.finish()
    env.close()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
