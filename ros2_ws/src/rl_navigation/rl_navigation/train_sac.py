"""SAC training script for drone approach-to-X navigation policy."""

import argparse
import glob as _glob
import os
import shutil
import signal
import sys
import time

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


_REACHED_CLOSE_THRESHOLD_M = 3.0  # drone within this distance = "reached close"


class WandbMetricsCallback(BaseCallback):
    """Forward SB3 rollout/train metrics + vision-approach env metrics to WandB.

    Metrics logged per rollout:
      Navigation:
        env/mean_d_xy         — mean XY distance to X marker (↓ better)
        env/ep_final_d_xy     — final d_xy when episode ends (↓ better)
        env/ep_best_d_xy      — closest approach within episode (↓ better)

      Reward breakdown:
        env/rew_dist          — distance-gradient reward component
        env/rew_orient        — heading-alignment reward component
        env/rew_vision        — YOLO centering reward component
        env/rew_ctrl          — stability/control penalty (r2, negative)
        env/cos_heading       — cosine of angle between velocity and target bearing

      Vision quality:
        env/target_lost_rate  — fraction of steps with no YOLO detection (↓ better)

      Episode outcomes (cumulative):
        env/total_episodes
        env/reached_close     — episodes where best d_xy ≤ 3 m
        env/end_timeout       — episodes ended by step-limit
        env/end_crash         — episodes ended by crash
        env/end_stagnation    — episodes ended by stagnation
        env/end_out_of_range  — episodes ended by leaving boundary
        env/end_ang_vel       — episodes ended by attitude instability
    """

    def __init__(self):
        super().__init__()
        # Per-step buffers (cleared each rollout)
        self._step_d_xy: list = []
        self._step_rew_dist: list = []
        self._step_rew_orient: list = []
        self._step_rew_vision: list = []
        self._step_rew_ctrl: list = []
        self._step_cos_heading: list = []
        self._step_target_lost: int = 0
        self._step_total: int = 0

        # Per-episode buffers (populated on done, cleared each rollout)
        self._ep_d_xy_min: dict = {}   # env_idx → best d_xy so far this episode
        self._ep_final_d_xy: list = []
        self._ep_best_d_xy: list = []
        self._ep_reward: list = []

        # Cumulative episode counters
        self._total_episodes: int = 0
        self._reached_close: int = 0
        self._end_timeout: int = 0
        self._end_crash: int = 0
        self._end_stagnation: int = 0
        self._end_out_of_range: int = 0
        self._end_ang_vel: int = 0
        self._end_inverted: int = 0
        self._end_overspeed: int = 0
        self._end_max_altitude: int = 0
        self._end_ekf_drift: int = 0
        self._end_unknown: int = 0

    def _on_step(self):
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [])

        for i, (info, done) in enumerate(zip(infos, dones)):
            # --- Per-step accumulation ---
            if 'd_xy' in info:
                d = info['d_xy']
                self._step_d_xy.append(d)
                # Track best (min) d_xy per env for this episode
                if i not in self._ep_d_xy_min or d < self._ep_d_xy_min[i]:
                    self._ep_d_xy_min[i] = d
            for key, buf in (
                ('rew_dist',    self._step_rew_dist),
                ('rew_orient',  self._step_rew_orient),
                ('rew_vision',  self._step_rew_vision),
                ('rew_ctrl',    self._step_rew_ctrl),
                ('cos_heading', self._step_cos_heading),
            ):
                if key in info:
                    buf.append(info[key])
            if info.get('target_lost'):
                self._step_target_lost += 1
            self._step_total += 1

            # --- On episode end ---
            if done:
                self._total_episodes += 1
                if 'd_xy' in info:
                    self._ep_final_d_xy.append(info['d_xy'])
                if i in self._ep_d_xy_min:
                    best = self._ep_d_xy_min.pop(i)
                    self._ep_best_d_xy.append(best)
                    if best <= _REACHED_CLOSE_THRESHOLD_M:
                        self._reached_close += 1
                if 'episode_reward' in info:
                    self._ep_reward.append(info['episode_reward'])
                reason = info.get('truncate_reason', '')
                if reason == 'crash':
                    self._end_crash += 1
                elif reason == 'stagnation':
                    self._end_stagnation += 1
                elif reason == 'timeout':
                    self._end_timeout += 1
                elif reason == 'out_of_range':
                    self._end_out_of_range += 1
                elif reason == 'ang_vel':
                    self._end_ang_vel += 1
                elif reason == 'inverted':
                    self._end_inverted += 1
                elif reason == 'overspeed':
                    self._end_overspeed += 1
                elif reason == 'max_altitude':
                    self._end_max_altitude += 1
                elif reason == 'ekf_drift':
                    self._end_ekf_drift += 1
                elif reason and reason != '':
                    self._end_unknown += 1
        return True

    def _on_rollout_end(self):
        if not wandb.run:
            return
        log_dict = {}

        # SB3 internal metrics
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

        # Per-step means
        if self._step_d_xy:
            log_dict['env/mean_d_xy'] = _mean(self._step_d_xy)
            self._step_d_xy.clear()
        for key, buf in (
            ('env/rew_dist',    self._step_rew_dist),
            ('env/rew_orient',  self._step_rew_orient),
            ('env/rew_vision',  self._step_rew_vision),
            ('env/rew_ctrl',    self._step_rew_ctrl),
            ('env/cos_heading', self._step_cos_heading),
        ):
            if buf:
                log_dict[key] = _mean(buf)
                buf.clear()
        if self._step_total > 0:
            log_dict['env/target_lost_rate'] = (
                self._step_target_lost / self._step_total)
            self._step_target_lost = 0
            self._step_total = 0

        # Per-episode means (over episodes that finished this rollout)
        if self._ep_final_d_xy:
            log_dict['env/ep_final_d_xy'] = _mean(self._ep_final_d_xy)
            self._ep_final_d_xy.clear()
        if self._ep_best_d_xy:
            log_dict['env/ep_best_d_xy'] = _mean(self._ep_best_d_xy)
            self._ep_best_d_xy.clear()
        if self._ep_reward:
            log_dict['env/ep_reward'] = _mean(self._ep_reward)
            self._ep_reward.clear()

        # Cumulative counters (always logged)
        log_dict['env/total_episodes'] = self._total_episodes
        log_dict['env/reached_close'] = self._reached_close
        log_dict['env/end_timeout'] = self._end_timeout
        log_dict['env/end_crash'] = self._end_crash
        log_dict['env/end_stagnation'] = self._end_stagnation
        log_dict['env/end_out_of_range'] = self._end_out_of_range
        log_dict['env/end_ang_vel'] = self._end_ang_vel
        log_dict['env/end_inverted'] = self._end_inverted
        log_dict['env/end_overspeed'] = self._end_overspeed
        log_dict['env/end_max_altitude'] = self._end_max_altitude
        log_dict['env/end_ekf_drift'] = self._end_ekf_drift
        log_dict['env/end_unknown'] = self._end_unknown

        if log_dict:
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
        # Sort by modification time (oldest first) so checkpoints from
        # a previous run with higher step numbers don't outlive the current
        # run's newer-but-lower-numbered files.
        files = sorted(
            _glob.glob(os.path.join(self._dir, f'{self._prefix}_*_steps.zip')),
            key=os.path.getmtime)
        for old in files[:-self._keep]:
            os.remove(old)
            if self.verbose:
                print(f'[Cleanup] Deleted old checkpoint: {old}')
        return True


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Train SAC policy for drone approach-to-X navigation.')
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
    parser.add_argument(
        '--run-name', type=str, default=None,
        help='Override wandb run_name from config')
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
    os.makedirs(checkpoint_dir, exist_ok=True)

    # On a fresh start (no --resume), remove any checkpoint files left over
    # from a previous run so CleanupOldCheckpointsCallback doesn't keep
    # higher-numbered old files and delete the current run's newer ones.
    if cli.resume is None:
        stale = _glob.glob(os.path.join(checkpoint_dir, 'sac_drop_*_steps.zip'))
        for f in stale:
            os.remove(f)
        if stale:
            print(f'[Startup] Removed {len(stale)} stale checkpoint(s) from previous run.')

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
        name=cli.run_name or cfg_wandb.get('run_name') or None,
        tags=cfg_wandb.get('tags', []),
        config=cfg,
        resume='allow',   # re-attaches to existing run after preemption
    )

    # ---- WandB x축 자동 매핑 ----
    # 누적 count metric 의 x축을 env/total_episodes 로 자동 설정.
    wandb.define_metric('env/total_episodes')
    wandb.define_metric('env/end_*',      step_metric='env/total_episodes')
    wandb.define_metric('env/reached_*',  step_metric='env/total_episodes')

    num_envs = cfg_train.get('num_envs', 1)

    print('=== SAC Drone Approach-to-X Training ===')
    print(f'  Config     : {config_path}')
    print(f'  Timesteps  : {total_timesteps:,}')
    print(f'  Checkpoints: {checkpoint_dir}')
    print(f'  Best model : {best_model_dir}')
    print(f'  Archive    : {archive_dir}')
    print(f'  Close thresh: {_REACHED_CLOSE_THRESHOLD_M} m')
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
    callbacks = CallbackList([
        checkpoint_callback, cleanup_callback,
        best_model_callback, milestone_callback,
        wandb_callback, wandb_metrics_callback])

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
