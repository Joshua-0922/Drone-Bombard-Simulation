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
import math as _math
import torch
import torch as th
from torch.nn import functional as F
from stable_baselines3 import SAC
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.common.callbacks import (
    BaseCallback, CallbackList, CheckpointCallback)
from stable_baselines3.common.type_aliases import ReplayBufferSamples
from stable_baselines3.common.utils import polyak_update
from stable_baselines3.common.vec_env import SubprocVecEnv
from wandb.integration.sb3 import WandbCallback


class DampedEntropySAC(SAC):
    """Round 6: SAC with damped entropy auto-tuning + hard cap (Issue #017).

    Round 4/5에서 ent_coef가 6.0+ 발산하여 학습 망가짐.
    원인: SAC auto-tuning이 bounded action space에서 양성 피드백 발산.
    처방:
      1) Soft damping: log_prob이 target보다 멀어질수록 alpha 증가폭 감소
         → policy가 자연스럽게 deterministic 수렴하는 것 허용
      2) Hard cap: 극단 폭주 방지 안전망
    """

    def __init__(self, *args,
                 ent_damping_threshold=5.0,
                 ent_coef_hard_cap=2.0,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.ent_damping_threshold = ent_damping_threshold
        self.ent_coef_hard_cap = ent_coef_hard_cap

    def train(self, gradient_steps, batch_size=64):
        self.policy.set_training_mode(True)
        optimizers = [self.actor.optimizer, self.critic.optimizer]
        if self.ent_coef_optimizer is not None:
            optimizers += [self.ent_coef_optimizer]
        self._update_learning_rate(optimizers)

        ent_coef_losses, ent_coefs = [], []
        actor_losses, critic_losses = [], []
        damping_factors = []  # 모니터링용

        for _ in range(gradient_steps):
            replay_data = self.replay_buffer.sample(
                batch_size, env=self._vec_normalize_env)

            if self.use_sde:
                self.actor.reset_noise()

            actions_pi, log_prob = self.actor.action_log_prob(replay_data.observations)
            log_prob = log_prob.reshape(-1, 1)

            ent_coef_loss = None
            if self.ent_coef_optimizer is not None and self.log_ent_coef is not None:
                ent_coef = th.exp(self.log_ent_coef.detach())

                # Round 6: Soft damping
                # log_prob_mean이 -target_entropy를 초과하면 (정책이 deterministic)
                # alpha-증가 그라디언트를 감쇠
                log_prob_mean = log_prob.mean().item()
                # target_entropy는 보통 -|A| = -5. -target_entropy = 5
                concentration = max(0.0, log_prob_mean - (-self.target_entropy))
                damping = self.ent_damping_threshold / (
                    self.ent_damping_threshold + concentration)
                damping_factors.append(damping)

                ent_coef_loss = -(self.log_ent_coef * (
                    log_prob + self.target_entropy).detach()).mean()
                ent_coef_loss = ent_coef_loss * damping
                ent_coef_losses.append(ent_coef_loss.item())
            else:
                ent_coef = self.ent_coef_tensor

            ent_coefs.append(ent_coef.item())

            if ent_coef_loss is not None and self.ent_coef_optimizer is not None:
                self.ent_coef_optimizer.zero_grad()
                ent_coef_loss.backward()
                self.ent_coef_optimizer.step()
                # Round 6: Hard cap
                with th.no_grad():
                    max_log_alpha = _math.log(self.ent_coef_hard_cap)
                    self.log_ent_coef.clamp_(max=max_log_alpha)

            with th.no_grad():
                next_actions, next_log_prob = self.actor.action_log_prob(
                    replay_data.next_observations)
                next_q_values = th.cat(
                    self.critic_target(replay_data.next_observations, next_actions),
                    dim=1)
                next_q_values, _ = th.min(next_q_values, dim=1, keepdim=True)
                next_q_values = next_q_values - ent_coef * next_log_prob.reshape(-1, 1)
                target_q_values = (
                    replay_data.rewards
                    + (1 - replay_data.dones) * self.gamma * next_q_values)

            current_q_values = self.critic(replay_data.observations, replay_data.actions)
            critic_loss = 0.5 * sum(
                F.mse_loss(current_q, target_q_values) for current_q in current_q_values)
            critic_losses.append(critic_loss.item())

            self.critic.optimizer.zero_grad()
            critic_loss.backward()
            self.critic.optimizer.step()

            q_values_pi = th.cat(
                self.critic(replay_data.observations, actions_pi), dim=1)
            min_qf_pi, _ = th.min(q_values_pi, dim=1, keepdim=True)
            actor_loss = (ent_coef * log_prob - min_qf_pi).mean()
            actor_losses.append(actor_loss.item())

            self.actor.optimizer.zero_grad()
            actor_loss.backward()
            self.actor.optimizer.step()

            if self._n_updates % self.target_update_interval == 0:
                polyak_update(
                    self.critic.parameters(),
                    self.critic_target.parameters(),
                    self.tau)
                polyak_update(self.batch_norm_stats, self.batch_norm_stats_target, 1.0)

            self._n_updates += 1

        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/ent_coef", float(sum(ent_coefs)/len(ent_coefs)))
        self.logger.record("train/actor_loss", float(sum(actor_losses)/len(actor_losses)))
        self.logger.record("train/critic_loss", float(sum(critic_losses)/len(critic_losses)))
        if ent_coef_losses:
            self.logger.record("train/ent_coef_loss",
                               float(sum(ent_coef_losses)/len(ent_coef_losses)))
        if damping_factors:
            self.logger.record("train/ent_damping",
                               float(sum(damping_factors)/len(damping_factors)))


class PrioritizedReplayBuffer(ReplayBuffer):
    """Static priority PER — Round 3 (조합 C, Issue #016).

    Reward 크기 기반 priority 사용. Success transition (큰 reward) 자주 샘플링.
    TD-error 업데이트 없는 단순 버전.
    """

    def __init__(self, *args, alpha=0.6, eps=0.1, priority_max=30.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.alpha = alpha
        self.eps = eps
        self.priority_max = priority_max
        self.priorities = np.zeros(self.buffer_size, dtype=np.float32)
        self.max_priority = 1.0

    def add(self, obs, next_obs, action, reward, done, infos):
        # 삽입 위치 기록 (super().add()가 self.pos 갱신하기 전)
        idx = self.pos
        super().add(obs, next_obs, action, reward, done, infos)
        # Priority = min(priority_max, (|reward| + eps)^alpha)
        # Round 3: priority_max 상한 — outlier (예: 고도 페널티 폭주) 차단
        r = float(np.abs(reward).max())
        priority = min(self.priority_max, (r + self.eps) ** self.alpha)
        self.priorities[idx] = priority
        if priority > self.max_priority:
            self.max_priority = priority

    def sample(self, batch_size, env=None):
        # Priority-weighted sampling
        upper = self.buffer_size if self.full else self.pos
        priorities = self.priorities[:upper]
        probs = priorities / priorities.sum()
        indices = np.random.choice(upper, size=batch_size, p=probs)
        # 마지막 위치(self.pos)는 다음 step에서 덮어쓸 거라 stale 가능 → 회피
        if not self.full:
            indices = np.where(indices == self.pos, (indices + 1) % upper, indices)
        return self._get_samples(indices, env=env)

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
        self._truncate_counts: dict = {
            'crash': 0, 'overspeed': 0, 'ang_vel': 0, 'inverted': 0,
            'timeout': 0, 'out_of_range': 0, 'max_altitude': 0,
        }

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
                reason = info.get('truncate_reason')
                if reason in self._truncate_counts:
                    self._truncate_counts[reason] += 1
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
        for reason, count in self._truncate_counts.items():
            log_dict[f'env/total_truncate_{reason}'] = count

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
        _model = DampedEntropySAC.load(
            cli.resume,
            env=env,
            device=cfg_sac.get('device', 'cuda'),
            tensorboard_log=None,
            custom_objects=dict(
                ent_damping_threshold=cfg_sac.get('ent_damping_threshold', 5.0),
                ent_coef_hard_cap=cfg_sac.get('ent_coef_hard_cap', 2.0),
            ),
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
        # Round 3 (조합 C): PER 사용 여부 결정
        use_per = cfg_sac.get('use_per', False)
        per_alpha = cfg_sac.get('per_alpha', 0.6)
        per_eps = cfg_sac.get('per_eps', 0.1)
        per_priority_max = cfg_sac.get('per_priority_max', 30.0)
        # Round 6: ent_coef damping + hard cap
        ent_damping_threshold = cfg_sac.get('ent_damping_threshold', 5.0)
        ent_coef_hard_cap = cfg_sac.get('ent_coef_hard_cap', 2.0)

        sac_kwargs = dict(
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
            ent_damping_threshold=ent_damping_threshold,
            ent_coef_hard_cap=ent_coef_hard_cap,
        )
        if use_per:
            print(f'[Round 3] PER enabled: alpha={per_alpha}, eps={per_eps}, priority_max={per_priority_max}')
            sac_kwargs['replay_buffer_class'] = PrioritizedReplayBuffer
            sac_kwargs['replay_buffer_kwargs'] = dict(
                alpha=per_alpha, eps=per_eps, priority_max=per_priority_max)
        print(f'[Round 6] DampedEntropySAC: damping_threshold={ent_damping_threshold}, hard_cap={ent_coef_hard_cap}')
        _model = DampedEntropySAC('MlpPolicy', env, **sac_kwargs)

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
