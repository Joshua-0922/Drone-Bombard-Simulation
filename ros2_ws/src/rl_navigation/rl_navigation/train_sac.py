"""SAC training script for drone fly-by drop policy."""

import argparse
import glob as _glob
import json
import os
import shutil
import signal
import subprocess
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
                 target_q_clip=500.0,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.ent_damping_threshold = ent_damping_threshold
        self.ent_coef_hard_cap = ent_coef_hard_cap
        # Round 7 v2 (#021 b): critic stability via target Q clipping.
        # Prevents bootstrap inflation from runaway feedback loop.
        self.target_q_clip = target_q_clip

    def train(self, gradient_steps, batch_size=64):
        self.policy.set_training_mode(True)
        optimizers = [self.actor.optimizer, self.critic.optimizer]
        if self.ent_coef_optimizer is not None:
            optimizers += [self.ent_coef_optimizer]
        self._update_learning_rate(optimizers)

        ent_coef_losses, ent_coefs = [], []
        actor_losses, critic_losses = [], []
        damping_factors = []  # 모니터링용
        # === Issue #028 diag: Q/actor loss/log_prob/reward batch stats ===
        q_target_means, q_target_maxs, q_target_mins, q_target_stds = [], [], [], []
        q_current_means, q_current_maxs, q_current_mins = [], [], []
        q_pi_means, q_pi_maxs, q_pi_mins = [], [], []
        q_clip_ratios = []
        actor_loss_q_terms, actor_loss_ent_terms = [], []
        log_prob_means, log_prob_stds = [], []
        reward_means, reward_maxs, reward_mins = [], [], []
        # === Issue #028 self-healing: A1 gradient clip + B3 weight NaN rollback ===
        nan_rollback_count = 0
        actor_grad_clip_hit = 0
        critic_grad_clip_hit = 0
        _ACTOR_GRAD_CLIP = 20.0
        _CRITIC_GRAD_CLIP = 200.0

        for _ in range(gradient_steps):
            replay_data = self.replay_buffer.sample(
                batch_size, env=self._vec_normalize_env)

            with th.no_grad():
                _r = replay_data.rewards
                reward_means.append(_r.mean().item())
                reward_maxs.append(_r.max().item())
                reward_mins.append(_r.min().item())

            # === B3: snapshot before gradient step (rollback 목적) ===
            _actor_snapshot = {k: v.detach().clone()
                               for k, v in self.actor.state_dict().items()}
            _critic_snapshot = {k: v.detach().clone()
                                for k, v in self.critic.state_dict().items()}

            if self.use_sde:
                self.actor.reset_noise()

            actions_pi, log_prob = self.actor.action_log_prob(replay_data.observations)
            log_prob = log_prob.reshape(-1, 1)

            ent_coef_loss = None
            if self.ent_coef_optimizer is not None and self.log_ent_coef is not None:
                ent_coef = th.exp(self.log_ent_coef.detach())

                # Round 7 v3 (#021 a): Per-sample damping (replaces q95-based).
                # 기존 q95 는 batch 전체에 단일 scalar damping → outlier 가
                # 평균만 끌어올려도 모든 sample 의 contribution 이 동일하게 damped.
                # 새 방식: 각 transition 의 log_prob 으로 concentration 계산 →
                # element-wise damping → 정상 transition 은 그대로, outlier 만 강하게 damped.
                concentration_per_sample = th.clamp(
                    log_prob.detach() - (-self.target_entropy), min=0.0)
                damping_per_sample = self.ent_damping_threshold / (
                    self.ent_damping_threshold + concentration_per_sample)
                # mean 으로 monitoring 만 보고 — 실제 적용은 element-wise.
                damping_factors.append(damping_per_sample.mean().item())

                ent_coef_loss = -(
                    self.log_ent_coef
                    * (log_prob + self.target_entropy).detach()
                    * damping_per_sample
                ).mean()
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
                # Q_target 통계
                q_target_means.append(target_q_values.mean().item())
                q_target_maxs.append(target_q_values.max().item())
                q_target_mins.append(target_q_values.min().item())
                q_target_stds.append(target_q_values.std().item())
                q_clip_ratios.append(
                    (target_q_values.abs() > self.target_q_clip).float().mean().item())

            current_q_values = self.critic(replay_data.observations, replay_data.actions)
            with th.no_grad():
                _cq_all = th.cat([cq.detach() for cq in current_q_values], dim=1)
                q_current_means.append(_cq_all.mean().item())
                q_current_maxs.append(_cq_all.max().item())
                q_current_mins.append(_cq_all.min().item())
            # Round 7 v3 (#021 b): critic stability — Huber loss + target Q clipping.
            # MSE: 큰 TD-error 에 대해 gradient 가 비례 증가 → 폭발 feedback 가능.
            # Huber: error > 1 에서 gradient 1.0 으로 saturate → 안전한 step size.
            # Target clip: bootstrap Q 폭주 차단 (reward scale 고려 ±target_q_clip).
            target_q_clipped = target_q_values.clamp(
                -self.target_q_clip, self.target_q_clip)
            critic_loss = 0.5 * sum(
                F.smooth_l1_loss(current_q, target_q_clipped)
                for current_q in current_q_values)
            critic_losses.append(critic_loss.item())

            self.critic.optimizer.zero_grad()
            critic_loss.backward()
            # A1: gradient clip (극단 spike 방지)
            _critic_norm = th.nn.utils.clip_grad_norm_(
                self.critic.parameters(), max_norm=_CRITIC_GRAD_CLIP)
            if _critic_norm.item() > _CRITIC_GRAD_CLIP:
                critic_grad_clip_hit += 1
            self.critic.optimizer.step()

            q_values_pi = th.cat(
                self.critic(replay_data.observations, actions_pi), dim=1)
            min_qf_pi, _ = th.min(q_values_pi, dim=1, keepdim=True)
            actor_loss = (ent_coef * log_prob - min_qf_pi).mean()
            actor_losses.append(actor_loss.item())
            with th.no_grad():
                q_pi_means.append(min_qf_pi.mean().item())
                q_pi_maxs.append(min_qf_pi.max().item())
                q_pi_mins.append(min_qf_pi.min().item())
                actor_loss_q_terms.append((-min_qf_pi).mean().item())
                actor_loss_ent_terms.append((ent_coef * log_prob).mean().item())
                log_prob_means.append(log_prob.mean().item())
                log_prob_stds.append(log_prob.std().item())

            self.actor.optimizer.zero_grad()
            actor_loss.backward()
            # A1: gradient clip (극단 spike 방지)
            _actor_norm = th.nn.utils.clip_grad_norm_(
                self.actor.parameters(), max_norm=_ACTOR_GRAD_CLIP)
            if _actor_norm.item() > _ACTOR_GRAD_CLIP:
                actor_grad_clip_hit += 1
            self.actor.optimizer.step()

            # === B3: weight NaN check → rollback ===
            _weight_nan = False
            for _p in self.actor.parameters():
                if th.isnan(_p).any() or th.isinf(_p).any():
                    _weight_nan = True
                    break
            if not _weight_nan:
                for _p in self.critic.parameters():
                    if th.isnan(_p).any() or th.isinf(_p).any():
                        _weight_nan = True
                        break
            if _weight_nan:
                self.actor.load_state_dict(_actor_snapshot)
                self.critic.load_state_dict(_critic_snapshot)
                nan_rollback_count += 1
                # gradient state clear (다음 iteration 위해)
                self.actor.optimizer.zero_grad()
                self.critic.optimizer.zero_grad()

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

        # === Issue #028 diag: Q / actor_loss decomposition / log_prob / reward / norms ===
        def _avg(xs):
            return float(sum(xs) / len(xs)) if xs else 0.0
        self.logger.record("train/q_target_mean", _avg(q_target_means))
        self.logger.record("train/q_target_max", _avg(q_target_maxs))
        self.logger.record("train/q_target_min", _avg(q_target_mins))
        self.logger.record("train/q_target_std", _avg(q_target_stds))
        self.logger.record("train/q_current_mean", _avg(q_current_means))
        self.logger.record("train/q_current_max", _avg(q_current_maxs))
        self.logger.record("train/q_current_min", _avg(q_current_mins))
        self.logger.record("train/q_pi_mean", _avg(q_pi_means))
        self.logger.record("train/q_pi_max", _avg(q_pi_maxs))
        self.logger.record("train/q_pi_min", _avg(q_pi_mins))
        self.logger.record("train/q_clip_ratio", _avg(q_clip_ratios))
        self.logger.record("train/actor_loss_q_term", _avg(actor_loss_q_terms))
        self.logger.record("train/actor_loss_ent_term", _avg(actor_loss_ent_terms))
        self.logger.record("train/log_prob_mean", _avg(log_prob_means))
        self.logger.record("train/log_prob_std", _avg(log_prob_stds))
        self.logger.record("train/reward_batch_mean", _avg(reward_means))
        self.logger.record("train/reward_batch_max", _avg(reward_maxs))
        self.logger.record("train/reward_batch_min", _avg(reward_mins))
        # Weight/gradient norms — gradient_steps=1 이라 마지막 iteration 값
        with th.no_grad():
            actor_wnorm = 0.0
            for p in self.actor.parameters():
                actor_wnorm += (p.detach() ** 2).sum().item()
            actor_wnorm = actor_wnorm ** 0.5
            critic_wnorm = 0.0
            for p in self.critic.parameters():
                critic_wnorm += (p.detach() ** 2).sum().item()
            critic_wnorm = critic_wnorm ** 0.5
            actor_gnorm = 0.0
            for p in self.actor.parameters():
                if p.grad is not None:
                    actor_gnorm += (p.grad ** 2).sum().item()
            actor_gnorm = actor_gnorm ** 0.5
            critic_gnorm = 0.0
            for p in self.critic.parameters():
                if p.grad is not None:
                    critic_gnorm += (p.grad ** 2).sum().item()
            critic_gnorm = critic_gnorm ** 0.5
        self.logger.record("train/actor_weight_norm", actor_wnorm)
        self.logger.record("train/critic_weight_norm", critic_wnorm)
        self.logger.record("train/actor_grad_norm", actor_gnorm)
        self.logger.record("train/critic_grad_norm", critic_gnorm)
        # Self-healing metrics
        self.logger.record("train/nan_rollback_count", nan_rollback_count)
        self.logger.record("train/actor_grad_clip_hit", actor_grad_clip_hit)
        self.logger.record("train/critic_grad_clip_hit", critic_grad_clip_hit)


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
        # Phase 1 redux v3: d_xy outlier 추적 (1 if > 6m else 0)
        self._step_d_xy_outlier: list = []
        self._step_rew_drop: list = []
        self._total_episodes: int = 0
        self._total_drops: int = 0
        self._total_auto_drops: int = 0
        self._total_success: int = 0
        self._total_jackpot: int = 0
        # Phase 1 redux v3: total_truncate_* wandb 제거 → 카운트 자체 안 함
        # Phase 1 redux v2: 현재 시점의 연속 성공 횟수.
        # success 발생 시 +1, 그 외 (실패 drop, drop 없는 episode 끝) 시 0 리셋.
        self._current_success_streak: int = 0

    def _on_step(self):
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [])
        for info, done in zip(infos, dones):
            if 'd_xy' in info:
                d_xy = info['d_xy']
                self._step_d_xy.append(d_xy)
                # Phase 1 redux v3: outlier 표시 (0~6m: 0, 6m+: 1)
                self._step_d_xy_outlier.append(1 if d_xy > 6.0 else 0)
            if 'rew_drop' in info:
                self._step_rew_drop.append(info['rew_drop'])

            if done:
                self._total_episodes += 1
                # Phase 1 redux v3: total_truncate_* wandb 제거 (truncate_counts 추적 안 함)
                if 'drop_error_actual_m' in info:
                    self._ep_drop_errors.append(info['drop_error_actual_m'])
                    self._total_drops += 1
                    if info.get('drop_trigger') != 'random':
                        self._total_auto_drops += 1
                    if info.get('is_success'):
                        self._total_success += 1
                        self._current_success_streak += 1
                    else:
                        # drop 했지만 success 아님 → 실패로 보고 streak 리셋
                        self._current_success_streak = 0
                    if info.get('jackpot'):
                        self._total_jackpot += 1
                else:
                    # drop 자체가 없는 episode 끝 (truncate w/o drop) → streak 리셋
                    self._current_success_streak = 0
        return True

    def _on_rollout_end(self):
        if not wandb.run:
            return
        log_dict = {}
        # Phase 1 redux v3: rollout/success_rate 도 fetch
        for key in ('rollout/ep_rew_mean', 'rollout/ep_len_mean', 'rollout/success_rate',
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
        # Phase 1 redux v3: d_xy outlier ratio (0~6m: 0, 6m+: 1 의 평균)
        if self._step_d_xy_outlier:
            log_dict['env/d_xy_outlier_ratio'] = _mean(self._step_d_xy_outlier)
            self._step_d_xy_outlier.clear()
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
        # Phase 1 redux v2: 현재 연속 성공 횟수 (rollout end 시점 snapshot)
        log_dict['env/current_success_streak'] = self._current_success_streak
        # Phase 1 redux v3: success_rate 도 env/ namespace 에 mirror (찾기 편하게)
        sr = self.logger.name_to_value.get('rollout/success_rate')
        if sr is not None:
            log_dict['env/success_rate'] = sr
        # Phase 1 redux v3: env/total_truncate_* 모두 제거 (사용자 요청)

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
        # Round 6 bug fix: step 번호로 정렬 (이전엔 알파벳순 → 100k+가 75k보다
        # 먼저 정렬되어 새 체크포인트가 즉시 삭제되는 버그)
        files = _glob.glob(
            os.path.join(self._dir, f'{self._prefix}_*_steps.zip'))
        def _step_num(p):
            name = os.path.basename(p)
            try:
                return int(name.replace(f'{self._prefix}_', '').replace('_steps.zip', ''))
            except ValueError:
                return -1
        files = sorted(files, key=_step_num)
        for old in files[:-self._keep]:
            os.remove(old)
            if self.verbose:
                print(f'[Cleanup] Deleted old checkpoint: {old}')
        return True


class InfraHealthMonitorCallback(BaseCallback):
    """Round 7 diagnostic — track Gazebo/PX4 health + reset velocity over time.

    Hypothesis: gz set_pose teleport preserves velocity → residual velocity
    accumulates across non-drop episodes → eventually destabilises Gazebo
    physics → gz model --list hangs → training crashes.

    Measures every check_freq steps:
      - gz model --list response time (ms) — if grows over time, registry/lock issue
      - gz sim RSS (MB)                    — if grows, Gazebo memory leak
      - bin/px4 RSS (MB)                   — if grows, PX4 internal state leak
      - Latest reset velocity diag from env — pre/post velocity comparison
    """

    def __init__(self, env, check_freq=200, verbose=0):
        super().__init__(verbose)
        self._env = env
        self._check_freq = check_freq

    @staticmethod
    def _rss_mb(pattern):
        """Return RSS in MB for first process matching pattern via pgrep."""
        try:
            pids = subprocess.run(
                ['pgrep', '-f', pattern], capture_output=True, text=True,
                timeout=2.0).stdout.strip().split()
            if not pids:
                return -1.0
            with open(f'/proc/{pids[0]}/status') as f:
                for line in f:
                    if line.startswith('VmRSS:'):
                        return int(line.split()[1]) / 1024.0  # KB → MB
        except (subprocess.TimeoutExpired, FileNotFoundError, ValueError, OSError):
            return -1.0
        return -1.0

    def _on_step(self):
        if self.num_timesteps % self._check_freq != 0:
            return True

        # gz model --list response time
        _t0 = time.time()
        try:
            subprocess.run(
                ['gz', 'model', '--list'],
                capture_output=True, timeout=5.0)
            gz_ms = (time.time() - _t0) * 1000.0
            gz_ok = 1
        except subprocess.TimeoutExpired:
            gz_ms = 5000.0
            gz_ok = 0

        gz_rss = self._rss_mb('gz sim')
        px4_rss = self._rss_mb('bin/px4')

        # Env reset diag (DummyVecEnv: env.envs[0])
        diag = {}
        try:
            inner = self._env.envs[0] if hasattr(self._env, 'envs') else self._env
            inner = getattr(inner, 'env', inner)  # Monitor wrapper unwrap
            diag = getattr(inner, '_reset_diag', {}) or {}
        except (AttributeError, IndexError):
            pass

        log_dict = {
            'infra/gz_list_ms': gz_ms,
            'infra/gz_list_ok': gz_ok,
            'infra/gz_rss_mb': gz_rss,
            'infra/px4_rss_mb': px4_rss,
        }
        if diag:
            log_dict.update({
                'infra/reset_pre_v': diag.get('pre_reset_v', 0.0),
                'infra/reset_pre_ang_v': diag.get('pre_reset_ang_v', 0.0),
                'infra/reset_post_cruise_v': diag.get('post_cruise_v', 0.0),
                'infra/reset_post_cruise_ang_v': diag.get('post_cruise_ang_v', 0.0),
                'infra/reset_idx': diag.get('reset_idx', 0),
                'infra/reset_prev_dropped': int(diag.get('prev_dropped', False)),
                # 2차/1차 처방 진단 — postmortem용
                'infra/used_full_restart': int(diag.get('used_full_restart', False)),
                'infra/forced_restart_triggered': int(diag.get('forced_restart_triggered', False)),
                'infra/consecutive_fast_resets': diag.get('consecutive_fast_resets', 0),
                'infra/cruise_timeout_attempts': diag.get('cruise_timeout_attempts', 0),
            })

        if wandb.run:
            wandb.log(log_dict, step=self.num_timesteps)

        if self.verbose:
            print(f'[InfraHealth] step={self.num_timesteps} '
                  f'gz_ms={gz_ms:.0f} gz_rss={gz_rss:.0f}MB '
                  f'px4_rss={px4_rss:.0f}MB '
                  f'pre_v={diag.get("pre_reset_v", 0):.2f} '
                  f'post_v={diag.get("post_cruise_v", 0):.2f}')

        return True


class DropEpisodeRecorderCallback(BaseCallback):
    """Save full trajectory data for episodes where a drop was triggered.

    Each drop episode is stored as a compressed .npz containing the action
    sequence (for replay), observations, rewards, and per-step metrics.
    An index.csv is maintained for quick scanning.
    """

    def __init__(self, save_dir, success_replay_dir=None, run_id=None, verbose=0):
        super().__init__(verbose)
        self._save_dir = save_dir
        # Round 6+: success+auto_drop 모델 저장 — run_id별 폴더 (host에 영구 보관)
        # 이전 best_drops 시스템 대체 (best 조건 제거, 모든 성공 저장)
        if success_replay_dir and run_id:
            self._success_replay_dir = os.path.join(success_replay_dir, run_id)
            os.makedirs(self._success_replay_dir, exist_ok=True)
        else:
            self._success_replay_dir = None
        os.makedirs(save_dir, exist_ok=True)
        self._buffers = {}
        self._drop_count = 0
        self._best_drop_error = float('inf')
        self._index_path = os.path.join(save_dir, 'index.csv')
        if not os.path.exists(self._index_path):
            with open(self._index_path, 'w') as f:
                # Phase 1 redux v2: drop_trigger 컬럼 추가 (auto/random/manual 구분)
                # representative_best_analysis 에서 auto drop 만 필터 위해 필수.
                f.write('filename,timestep,drop_error_m,is_success,'
                        'episode_reward,n_steps,drop_trigger\n')

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

                    drop_trigger = info.get('drop_trigger', 'unknown')
                    with open(self._index_path, 'a') as f:
                        f.write(f'{fname},{self.num_timesteps},'
                                f'{drop_error:.4f},{is_success},'
                                f'{ep_reward:.2f},{n_steps},{drop_trigger}\n')

                    # Round 6+: success + auto_drop 모델을 success_replay로 저장.
                    # 이전 best_drops 시스템 대체 (run_id별 영구 보관, host symlink 접근).
                    if (is_success
                            and info.get('drop_trigger') == 'auto'
                            and self._success_replay_dir):
                        if drop_error < self._best_drop_error:
                            self._best_drop_error = drop_error
                        save_path = os.path.join(
                            self._success_replay_dir,
                            f'success_step{self.num_timesteps}_err{drop_error:.2f}m')
                        self.model.save(save_path)
                        if self.verbose:
                            print(f'[SuccessReplay] SUCCESS (auto)! {drop_error:.3f}m '
                                  f'→ model saved to {save_path}.zip')

                    if self.verbose:
                        print(f'[DropRecorder] Saved {fname} '
                              f'(err={drop_error:.3f}m, rew={ep_reward:.1f},'
                              f' {n_steps} steps)')

                self._buffers[i] = {
                    'obs': [], 'act': [], 'rew': [],
                    'd_xy': [], 'd_impact': [],
                }
        return True


class RepresentativeBestCallback(BaseCallback):
    """Phase 1 redux v2: 학습 종료 시 representative_top_3 자동 산출.

    정의:
        peak_episode = LAST episode E where rolling_100_success_rate(E) == max
        peak_window  = episodes [E-99, E]
        representative_top_3 = sorted(auto+success drops in window by drop_error)[:3]

    규칙:
        - "auto drop 만" 카운트 (random/manual 제외)
        - drop < 3 시 → "not measurable" (skip)
        - tie 시 LAST peak

    저장:
        매 rollout end 시 manifest 갱신 → SIGTERM/SIGINT 발생해도 마지막 상태 보존.
        REPRESENTATIVE_BEST.json → success_replay/{run_id}/
    """

    def __init__(self, success_replay_dir=None, run_id=None, verbose=0):
        super().__init__(verbose)
        if success_replay_dir and run_id:
            self._manifest_dir = os.path.join(success_replay_dir, run_id)
            self._manifest_path = os.path.join(self._manifest_dir, 'REPRESENTATIVE_BEST.json')
        else:
            self._manifest_path = None
        self._run_id = run_id
        # Per-episode: 1 if is_success else 0
        self._is_success_per_ep: list = []
        # 모든 auto+success drops 누적
        self._auto_success_drops: list = []
        # Per-rollout snapshot
        self._rolling_history: list = []

    def _on_step(self):
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [])
        for info, done in zip(infos, dones):
            if not done:
                continue
            # episode 가 drop 으로 끝났을 때만 is_success 의미 있음
            if 'drop_error_actual_m' in info:
                is_success = bool(info.get('is_success', False))
                self._is_success_per_ep.append(1 if is_success else 0)
                if is_success and info.get('drop_trigger') == 'auto':
                    self._auto_success_drops.append({
                        'episode': len(self._is_success_per_ep),
                        'step': int(self.num_timesteps),
                        'drop_error_m': float(info['drop_error_actual_m']),
                    })
            else:
                # drop 없이 episode 끝남 → 실패로 카운트
                self._is_success_per_ep.append(0)
        return True

    def _on_rollout_end(self):
        # 현재 시점의 rolling-100 success_rate snapshot
        if not self._is_success_per_ep:
            return
        window = self._is_success_per_ep[-100:]
        sr = sum(window) / len(window)
        self._rolling_history.append({
            'episode': len(self._is_success_per_ep),
            'step': int(self.num_timesteps),
            'rolling_100_sr': sr,
        })
        # Manifest 매 rollout 갱신 → SIGTERM 받아도 최근 상태 보존
        self._save_manifest()

    def _save_manifest(self):
        if not self._manifest_path:
            return
        if not self._rolling_history:
            return

        # 마지막 peak 찾기
        max_sr = max(r['rolling_100_sr'] for r in self._rolling_history)
        peaks = [r for r in self._rolling_history if r['rolling_100_sr'] == max_sr]
        peak = peaks[-1]
        peak_ep = peak['episode']
        peak_step = peak['step']
        win_ep_start = peak_ep - 100
        win_ep_end = peak_ep

        # Peak window step 범위 추정 (rolling_history 중 window 시작 직전 rollout)
        win_step_start = None
        for r in self._rolling_history:
            if r['episode'] >= win_ep_start:
                win_step_start = r['step']
                break
        if win_step_start is None:
            win_step_start = self._rolling_history[0]['step']
        win_step_end = peak_step

        # 윈도우 내 auto+success drops
        drops_in_window = [
            d for d in self._auto_success_drops
            if win_ep_start <= d['episode'] <= win_ep_end
        ]
        sorted_drops = sorted(drops_in_window, key=lambda d: d['drop_error_m'])

        # 결정 C: drop < 3 면 not measurable
        if len(sorted_drops) < 3:
            manifest = {
                'run_id': self._run_id,
                'status': 'not_measurable',
                'reason': (f'Auto+success drops in peak window = {len(sorted_drops)} (< 3). '
                           f'Per decision C: skip representative_best measurement.'),
                'peak_success_rate': max_sr,
                'peak_episode': peak_ep,
                'peak_step': peak_step,
                'peak_window_episodes': [win_ep_start, win_ep_end],
                'peak_window_steps': [win_step_start, win_step_end],
                'auto_success_drops_in_window': len(sorted_drops),
                'drops_found_in_window': [
                    {'step': d['step'], 'drop_error_m': d['drop_error_m'],
                     'episode': d['episode']}
                    for d in sorted_drops
                ],
                'total_auto_success_drops': len(self._auto_success_drops),
                'total_episodes': len(self._is_success_per_ep),
                'updated_at_step': int(self.num_timesteps),
            }
        else:
            top_3 = sorted_drops[:3]
            manifest = {
                'run_id': self._run_id,
                'status': 'measured',
                'peak_success_rate': max_sr,
                'peak_episode': peak_ep,
                'peak_step': peak_step,
                'peak_window_episodes': [win_ep_start, win_ep_end],
                'peak_window_steps': [win_step_start, win_step_end],
                'auto_success_drops_in_window': len(sorted_drops),
                'representative_top_3': [
                    {
                        'rank': i + 1,
                        'drop_error_m': d['drop_error_m'],
                        'step': d['step'],
                        'episode': d['episode'],
                        'model_filename': f"success_step{d['step']}_err{d['drop_error_m']:.2f}m",
                    }
                    for i, d in enumerate(top_3)
                ],
                'total_auto_success_drops': len(self._auto_success_drops),
                'total_episodes': len(self._is_success_per_ep),
                'updated_at_step': int(self.num_timesteps),
            }

        os.makedirs(self._manifest_dir, exist_ok=True)
        with open(self._manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2, ensure_ascii=False)


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
    # Round 6+: success_replay 경로 (host에 bind-mounted ros2_ws 통해 접근)
    # 컨테이너: /workspace/ros2_ws/success_replay/{run_id}/
    # Host:    /home/juns/.../ros2_ws/success_replay/{run_id}/
    # local:   /home/juns/.../local/success_replay → ../ros2_ws/success_replay (symlink)
    _success_replay_dir = '/workspace/ros2_ws/success_replay'
    _success_run_id = wandb.run.id if wandb.run else 'no_wandb'
    drop_recorder_callback = DropEpisodeRecorderCallback(
        save_dir=drop_episodes_dir,
        success_replay_dir=_success_replay_dir,
        run_id=_success_run_id,
        verbose=1)
    # Round 7 diagnostic: leak investigation
    infra_health_callback = InfraHealthMonitorCallback(
        env=env, check_freq=200, verbose=1)
    # Phase 1 redux v2: representative_top_3 자동 산출
    representative_callback = RepresentativeBestCallback(
        success_replay_dir=_success_replay_dir,
        run_id=_success_run_id,
        verbose=1)
    callbacks = CallbackList([
        checkpoint_callback, cleanup_callback,
        best_model_callback, milestone_callback,
        wandb_callback, wandb_metrics_callback,
        drop_recorder_callback, infra_health_callback,
        representative_callback])

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
                target_q_clip=cfg_sac.get('target_q_clip', 500.0),
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
        # Round 7: target_entropy 명시 (default -|A| = -5는 bounded action에 부적합)
        target_entropy = cfg_sac.get('target_entropy', 'auto')
        # Round 7 v3 (#021): critic stability via target Q clipping.
        target_q_clip = cfg_sac.get('target_q_clip', 500.0)

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
            target_entropy=target_entropy,
            target_q_clip=target_q_clip,
        )
        if use_per:
            print(f'[Round 3] PER enabled: alpha={per_alpha}, eps={per_eps}, priority_max={per_priority_max}')
            sac_kwargs['replay_buffer_class'] = PrioritizedReplayBuffer
            sac_kwargs['replay_buffer_kwargs'] = dict(
                alpha=per_alpha, eps=per_eps, priority_max=per_priority_max)
        print(f'[Round 6] DampedEntropySAC: damping_threshold={ent_damping_threshold}, hard_cap={ent_coef_hard_cap}')
        print(f'[Round 7] target_entropy={target_entropy} (default auto = -|A| = -5)')
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
