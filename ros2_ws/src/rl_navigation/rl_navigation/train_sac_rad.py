"""RAD v1 — SAC training script for Approach (Phase 1) + Drop (Phase 2).

기존 train_sac.py 와 다른 점:
  • Phase 1 / Phase 2 분기 (yaml training.phase 또는 CLI --phase)
  • Phase 2: Phase 1 모델 load (warm start init only) + Phase 1 rollout wrapper
  • Success rate gate 종료 (step ≥ floor AND 50k window success ≥ 90%)

DampedEntropySAC, PrioritizedReplayBuffer 는 train_sac.py 에서 그대로 import 재사용.

학습 절차:
  Phase 1: ros2 run rl_navigation train_sac_rad --config hyperparams_rad.yaml --phase 1
  Phase 2: ros2 run rl_navigation train_sac_rad --config hyperparams_rad.yaml --phase 2 \
                --phase1-model rl_checkpoints_rad/sac_phase1_final.zip

참조: local/design/rad_v1_design.md (single source of truth)
"""
import argparse
import os
import shutil
import signal
import sys
import time
from collections import deque

import gymnasium as gym
import numpy as np
import wandb
import yaml
from ament_index_python.packages import get_package_share_directory
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from wandb.integration.sb3 import WandbCallback

# RAD: env 및 DampedEntropySAC, PrioritizedReplayBuffer 재사용
from rl_navigation.drone_drop_env_rad import DroneDropEnvRAD
from rl_navigation.train_sac import (
    DampedEntropySAC,
    PrioritizedReplayBuffer,
)


# =============================================================================
# Phase 1 Rollout Wrapper (Phase 2 학습 시 사용)
# =============================================================================
class Phase1RolloutWrapper(gym.Wrapper):
    """RAD: Phase 2 학습 시 매 ep 시작 시 Phase 1 정책으로 sphere 진입까지 rollout.

    동작:
      reset():
        1. base env.reset() — cruise 완료까지 (TRACKING state)
        2. env._cfg_phase = 'phase1' 일시 변경 (sphere 진입 검사 활성)
        3. Phase 1 정책 rollout — sphere 진입 또는 max_rollout_steps 도달
        4. 실패 시 (max_rollout 초과 또는 crash) → wrapper.reset() 재시도
        5. 성공 시 env._cfg_phase = 'phase2' 복원 + reset_phase2_state()
        6. 새 obs 반환 (Phase 2 정책의 init obs)

      step():
        그대로 env.step() — Phase 2 모드 (sphere escape, drop trigger active)

    Phase 1 정책의 rollout step 은 SB3 의 replay buffer 에 들어가지 않음
    (wrapper 가 env.step() 직접 호출, SB3 의 learn loop 와 별도).
    """

    def __init__(self, env, phase1_model, max_rollout_steps=300, max_retries=3):
        super().__init__(env)
        self.phase1_model = phase1_model
        self.max_rollout_steps = max_rollout_steps
        self.max_retries = max_retries
        # 통계
        self.rollout_success_count = 0
        self.rollout_fail_count = 0

    def reset(self, **kwargs):
        for retry in range(self.max_retries):
            obs, info = self.env.reset(**kwargs)
            # Phase 분기 일시 변경
            original_phase = self.env._cfg_phase
            self.env._cfg_phase = 'phase1'
            try:
                # Phase 1 정책 rollout
                rollout_step = 0
                success = False
                for rollout_step in range(self.max_rollout_steps):
                    action, _ = self.phase1_model.predict(obs, deterministic=False)
                    obs, reward, terminated, truncated, ep_info = self.env.step(action)
                    if terminated and ep_info.get('phase1_sphere_entered'):
                        success = True
                        break
                    if terminated or truncated:
                        # Phase 1 rollout 실패 (crash, out_of_range 등)
                        break
            finally:
                self.env._cfg_phase = original_phase

            if success:
                self.rollout_success_count += 1
                # Phase 2 ep state reset
                self.env.reset_phase2_state()
                # 새 obs 받기 (sphere 진입 시점)
                new_obs = self.env._get_obs()
                new_info = {
                    'phase1_rollout_steps': rollout_step + 1,
                    'phase1_rollout_retries': retry,
                }
                return new_obs, new_info

            # Rollout 실패 — 재시도
            self.rollout_fail_count += 1
            if self.env._cfg_phase == 'phase1':
                self.env._cfg_phase = original_phase

        # 3 회 모두 실패 — emergency: 마지막 obs 반환 (SB3 가 ep 시작 시도)
        # 통계 wandb 로 기록 (Phase1RolloutFailureCallback 이 추적)
        return obs, {'phase1_rollout_failed': True}


# =============================================================================
# Success Rate Gate Callback (Phase 1/2 학습 종료 조건)
# =============================================================================
class SuccessRateGateCallback(BaseCallback):
    """RAD: step ≥ floor AND 최신 N window 의 success_rate ≥ threshold 시 학습 종료.

    Phase 1 success: env.info 의 'is_success' = True (sphere 진입 + 모든 C 조건 만족)
    Phase 2 success: env.info 의 'is_success' = True (drop_error ≤ success_threshold)
    """

    def __init__(self, step_floor, window_size, threshold, phase_name='phase1', verbose=1):
        super().__init__(verbose)
        self.step_floor = step_floor
        self.window_size = window_size
        self.threshold = threshold
        self.phase_name = phase_name
        # Episode 결과 history (시간순)
        self._ep_history = deque(maxlen=10_000)  # (step_count, success_bool)
        self._last_step = 0

    def _on_step(self):
        # info 에서 ep 종료 시점의 is_success 추출
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [False])
        for i, done in enumerate(dones):
            if done and i < len(infos):
                info = infos[i]
                # 'is_success' key 있으면 사용 (Phase 1/2 둘 다 일관)
                if 'is_success' in info:
                    self._ep_history.append((self.num_timesteps, bool(info['is_success'])))

        # Step floor 도달 후 window 의 success rate 검사
        if self.num_timesteps < self.step_floor:
            return True

        # window 안의 ep 만 계산
        window_start = self.num_timesteps - self.window_size
        in_window = [s for (step, s) in self._ep_history if step >= window_start]
        if len(in_window) < 10:   # window 안 ep 수 부족 (통계 신뢰도 ↓)
            return True

        success_rate = sum(in_window) / len(in_window)
        if self.num_timesteps - self._last_step >= 1000:
            self._last_step = self.num_timesteps
            self.logger.record(f'{self.phase_name}/success_rate_window', success_rate)
            self.logger.record(f'{self.phase_name}/ep_in_window', len(in_window))

        if success_rate >= self.threshold:
            if self.verbose:
                print(f'\n[SuccessRateGate] {self.phase_name} 종료 조건 도달: '
                      f'step={self.num_timesteps}/{self.step_floor}+, '
                      f'success_rate={success_rate:.3f} ≥ {self.threshold}, '
                      f'window_eps={len(in_window)}\n')
            return False   # 학습 종료
        return True


# =============================================================================
# Terminal Type Monitor (Issue #028 diag)
# =============================================================================
class TerminalTypeMonitorCallback(BaseCallback):
    """Env 의 terminal_type / ep_reward_sum / ep_len rolling window (last N ep) → WandB.

    목적: H1 (task variance → crash rate 지속) vs H3 (reward magnitude → ep_reward 분포) 구별.
    Metric:
      - env/{terminal_type}_rate_100 (crash / entry_success / entry_partial_fail / bad_att_* / trunc_*)
      - env/ep_reward_{mean,min,max}_100
      - env/ep_len_mean_100
    """

    def __init__(self, window=100, log_freq=1000, verbose=0):
        super().__init__(verbose)
        self.window = window
        self.log_freq = log_freq
        self._history = deque(maxlen=window)   # (terminal_type, ep_reward, ep_len)
        self._init_dist_3d_hist = deque(maxlen=window)
        self._init_dist_xy_hist = deque(maxlen=window)
        self._init_pos_x_hist = deque(maxlen=window)
        self._init_pos_y_hist = deque(maxlen=window)
        self._init_pos_z_hist = deque(maxlen=window)
        self._init_speed_hist = deque(maxlen=window)
        self._last_log_step = 0

    def _on_step(self):
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [False])
        for i, done in enumerate(dones):
            if done and i < len(infos):
                info = infos[i]
                tt = info.get('terminal_type')
                if tt is None:
                    continue
                self._history.append((
                    tt,
                    float(info.get('ep_reward_sum', 0.0)),
                    int(info.get('ep_len', 0)),
                ))
                if 'initial_target_dist_3d' in info:
                    self._init_dist_3d_hist.append(float(info['initial_target_dist_3d']))
                    self._init_dist_xy_hist.append(float(info['initial_target_dist_xy']))
                    self._init_pos_x_hist.append(float(info['initial_pos_x']))
                    self._init_pos_y_hist.append(float(info['initial_pos_y']))
                    self._init_pos_z_hist.append(float(info['initial_pos_z']))
                    self._init_speed_hist.append(float(info['initial_speed_xy']))

        if self.num_timesteps - self._last_log_step < self.log_freq:
            return True
        self._last_log_step = self.num_timesteps
        if not self._history:
            return True

        n = len(self._history)
        types = [t for t, _, _ in self._history]
        rewards = [r for _, r, _ in self._history]
        lens = [l for _, _, l in self._history]

        type_counts = {}
        for t in types:
            type_counts[t] = type_counts.get(t, 0) + 1
        for t, c in type_counts.items():
            self.logger.record(f'env/{t}_rate_{self.window}', c / n)

        self.logger.record(f'env/ep_reward_mean_{self.window}',
                           float(sum(rewards) / n))
        self.logger.record(f'env/ep_reward_min_{self.window}', float(min(rewards)))
        self.logger.record(f'env/ep_reward_max_{self.window}', float(max(rewards)))
        self.logger.record(f'env/ep_len_mean_{self.window}',
                           float(sum(lens) / n))
        self.logger.record('env/n_ep_in_window', n)
        # v6 옵션 6: 초기 위치 통계 (정책 시작 시점 fact)
        if self._init_dist_3d_hist:
            def _stats(xs, name):
                lst = list(xs)
                self.logger.record(f'env/{name}_mean', float(sum(lst)/len(lst)))
                self.logger.record(f'env/{name}_min', float(min(lst)))
                self.logger.record(f'env/{name}_max', float(max(lst)))
            _stats(self._init_dist_3d_hist, 'initial_target_dist_3d')
            _stats(self._init_dist_xy_hist, 'initial_target_dist_xy')
            _stats(self._init_pos_x_hist, 'initial_pos_x')
            _stats(self._init_pos_y_hist, 'initial_pos_y')
            _stats(self._init_pos_z_hist, 'initial_pos_z')
            _stats(self._init_speed_hist, 'initial_speed_xy')
        return True


# =============================================================================
# CLI
# =============================================================================
def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--config',
        default=None,
        help='hyperparams_rad.yaml 경로 (default: package share)',
    )
    parser.add_argument(
        '--phase',
        choices=['1', '2'],
        default=None,
        help='Phase override (yaml training.phase 보다 우선)',
    )
    parser.add_argument(
        '--phase1-model',
        default=None,
        help='Phase 2 시 필수. Phase 1 모델 zip 경로',
    )
    parser.add_argument(
        '--total-timesteps',
        type=int,
        default=None,
        help='Override yaml training.total_timesteps',
    )
    parser.add_argument(
        '--instance-id',
        type=int,
        default=0,
        help='Multi-instance training instance id',
    )
    return parser.parse_args()


# =============================================================================
# main
# =============================================================================
def main():
    args = _parse_args()

    # --- Config load ---
    if args.config:
        cfg_path = args.config
    else:
        cfg_path = os.path.join(
            get_package_share_directory('rl_navigation'),
            'config', 'hyperparams_rad.yaml',
        )
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # Phase 분기 (CLI > yaml)
    if args.phase:
        phase = f'phase{args.phase}'
        cfg['training']['phase'] = phase
    else:
        phase = cfg['training']['phase']
    assert phase in ('phase1', 'phase2'), f'Invalid phase: {phase}'

    # Phase 2 시 phase1_model_path 확인
    if phase == 'phase2':
        p1_path = args.phase1_model or cfg['training'].get('phase1_model_path', '')
        if not p1_path or not os.path.exists(p1_path):
            print(f'[ERROR] Phase 2 학습은 phase1_model 필요. 받은 경로: {p1_path}',
                  file=sys.stderr)
            sys.exit(1)
        cfg['training']['phase1_model_path'] = p1_path

    # --- Yaml 을 임시 파일로 저장 (env 가 phase override 받도록) ---
    tmp_cfg_path = '/tmp/hyperparams_rad_runtime.yaml'
    with open(tmp_cfg_path, 'w') as f:
        yaml.safe_dump(cfg, f)

    # --- Env 생성 ---
    print(f'[train_sac_rad] Phase = {phase}, config = {tmp_cfg_path}')
    env = DroneDropEnvRAD(config_path=tmp_cfg_path, instance_id=args.instance_id)

    # --- Phase 1 / Phase 2 모델 setup ---
    sac_cfg = cfg['sac']
    common_sac_kwargs = dict(
        policy='MlpPolicy',
        env=env,
        learning_rate=sac_cfg['learning_rate'],
        buffer_size=sac_cfg['buffer_size'],
        batch_size=sac_cfg['batch_size'],
        tau=sac_cfg['tau'],
        gamma=sac_cfg['gamma'],
        learning_starts=sac_cfg['learning_starts'],
        gradient_steps=sac_cfg['gradient_steps'],
        device=sac_cfg['device'],
        policy_kwargs=dict(net_arch=sac_cfg['net_arch']),
        target_entropy=sac_cfg['target_entropy'],
        verbose=1,
    )

    # PER 적용
    if sac_cfg.get('use_per', False):
        common_sac_kwargs['replay_buffer_class'] = PrioritizedReplayBuffer
        common_sac_kwargs['replay_buffer_kwargs'] = dict(
            alpha=sac_cfg.get('per_alpha', 0.6),
            eps=sac_cfg.get('per_eps', 0.1),
            priority_max=sac_cfg.get('per_priority_max', 30.0),
        )

    # DampedEntropySAC 추가 args
    damped_kwargs = dict(
        ent_damping_threshold=sac_cfg.get('ent_damping_threshold', 5.0),
        ent_coef_hard_cap=sac_cfg.get('ent_coef_hard_cap', 1.0),
        target_q_clip=sac_cfg.get('target_q_clip', 500.0),
    )

    # --- Phase 2: Phase 1 모델 load + warm start init + Phase 1 rollout wrapper ---
    if phase == 'phase2':
        phase1_path = cfg['training']['phase1_model_path']
        print(f'[train_sac_rad] Loading Phase 1 model: {phase1_path}')
        # Phase 1 model 은 deploy 용 (freeze, stochastic predict)
        phase1_model = SAC.load(phase1_path, device='cpu')

        # Wrapper 생성
        max_rollout = cfg['training'].get('phase1_rollout_max_steps', 300)
        env = Phase1RolloutWrapper(env, phase1_model, max_rollout_steps=max_rollout)
        common_sac_kwargs['env'] = env

        # Phase 2 model: Phase 1 weights 로 init (warm start init only)
        model = DampedEntropySAC(**common_sac_kwargs, **damped_kwargs)
        # weights 만 copy (replay buffer fresh)
        model.set_parameters(phase1_path, exact_match=False)
        print('[train_sac_rad] Phase 2 warm start init from Phase 1 weights (buffer fresh)')
    else:
        # Phase 1: fresh model
        model = DampedEntropySAC(**common_sac_kwargs, **damped_kwargs)
        print('[train_sac_rad] Phase 1 fresh model')

    # --- Callbacks ---
    callbacks = []

    # Checkpoint (rolling)
    ckpt_dir = cfg['training']['checkpoint_dir']
    os.makedirs(ckpt_dir, exist_ok=True)
    ckpt_freq = cfg['training']['checkpoint_freq']
    ckpt_name_prefix = f'sac_{phase}'
    callbacks.append(CheckpointCallback(
        save_freq=ckpt_freq,
        save_path=ckpt_dir,
        name_prefix=ckpt_name_prefix,
        save_replay_buffer=False,   # buffer 별도 SIGTERM 시 저장
    ))

    # SuccessRateGate
    if phase == 'phase1':
        step_floor = cfg['training']['phase1_step_floor']
        window_size = cfg['training']['phase1_success_window']
        threshold = cfg['training']['phase1_success_threshold']
    else:
        step_floor = cfg['training']['phase2_step_floor']
        window_size = cfg['training']['phase2_success_window']
        threshold = cfg['training']['phase2_success_threshold']
    callbacks.append(SuccessRateGateCallback(
        step_floor=step_floor,
        window_size=window_size,
        threshold=threshold,
        phase_name=phase,
    ))

    # Terminal type monitor (Issue #028 diag)
    callbacks.append(TerminalTypeMonitorCallback(
        window=100,
        log_freq=1000,
    ))

    # Wandb
    wb_cfg = cfg.get('wandb', {})
    if wb_cfg.get('project'):
        run = wandb.init(
            project=wb_cfg['project'],
            entity=wb_cfg.get('entity'),
            name=wb_cfg.get('run_name', f'rad_{phase}_v1'),
            tags=wb_cfg.get('tags', ['rad', phase, 'v1']),
            config=cfg,
            sync_tensorboard=True,
            save_code=True,
        )
        callbacks.append(WandbCallback(
            model_save_path=os.path.join(ckpt_dir, 'wandb_models'),
            verbose=2,
        ))
        print(f'[train_sac_rad] wandb run = {run.name} ({run.id})')

    # --- Emergency save (SIGTERM 만 — Ctrl+C 는 무시) ---
    def _save_on_sigterm(signum, frame):
        print(f'\n[SIGTERM] preempt save 시작...')
        preempt_path = os.path.join(ckpt_dir, f'sac_{phase}_preempt.zip')
        model.save(preempt_path)
        replay_path = os.path.join(ckpt_dir, f'sac_{phase}_preempt_replay.pkl')
        model.save_replay_buffer(replay_path)
        print(f'  → {preempt_path}\n  → {replay_path}')
        if wb_cfg.get('project') and 'run' in locals():
            try:
                wandb.save(preempt_path)
                wandb.save(replay_path)
                run.finish()
            except Exception:
                pass
        sys.exit(0)
    signal.signal(signal.SIGTERM, _save_on_sigterm)

    # --- Total timesteps ---
    total_ts = args.total_timesteps or cfg['training']['total_timesteps']

    # --- Train ---
    print(f'[train_sac_rad] Starting {phase} training: total_timesteps={total_ts}')
    print(f'  step_floor={step_floor}, window={window_size}, threshold={threshold}')
    try:
        model.learn(
            total_timesteps=total_ts,
            callback=CallbackList(callbacks),
            log_interval=10,
        )
    except KeyboardInterrupt:
        print('\n[KeyboardInterrupt] WARNING: 모델/buffer 저장 안 됨. SIGTERM 권장.')
    finally:
        # 최종 모델 저장 (정상 종료 시)
        final_path = os.path.join(ckpt_dir, f'sac_{phase}_final.zip')
        model.save(final_path)
        print(f'[train_sac_rad] Final model: {final_path}')
        if wb_cfg.get('project'):
            try:
                wandb.save(final_path)
                wandb.finish()
            except Exception:
                pass

    env.close()


if __name__ == '__main__':
    main()
