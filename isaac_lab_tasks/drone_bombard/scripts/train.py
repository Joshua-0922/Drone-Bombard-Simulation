#!/usr/bin/env python3
"""드론 폭격 RL 학습 진입점 (SB3 SAC + Isaac Lab).

사용법:
  python train.py --num_envs 4096 --wandb --total_timesteps 5000000

Isaac Lab 의 Sb3VecEnvWrapper 로 SB3 SAC 를 연결한다.
SIGTERM 핸들러와 롤링 체크포인트로 학습 중단 시 모델을 보존한다.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys

# Isaac Sim 초기화는 다른 임포트보다 반드시 먼저 수행해야 한다.
from isaaclab.app import AppLauncher

# ---------------------------------------------------------------------------
# CLI 인자 파싱
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="Drone Bombard SAC 학습")
parser.add_argument("--num_envs", type=int, default=4096, help="병렬 환경 수")
parser.add_argument("--total_timesteps", type=int, default=5_000_000, help="총 학습 스텝")
parser.add_argument("--checkpoint", type=str, default=None, help="이어서 학습할 체크포인트 경로")
parser.add_argument("--wandb", action="store_true", help="WandB 로깅 활성화")
parser.add_argument("--wandb_project", type=str, default="drone-bombard-isaac")
parser.add_argument("--log_dir", type=str, default="logs/sac_drone_drop")
parser.add_argument("--headless", action="store_true", default=True, help="GUI 없이 실행")
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

# Isaac Sim 앱 실행
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# ---------------------------------------------------------------------------
# 이후 임포트 (Isaac Sim 초기화 후)
# ---------------------------------------------------------------------------

import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    CallbackList,
    CheckpointCallback,
    EvalCallback,
)

from isaaclab_rl.sb3 import Sb3VecEnvWrapper, process_sb3_cfg  # type: ignore

from drone_bombard.config.agents.sac_cfg import SacAgentCfg
from drone_bombard.config.drone_drop_env_cfg import DroneBombardEnvCfg
from drone_bombard.env import DroneBombardEnv

# ---------------------------------------------------------------------------
# WandB 콜백
# ---------------------------------------------------------------------------

def make_wandb_callback(project: str, cfg: SacAgentCfg):
    """WandB 콜백 생성. wandb 패키지 없으면 None 반환."""
    try:
        import wandb
        from stable_baselines3.common.callbacks import BaseCallback

        class WandbMetricsCallback(BaseCallback):
            def __init__(self):
                super().__init__()
                wandb.init(project=project, config=vars(cfg))

            def _on_step(self) -> bool:
                if self.n_calls % cfg.log_interval == 0:
                    infos = self.locals.get("infos", [])
                    for info in infos:
                        for k, v in info.items():
                            if isinstance(v, (int, float)):
                                wandb.log({f"env/{k}": v}, step=self.num_timesteps)
                return True

            def _on_training_end(self):
                wandb.finish()

        return WandbMetricsCallback()
    except ImportError:
        print("[train] wandb 패키지 없음 — WandB 콜백 비활성화")
        return None


# ---------------------------------------------------------------------------
# SIGTERM 핸들러
# ---------------------------------------------------------------------------

_model_ref = None

def _sigterm_handler(signum, frame):
    """SIGTERM 수신 시 현재 모델 저장 후 종료."""
    if _model_ref is not None:
        save_path = os.path.join(args.log_dir, "model_sigterm")
        _model_ref.save(save_path)
        print(f"[train] SIGTERM — 모델 저장됨: {save_path}")
    sys.exit(0)

signal.signal(signal.SIGTERM, _sigterm_handler)


# ---------------------------------------------------------------------------
# 메인 학습 루프
# ---------------------------------------------------------------------------

def main():
    global _model_ref

    os.makedirs(args.log_dir, exist_ok=True)

    # 환경 설정
    env_cfg = DroneBombardEnvCfg()
    env_cfg.scene.num_envs = args.num_envs

    # Isaac Lab 환경 생성
    env = DroneBombardEnv(cfg=env_cfg)

    # SB3 래퍼 적용
    env = Sb3VecEnvWrapper(env)

    # SAC 하이퍼파라미터
    sac_cfg = SacAgentCfg()

    # 체크포인트에서 이어서 학습
    if args.checkpoint:
        print(f"[train] 체크포인트 로드: {args.checkpoint}")
        model = SAC.load(
            args.checkpoint,
            env=env,
            device="cuda" if torch.cuda.is_available() else "cpu",
        )
    else:
        model = SAC(
            policy="MlpPolicy",
            env=env,
            learning_rate=sac_cfg.learning_rate,
            buffer_size=sac_cfg.buffer_size,
            batch_size=sac_cfg.batch_size,
            tau=sac_cfg.tau,
            gamma=sac_cfg.gamma,
            learning_starts=sac_cfg.learning_starts,
            ent_coef=sac_cfg.ent_coef,
            target_entropy=sac_cfg.target_entropy,
            train_freq=sac_cfg.train_freq,
            gradient_steps=sac_cfg.gradient_steps,
            policy_kwargs={"net_arch": sac_cfg.net_arch},
            tensorboard_log=args.log_dir,
            verbose=1,
            device="cuda" if torch.cuda.is_available() else "cpu",
        )

    _model_ref = model

    # 콜백 구성
    callbacks = []

    checkpoint_cb = CheckpointCallback(
        save_freq=sac_cfg.checkpoint_freq,
        save_path=os.path.join(args.log_dir, "checkpoints"),
        name_prefix="drone_drop_sac",
        save_replay_buffer=False,
    )
    callbacks.append(checkpoint_cb)

    if args.wandb:
        wb_cb = make_wandb_callback(args.wandb_project, sac_cfg)
        if wb_cb is not None:
            callbacks.append(wb_cb)

    # 학습 실행
    print(f"[train] 학습 시작 — {args.num_envs} 환경, {args.total_timesteps} 스텝")
    model.learn(
        total_timesteps=args.total_timesteps,
        callback=CallbackList(callbacks),
        log_interval=sac_cfg.log_interval,
        reset_num_timesteps=(args.checkpoint is None),
    )

    # 최종 모델 저장
    final_path = os.path.join(args.log_dir, "model_final")
    model.save(final_path)
    print(f"[train] 학습 완료 — 모델 저장: {final_path}")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
