"""SB3 SAC 하이퍼파라미터 설정.

기존 Gazebo 파이프라인의 hyperparams.yaml 에서 이전.
lr=3e-4, buffer=100K, batch=256, tau=0.005, gamma=0.99, net=[256,256]
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class SacAgentCfg:
    """Stable-Baselines3 SAC 하이퍼파라미터."""

    # 학습률
    learning_rate: float = 3e-4

    # 리플레이 버퍼 크기 (L4 24 GB 에서 100K 로 시작; OOM 시 줄임)
    buffer_size: int = 100_000

    # 미니배치 크기
    batch_size: int = 256

    # Polyak 평균 계수 (타겟 네트워크 소프트 업데이트)
    tau: float = 0.005

    # 할인율
    gamma: float = 0.99

    # 학습 시작 전 수집 스텝 수
    learning_starts: int = 10_000

    # 정책/Q-함수 네트워크 구조
    net_arch: List[int] = field(default_factory=lambda: [256, 256])

    # 행동 공간 정규화 사용 여부
    use_sde: bool = False

    # 엔트로피 계수 (None = 자동 조정)
    ent_coef: str = "auto"

    # 타겟 엔트로피 ("auto" 사용 시 무시)
    target_entropy: str = "auto"

    # 업데이트 주기 (매 N 스텝마다 그래디언트 업데이트)
    train_freq: int = 1

    # 그래디언트 업데이트 횟수 (train_freq 당)
    gradient_steps: int = 1

    # WandB 프로젝트 이름
    wandb_project: str = "drone-bombard-isaac"

    # 체크포인트 저장 간격 (스텝)
    checkpoint_freq: int = 50_000

    # 총 학습 스텝 수
    total_timesteps: int = 5_000_000

    # 로그 출력 간격 (스텝)
    log_interval: int = 100
