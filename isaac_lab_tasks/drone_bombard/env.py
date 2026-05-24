"""DroneBombardEnv — ManagerBasedRLEnv 서브클래스.

커스텀 버퍼:
  payload_attached [N]    bool  — 페이로드 부착 상태
  dropped          [N]    bool  — 투하 완료 여부
  d_xy_prev        [N]    float — 이전 step 드론-타겟 수평 거리
  action_prev      [N, 5] float — 이전 step 액션 (action smoothness 계산용)
  action_cur       [N, 5] float — 현재 step 액션
  target_pos       [N, 2] float — 타겟 XY 위치 (env 로컬 좌표)
  curriculum_stage [N]    int   — 커리큘럼 단계 (0~3)
  curriculum_success_window — 각 env 의 성공/실패 기록 deque
"""

from __future__ import annotations

from collections import deque
from typing import Any

import torch
from isaaclab.envs import ManagerBasedRLEnv

from .config.drone_drop_env_cfg import DroneBombardEnvCfg
from .mdp.events import check_ccip_auto_drop
from .mdp.payload_attachment import init_payload_attachment, update_payload_attachment


class DroneBombardEnv(ManagerBasedRLEnv):
    """드론 폭격 강화학습 환경.

    ManagerBasedRLEnv 에 페이로드 부착 메커니즘과
    커리큘럼 관련 버퍼를 추가한 서브클래스.
    """

    cfg: DroneBombardEnvCfg

    def __init__(self, cfg: DroneBombardEnvCfg, **kwargs):
        super().__init__(cfg=cfg, **kwargs)
        self._init_custom_buffers()
        mode = getattr(self.cfg, "payload_attachment_mode", "kinematic_sync")
        init_payload_attachment(self, mode=mode)

    # ------------------------------------------------------------------
    # 커스텀 버퍼 초기화
    # ------------------------------------------------------------------

    def _init_custom_buffers(self) -> None:
        """커스텀 텐서 버퍼 생성."""
        n = self.num_envs
        d = self.device

        self.payload_attached = torch.ones(n, dtype=torch.bool, device=d)
        self.dropped = torch.zeros(n, dtype=torch.bool, device=d)
        self.d_xy_prev = torch.zeros(n, dtype=torch.float32, device=d)
        self.action_prev = torch.zeros(n, 5, dtype=torch.float32, device=d)
        self.action_cur = torch.zeros(n, 5, dtype=torch.float32, device=d)

        # 타겟 초기 위치 (env 로컬 좌표; reset 이벤트에서 덮어씀)
        self.target_pos = torch.zeros(n, 2, dtype=torch.float32, device=d)
        self.target_pos[:, 0] = 11.0
        self.target_pos[:, 1] = 10.0

        # 커리큘럼
        self.curriculum_stage = torch.zeros(n, dtype=torch.long, device=d)
        self.curriculum_success_window = [
            deque(maxlen=100) for _ in range(n)
        ]

        # 착탄 오차 / 투하 시점 CCIP (Gazebo drop_calculator / info 키 호환)
        self.drop_error_actual = torch.full(n, float("nan"), dtype=torch.float32, device=d)
        self.d_impact_at_drop = torch.full(n, float("nan"), dtype=torch.float32, device=d)

        # Step별 보상 컴포넌트 (WandB 로깅)
        self._last_rew_dist = torch.zeros(n, dtype=torch.float32, device=d)
        self._last_rew_drop = torch.zeros(n, dtype=torch.float32, device=d)

    # ------------------------------------------------------------------
    # Physics step 훅 — 페이로드 부착 / CCIP 자동투하
    # ------------------------------------------------------------------

    def _pre_physics_step(self, action: torch.Tensor) -> None:
        """physics step 이전: 액션 저장 → 페이로드 위치 동기화 → CCIP 자동투하."""
        # action_cur 갱신 (stability reward 에서 Δa 계산 전에 swap)
        self.action_prev[:] = self.action_cur
        self.action_cur[:] = action

        super()._pre_physics_step(action)

        # 페이로드 부착 위치 동기화
        update_payload_attachment(self)

        # CCIP 자동 투하 체크
        threshold = getattr(self.cfg, "auto_drop_threshold", 0.5)
        check_ccip_auto_drop(self, auto_drop_threshold=threshold)

    # ------------------------------------------------------------------
    # 에피소드 종료 후처리 — 커리큘럼 기록 업데이트
    # ------------------------------------------------------------------

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        terminated, truncated = super()._get_dones()

        # 종료된 환경의 성공 여부를 커리큘럼 윈도우에 기록
        done = terminated | truncated
        if done.any():
            payload = self.scene["payload"]
            payload_xy = payload.data.root_pos_w[:, :2]
            d_error_all = torch.norm(payload_xy - self.target_pos, dim=-1)

            # 착지 시 실측 miss distance 기록 (Gazebo drop_calculator 대체)
            landed = self.dropped & (payload.data.root_pos_w[:, 2] <= 0.04)
            if landed.any():
                self.drop_error_actual[landed] = d_error_all[landed]

            done_ids = done.nonzero(as_tuple=False).squeeze(-1)
            for idx in done_ids.tolist():
                d_error = d_error_all[idx].item()
                success = self.dropped[idx].item() and d_error <= 0.5
                self.curriculum_success_window[idx].append(float(success))

        return terminated, truncated

    # ------------------------------------------------------------------
    # extras 에 디버그 정보 추가
    # ------------------------------------------------------------------

    def _get_extras(self) -> dict[str, Any]:
        extras = super()._get_extras()

        payload = self.scene["payload"]
        d_payload = torch.norm(
            payload.data.root_pos_w[:, :2] - self.target_pos, dim=-1
        )
        d_xy = torch.norm(
            self.scene["drone"].data.root_pos_w[:, :2] - self.target_pos,
            dim=-1,
        )
        extras["mean_d_xy"] = d_xy.mean().item()
        extras["payload_drop_rate"] = self.dropped.float().mean().item()
        extras["curriculum_stage_mean"] = self.curriculum_stage.float().mean().item()

        valid_err = self.drop_error_actual[torch.isfinite(self.drop_error_actual)]
        if valid_err.numel() > 0:
            extras["drop_error_actual_m"] = valid_err.mean().item()
            extras["drop_success_rate"] = (valid_err <= 0.5).float().mean().item()
        else:
            extras["drop_error_actual_m"] = float("nan")

        valid_impact = self.d_impact_at_drop[torch.isfinite(self.d_impact_at_drop)]
        if valid_impact.numel() > 0:
            extras["d_impact"] = valid_impact.mean().item()

        extras["rew_dist"] = self._last_rew_dist.mean().item()
        extras["rew_drop"] = self._last_rew_drop.mean().item()
        return extras
