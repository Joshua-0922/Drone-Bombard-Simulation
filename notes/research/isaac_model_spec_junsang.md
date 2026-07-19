---
date: 2026-07-19
tags: [research, isaac, model-spec, parameters, config, reference]
type: research
status: active
owner: junsang
---

# Isaac 드론 모델 스펙 — 전체 파라미터 한눈에

> **목적:** "현재 모델이 뭐고 파라미터가 어떻게 설정돼 있나"를 **한 노트로**. 모든 cfg 값 + 버전별
> 차이. 코드: `Issac_JS/isaac_lab/drone_bombard/v11_env.py` (cfg 상속 체인).
> 관련: [[research/isaac_v11_v13_design_guide_junsang]] · [[research/isaac_expansion_roadmap_junsang]]

---

## 현재 최종 모델 = v19 (전체 통합)
랜덤타겟 + blind 탐색 + 픽셀 양자화 인지 + DR(바람/드래그) + 기체바람 + CCIP residual + **실제 물리 payload 낙하**. obs **28D**, action **7D**, PPO(rsl_rl).

---

## 1. 공통 (base) — 물리/알고리즘
| 항목 | 값 |
|------|-----|
| 알고리즘 | PPO (rsl_rl), net_arch 기본 |
| 물리 dt | 1/100 (100Hz) |
| decimation | 10 → **정책 10Hz** |
| 컨트롤러 | 캐스케이드 속도→wrench (velocity P → thrust → attitude P → rate P) |
| 기체 | Crazyflie articulation |
| env_spacing | 16m / num_envs 학습 512 |
| 학습 | max_iterations 300 (v16 250, v18-P2·v19 200), ~24 step/iter → ~360만 step |

## 2. 시나리오 (v11 base)
| 파라미터 | 값 |
|------|-----|
| marker_dist | 20m (정면) |
| cruise_speed | 4 m/s (핸드오프) |
| cruise_dir_deg / spawn_alt | 0° / 10m |
| marker_random / spawn_radius | v12+ True / 5m 원 |

## 3. release envelope (투하 게이트)
| 파라미터 | 값 |
|------|-----|
| **release_radius** | **1.0** (v18 완화 1.5, hard 1.0) |
| alt_min / alt_max | 3 / 8 m |
| max_speed / max_vz / max_tilt / max_ang_vel | 5 / 3 / 0.35rad / 4 |

## 4. 보상 (v11)
| 파라미터 | 값 | | 파라미터 | 값 |
|------|-----|---|------|-----|
| w_progress | 1.0 | | reward_success | **300** |
| w_ccip | 0.5 | | k_landing | 1.0 |
| k_ccip | 1.0 | | success_radius | **1.0m** |
| w_ang_vel / w_tilt | 0.05 / 0.05 | | no_drop_penalty | **-30** |
| w_action_smooth / w_time | 0.05 / 0.01 | | crash_penalty | -50 |
| gate_reward / drop_signal_reward | 0.05 / **1.0** | | out_of_range_penalty | -30 |

## 5. 버전별 추가 파라미터 (토글)
| 기능 (버전) | cfg 플래그·값 |
|------|------|
| 부분관측 (v13) | `reveal_radius=7.0`, `v13_undetected_penalty=-0.2`, obs 25D |
| DR+residual (v14) | `v14_dr=True`, `v14_residual=True`, `v14_residual_scale=3.0`, `v14_wind_std=1.0`, `v14_drag_max=0.15`, obs 27D·action 7D |
| 기체 바람 (v15) | `wind_force_enabled=True`, `wind_drag_k=0.06`, `v14_wind_std=2.0`, `v14_wind_max=5.0` |
| 물리 drop (v16) | `payload_physics_enabled=True`, `payload_phys_mass=0.1`, `radius=0.05`, `height=0.06`, `drag_k=0.005`, `mount_z=-0.14` |
| 픽셀 vision (v17) | `pixel_vision_enabled=True`, `pixel_cell_k=0.15` (cell=k·slant) |

## 6. 통합 모델 파라미터 (v18/v19)
obs **28D** (24 + wind_xy 2 + drag 1 + detected 1), action **7D**.

| 파라미터 | **완화 (Phase 1, 기본)** | **hard (Phase 2, `--v18_hard`)** |
|------|------|------|
| release_radius | 1.5 | 1.0 |
| v14_residual_scale | 2.0 | 3.0 |
| v14_wind_std | 1.5 | 2.0 |
| pixel_cell_k | 0.12 | 0.15 |
| (v19) payload_physics | True | True |

> **커리큘럼**: 완화(Phase1)로 데드락 회피 부트스트랩 → hard(Phase2) warm-start. (Rule 12)

## 7. 시각화 (show_markers일 때만, 학습 무영향)
`show_markers`, chase 카메라, 타겟 비콘, payload 하이라이트 → [[research/isaac_viz_tools_junsang]]

---

## 코드 위치
- cfg 상속: `DroneBombardV11Cfg` → V12 → V13/V14 → V15/V17 → V18 → V19 (`v11_env.py`)
- 태그: `week1-v15-analytic`, `v18-integration-phase2`

## 관련 노트
- [[research/isaac_v11_v13_design_guide_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[research/isaac_v18_curriculum_continuation_junsang]] · [[experiments/training_history]] · [[00_index]]
