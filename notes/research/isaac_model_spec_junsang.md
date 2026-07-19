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

## 관측(obs) · 행동(action) 채널

### action (7D)
| idx | 역할 | 설명 |
|-----|------|------|
| 0 | vx | 전후 속도 setpoint (×vx_scale 4 → ±4 m/s) |
| 1 | vy | 좌우 속도 (×vy_scale 3) |
| 2 | vz | 상하 속도 (×vz_scale 3) |
| 3 | yaw_rate | 요 각속도 (×yaw_scale 1) |
| **4** | **drop_signal** | 투하 의도 (`>0.5`면 시도, 게이트 열렸을 때만 실제 투하) |
| **5** | **residual Δx** | CCIP 예측착탄 x 보정 (×residual_scale → ±2~3m) |
| **6** | **residual Δy** | CCIP 예측착탄 y 보정 (바람 드리프트 보정) |
- `[0:4]` 비행 · `[4]` 투하타이밍 · `[5:7]` 잔차보정. **6D**(v11~13,16,17)=residual 없음 / **7D**(v14,15,18,19)=residual 있음.

### obs (24D 기준; 정규화 clamp[-1,1])
| idx | 채널 | 정규화 |
|-----|------|------|
| 0,1 | marker 상대위치 rel_x, rel_y | /20 |
| 2 | rel_z (= −고도) | /20 |
| 3,4,5 | 속도 vx,vy,vz | /10 |
| 6,7 | roll, pitch | /π |
| 8,9 | sin(yaw), cos(yaw) | — |
| 10,11,12 | 각속도 ωx,ωy,ωz | /π |
| 13,14 | CCIP 오차 ex, ey | /10 |
| 15 | d_impact (예측착탄-타겟 거리) | /10 |
| 16 | t_f (낙하 예상시간) | /5 |
| 17 | 수평속력 | /10 |
| 18 | 고도 | /20 |
| 19 | payload_attached (0/1) | — |
| 20~23 | 이전 action[0:4] | — |

**확장 시 추가 채널**: v13 = +**detected**(1) → 25D · v14 = +**wind_xy**(2)+**drag**(1) → 27D · v18/19 = 24+wind/drag(3)+detected(1) = **28D**. (v13/17/18/19: marker 채널 0,1,13,14,15는 **미탐지 시 0으로 마스킹**)

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

## 4. 보상 함수 (수식 + 의의)

**per-step 총 보상** (v11 기준; v13/18은 detected 게이팅·미탐지 페널티 추가):

$$
r_t = w_{prog}(d_{prev}-d_{xy}) + w_{ccip}\,e^{-k_{ccip}\,d_{imp}} - w_{\omega}\lVert\omega\rVert^2 - w_{tilt}(\phi^2+\theta^2) - w_{sm}\lVert\Delta a\rVert^2 - w_{time} + w_{gate}\,\mathbf{1}_{gate} + w_{drop}\,\mathbf{1}_{gate\cap drop} + \mathbf{1}_{rel}\,R_{land} - P
$$

- 앞부분 = 접근·조준·자세/부드러움/시간(dense shaping), 중간 = 투하 유도, $R_{land}$ = 착탄 터미널, $P$ = 실패 페널티.
- **착탄 터미널 보상** (성공존 만점 + 밖은 지수 감쇠):

$$
R_{land} = \begin{cases} 300 & e_{land} \le 1\ \text{m} \\ 300\,e^{-k_{land}\,e_{land}} & \text{그 외} \end{cases}
$$

### 항목별 (값 / 수식 / 의의)
| 항 | 값 | 수식 | 의의 |
|----|-----|------|------|
| 접근 progress | `w_progress`=1.0 | $w_{prog}(d_{prev}-d_{xy})$ | **potential-based shaping**: 가까워지면 +, 멀어지면 −. 매 스텝 방향 유도, 최적정책 불변 |
| CCIP 조준 | `w_ccip`=0.5, `k_ccip`=1.0 | $w_{ccip}\,e^{-k_{ccip}\,d_{imp}}$ | **지수**: 예측착탄-타겟 거리 0이면 최대(0.5), 멀수록 0. **근접에서 gradient 급증** → 선형보다 정밀 조준 |
| 각속도 | `w_ang_vel`=0.05 | $-w_{\omega}\lVert\omega\rVert^2$ | 제곱: 급회전 억제 → bad_attitude 방지 |
| 기울기 | `w_tilt`=0.05 | $-w_{tilt}(\phi^2+\theta^2)$ | roll·pitch² 억제 |
| 부드러움 | `w_action_smooth`=0.05 | $-w_{sm}\lVert\Delta a\rVert^2$ | action 변화² 억제 → 부드러운 제어(sim-to-real 유리) |
| 시간 | `w_time`=0.01 | $-w_{time}$ | 빨리 접근·투하 유도(배회 방지) |
| 게이트 | `gate_reward`=0.05 | $w_{gate}\,\mathbf{1}_{gate}$ | envelope(투하가능) 진입·체류 유도 |
| drop_signal | `drop_signal_reward`=1.0 | $w_{drop}\,\mathbf{1}_{gate\cap drop}$ | **투하 시도 유도**. 게이트 밖 시도는 무보상·무페널티(공짜) |
| 착탄 터미널 ⭐ | `reward_success`=300, `k_landing`=1.0, `success_radius`=1.0 | 위 $R_{land}$ | **최종 목표**. 성공존 만점, 밖은 지수 감쇠 → 아깝게 빗나감도 부분보상 |
| crash | -50 | — | 지면충돌·저고도 회피 |
| out_of_range | -30 | — | 타겟 이탈 회피 |
| no_drop timeout | -30 | — | **안 떨어뜨리고 끝나면 손해** → hover exploit 방지 |
| 미탐지 (v13+) | `v13_undetected_penalty`=-0.2 | $(1-\mathbf{1}_{det})\,(-0.2)$ | 못 찾을 때 매 스텝 벌 → blind 탐색 유도 |

### 왜 이런 함수 형태인가 (설계 의의)
- **지수 $e^{-k\,d}$ (조준·착탄)**: 근접에서 gradient가 커 **정밀함을 강하게 유도**. 선형은 근접 민감도 낮음. $k$로 보상이 좁아지는 급함 조절.
- **제곱 (자세·부드러움)**: 값 클수록 급증 → **극단만 강하게 억제**, 정상 범위는 관대.
- **potential-based (progress)**: 조밀한 방향 신호를 주되 **최적정책 불변**(reward shaping 정석).
- **binary+지수 혼합 (착탄)**: 성공존은 만점(명확한 목표), 밖은 지수 부분보상(학습 신호 끊김 방지).
- **비대칭 투하 유도**: 시도 +1 · 성공 +300 · 안 함 -30 · 게이트밖 시도 공짜 → **"일단 투하해보게"** (Rule 12).

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

## 8. Vision — 현재 어떻게 하고 있나

> **중요**: base env엔 실제 핀홀 카메라 vision(`_update_vision`: down-camera 투영 → u,v,conf)이
> 있지만 **v11+는 안 씀(제거)**. 현재 vision은 이를 **간이화한 프록시** 2단계:

**① 거리 게이트 reveal (v13)** — 카메라가 아니라 "근접하면 켜지는 센서":
- 드론이 타겟과 **수평거리 ≤ reveal_radius(7m)** 이면 위치 obs 공개, 아니면 마스킹(0) + `detected` 플래그.
- 벗어나면 다시 OFF + 미탐지 페널티 $-0.2$/step.

**② 픽셀 양자화 (v17)** — reveal 켜졌을 때 위치를 **정확히가 아니라 셀 중심으로** 스냅:
- 셀 크기 $cell = k_{px}\cdot \text{slant}$ (slant = 드론→타겟 3D거리, $k_{px}$=0.15)
- 인지 위치 $\hat{p} = p_{drone} + (\lfloor rel/cell \rfloor + 0.5)\cdot cell$
- → **멀면 셀 큼(애매)·가까우면 셀 작음(정밀)**. 다가갈수록 참값 수렴.
- 통합(v18/19)은 reveal(7m) + 픽셀 양자화 함께.

정책은 **이 인지값(양자화)만** 보고 조준(obs·게이트·residual 계산 전부 인지 타겟 기준), **success는 실제 타겟으로** 판정 → 애매한 조준을 접근으로 다듬어야 명중.

⚠️ **아직 안 함**: 실제 핀홀 카메라/YOLO 이미지 인지(footprint 고도의존 등)는 미사용 — 픽셀 양자화가 그 근사. 로드맵 다음 축.

---

## 9. CCIP residual — 현재 어떻게 하고 있나

> **목적**: 바람/드래그로 nominal(무바람) 탄도 예측이 빗나가는 걸 정책이 학습으로 보정.

**계산 흐름**:
- nominal 예측: $p_{nom}$ = `predict_impact_nominal`(위치·속도·고도) — 드래그·바람 무시
- residual 보정: $p_{corr} = p_{nom} + a_{[5:7]}\cdot scale$ ($scale$ = residual_scale, 2~3m)
- **보정 예측 $p_{corr}$가 obs CCIP 채널·release 게이트·CCIP shaping 보상에 모두 사용** (정책은 "보정한 예측"으로 조준·투하 결정)
- **실제 착탄**: `ballistic_impact(…, 실제 drag/wind)` → success/터미널 보상 채점

→ 정책은 residual로 "바람 때문에 이만큼 흘러가니 미리 보정"을 학습. 보정이 실제 drift와 맞으면 명중.

**wind trap 해결 (Stage A)**: env마다 바람이 다른데 정책이 모르면 보정 불가 → **바람/드래그를 obs 채널로 직접 제공**(v14, obs +3D). 정책이 바람→residual 매핑 학습. (Stage B = 바람 관측 없이 운동에서 추정, 미구현)

**주의**:
- residual 범위(±scale)가 드리프트보다 작으면 **포화** → wind_std↑ 시 scale도↑ (**Rule 11**)
- 초기 랜덤 residual이 release 게이트 봉쇄 → 통합 시 **데드락** → 커리큘럼(게이트 완화+warm-start) (**Rule 12**)

---

## 코드 위치
- cfg 상속: `DroneBombardV11Cfg` → V12 → V13/V14 → V15/V17 → V18 → V19 (`v11_env.py`)
- 태그: `week1-v15-analytic`, `v18-integration-phase2`

## 관련 노트
- [[research/isaac_v11_v13_design_guide_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[research/isaac_v18_curriculum_continuation_junsang]] · [[experiments/training_history]] · [[00_index]]
