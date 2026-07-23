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

**현재 best (2026-07-23): `v19_precise` iter875 — success 100% · release 100% · 실제 착탄 0.356m.**
백업: 노트북 `~/v19_backup/v19_precise_run/model_best.pt`, VM `~/v19_precise_backup/run/` + `/workspace/drone-bombard/v19_precise_best.pt`.

> **⚠️ 보상 2건 수정 이력 (반드시 인지):**
> 1. **no-drop 붕괴 수정 (A+B)** — 옛 v19(iter499)가 **상주 CCIP shaping 수확**으로 투하를 아예 안 하게 붕괴(release 0). → CCIP shaping을 **포텐셜(차분)형**으로(A) + **인엔벨로프 누진 페널티**(B). ([[experiments/exp_015_v19_abd_retrain_junsang]], Rule 13)
> 2. **정밀도 향상 (연속 착지보상)** — 성공존 내부 평평보상(0.1m·0.9m 동일)이 착탄을 0.56m로 정체 → **연속 지수 착지보상**으로 교체 → 0.356m·success 100%. ([[experiments/exp_016_v19_precision_landing_junsang]])

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
| env_spacing | 16m / num_envs 학습 **512(초기)~1024(v19 재학습)**, 평가 64 |
| 학습 | max_iterations 300 (v16 250, v18-P2·v19 200, v19_abd/precise 각 +300) · save_interval **25**(v19+) · ~3.7s/iter@1024env |

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

> **⚠️ v19 현행 보상은 아래 base식에서 3곳이 바뀜 (붕괴수정 A·B + 정밀화):**
> - **(A) CCIP 조준항을 포텐셜(차분)형으로**: $w_{ccip}\,e^{-k\,d_{imp}}$ (상주) → $w_{ccip}(e^{-k\,d_{imp,t}}-e^{-k\,d_{imp,t-1}})$. **조준을 유지만 하면 0**, 개선할 때만 보상 → "호버 수확" 국소최적 제거. `cfg.ccip_potential_shaping=True`.
> - **(B) 인엔벨로프 미투하 누진 페널티**: $-w_{loiter}\cdot(\text{게이트 열린 채 안 던진 연속 스텝수})$. 서성일수록 비용↑ → 투하 압박. `v19_w_loiter=0.02`.
> - **(정밀화) 착탄 보상을 연속형으로**(아래 $R_{land}$).
> 근거: [[experiments/exp_015_v19_abd_retrain_junsang]] · [[experiments/exp_016_v19_precision_landing_junsang]] · Rule 13.

- **착탄 터미널 보상**:

$$
R_{land}^{\text{base}} = \begin{cases} 300 & e_{land} \le 1\ \text{m} \\ 300\,e^{-k_{land}\,e_{land}} & \text{그 외} \end{cases}
\qquad\Longrightarrow\qquad
R_{land}^{\text{v19}} = 300\,e^{-k_{land}^{v19}\,e_{land}} + 100\cdot\mathbf{1}[e_{land}\le 1]
$$

base(좌)는 **성공존 내부가 평평(300 고정)** → 0.1m·0.9m 동일 → 정밀 유인 없어 착탄 0.56m 정체.
v19(우, `precise_landing_reward=True`, $k_{land}^{v19}=2.0$)는 **0m까지 계속 당기는 연속 지수** + 성공 이산 보너스(100) → 착탄 **0.356m·success 100%**.

### 항목별 (값 / 수식 / 의의)
| 항 | 값 | 수식 | 의의 |
|----|-----|------|------|
| 접근 progress | `w_progress`=1.0 | $w_{prog}(d_{prev}-d_{xy})$ | **potential-based shaping**: 가까워지면 +, 멀어지면 −. 매 스텝 방향 유도, 최적정책 불변 |
| CCIP 조준 | `w_ccip`=0.5, `k_ccip`=1.0 | **v19: 차분** $w_{ccip}(e^{-k d_{t}}-e^{-k d_{t-1}})$ / base: $w_{ccip}e^{-k d_{imp}}$ | **v19는 포텐셜(차분)형**: 조준 개선 시만 +, 유지 시 0 (base 상주형은 완벽조준 호버로 수확 → 붕괴, Rule 13) |
| 각속도 | `w_ang_vel`=0.05 | $-w_{\omega}\lVert\omega\rVert^2$ | 제곱: 급회전 억제 → bad_attitude 방지 |
| 기울기 | `w_tilt`=0.05 | $-w_{tilt}(\phi^2+\theta^2)$ | roll·pitch² 억제 |
| 부드러움 | `w_action_smooth`=0.05 | $-w_{sm}\lVert\Delta a\rVert^2$ | action 변화² 억제 → 부드러운 제어(sim-to-real 유리) |
| 시간 | `w_time`=0.01 | $-w_{time}$ | 빨리 접근·투하 유도(배회 방지) |
| 게이트 | `gate_reward`=0.05 | $w_{gate}\,\mathbf{1}_{gate}$ | envelope(투하가능) 진입·체류 유도 |
| drop_signal | `drop_signal_reward`=1.0 | $w_{drop}\,\mathbf{1}_{gate\cap drop}$ | **투하 시도 유도**. 게이트 밖 시도는 무보상·무페널티(공짜) |
| **loiter (B, v19)** | `v19_w_loiter`=0.02 | $-w_{loiter}\cdot n_{gate}$ | **인엔벨로프 미투하 누진 페널티**: 게이트 열린 채 안 던진 연속 스텝수 $n_{gate}$에 비례 → 서성이면 손해, 투하 압박 (붕괴수정 B) |
| 착탄 터미널 ⭐ | `reward_success`=300, `k_landing`=1.0/**v19 2.0**, `success_radius`=1.0, **`v19_success_bonus`=100** | 위 $R_{land}^{v19}$ | **최종 목표**. v19는 **연속 지수+성공보너스** → 성공존 안에서도 0m까지 정밀 유도(평평보상 제거) |
| crash | -50 | — | 지면충돌·저고도 회피 |
| out_of_range | -30 | — | 타겟 이탈 회피 |
| no_drop timeout | -30 | — | **안 떨어뜨리고 끝나면 손해** → hover exploit 방지 |
| 미탐지 (v13+) | `v13_undetected_penalty`=-0.2 | $(1-\mathbf{1}_{det})\,(-0.2)$ | 못 찾을 때 매 스텝 벌 → blind 탐색 유도 |

### 왜 이런 함수 형태인가 (설계 의의)
- **지수 $e^{-k\,d}$ (조준·착탄)**: 근접에서 gradient가 커 **정밀함을 강하게 유도**. 선형은 근접 민감도 낮음. $k$로 보상이 좁아지는 급함 조절.
- **제곱 (자세·부드러움)**: 값 클수록 급증 → **극단만 강하게 억제**, 정상 범위는 관대.
- **potential-based (progress·v19 CCIP)**: 조밀한 방향 신호를 주되 **최적정책 불변**(Ng et al. 1999). ⭐ **shaping은 "유지"가 아니라 "개선"에만 줘라** — 상주형(절대값 $e^{-kd}$)은 목표 근처서 가만있어도 매 스텝 수확 → 노이즈 큰 터미널 대신 "완벽조준+호버(안 던짐)" 국소최적으로 **붕괴**(v19 release 0). 차분형이 이를 원천 차단 (**Rule 13**).
- **연속 착탄 (v19)**: base의 "성공존 만점 평평"은 명확하지만 **내부 정밀 gradient가 없어 정체** → 연속 지수+성공보너스로 **0m까지 계속 당김**(성공 신호는 이산 보너스로 유지). 착탄 0.56→0.36m.
- **비대칭 투하 유도**: 시도 +1 · 성공 +300 · 안 함 -30 · 게이트밖 시도 공짜 + **인엔벨로프 누진 페널티(B)** → **"일단, 그리고 정확히 투하하게"** (Rule 12·13).

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

### v19 붕괴수정·정밀 cfg (2026-07-23 추가, 전부 토글)
| 플래그 | 값 | 역할 |
|------|------|------|
| `ccip_potential_shaping` | True | **A**: CCIP 조준 shaping 차분(포텐셜)형 — 호버 수확 붕괴 방지 |
| `v19_w_loiter` | 0.02 | **B**: 인엔벨로프 미투하 누진 페널티 계수 |
| `precise_landing_reward` | True | 연속 착지보상 on (off면 기존 평평형) |
| `v19_k_landing` | 2.0 | 연속 착지보상 지수 급함(정밀도 압박) |
| `v19_success_bonus` | 100 | 성공존 진입 이산 보너스 |

> 재학습: v18-P2 → v19_abd(A+B+D, iter600, success 76.7%·0.563m) → v19_precise(연속착탄, iter875, **success 100%·0.356m**). 코드 `Issac_JS 09a53a2`.

## 7. 시각화 (show_markers일 때만, 학습 무영향)
`show_markers`, chase 카메라, 타겟 비콘, payload 하이라이트 → [[research/isaac_viz_tools_junsang]]

---

## 7.5 메커니즘 상세 — 바람 · 비전 · payload · marker (수식+수치)

> 아래는 코드 실값(`v11_env.py` / `drone_bombard_env.py`) 기준. §8·§9는 개념 설명, 여기는 **정확한 수식·수치**.

### 🌬 바람(wind) 메커니즘
- **샘플링**: 에피소드 리셋마다 각 수평축 독립 정규분포 — $w_{x},w_{y}\sim \mathcal{N}(0,\ \sigma_w^2)$, 이후 크기 $\lVert w\rVert$를 $w_{max}$로 클램프. (`v11_env.py:_reset_idx` L590)
  - $\sigma_w$ = `v14_wind_std` = **1.5 m/s** (v19; v14=1.0, v15/v18-hard=2.0), $w_{max}$=`v14_wind_max`=**5.0 m/s**.
  - **에피소드 내 일정**(정상풍), **방향 균일 랜덤**, **수평만**(수직 없음). 시변 돌풍=미구현(Stage B).
- **기체 작용** (`wind_force_enabled=True`, v15): 상대기류에 2차 항력 — $F = k_{air}\,\lVert v_{air}\rVert\,v_{air}$, $v_{air}=w - v_{drone}$. $k_{air}$=`wind_drag_k`=**0.06** N/(m/s)². → 드론이 밀려 위치유지에 힘 써야 함.
- **payload 작용**: §payload 참조(낙하 중 드리프트).
- **예측 반영**: `_wind_xy`가 탄도 예측·obs 채널(정규화 `v14_wind_obs_scale`=6.0)에 들어감 → 정책이 바람 알고 residual 보정.
- **항력계수 DR**: `_drag_coef` $\sim U[0,\ 0.15]$(`v14_drag_max`) — payload 탄도계수 랜덤화, 별도.
- ⚠️ wind 너무 세면 드리프트 > residual 범위 → **포화**(Rule 11). 그래서 v19는 1.5로 완화.

### 👁 비전(픽셀 양자화) 메커니즘
2단계: **① 거리 게이트 reveal** + **② 픽셀 양자화**.
- **reveal**: 수평거리 $d_{xy}\le$ `reveal_radius`=**7.0 m** 이면 위치 obs 공개(`detected=1`), 아니면 마스킹(0)+미탐지 페널티 −0.2/step.
- **픽셀 양자화** (`_quantize_target`, L898): 인지 위치를 셀 중심으로 스냅.
$$
\text{slant}=\sqrt{\lVert rel\rVert^2 + h^2},\quad
cell = \max(k_{px}\cdot\text{slant},\ 0.05),\quad
\hat{p} = p_{drone} + \big(\lfloor rel/cell\rfloor + 0.5\big)\cdot cell
$$
  - $rel = p_{target}-p_{drone}$(수평), $h$=고도, $k_{px}$=`pixel_cell_k`=**0.12**(v19; v17=0.15). 셀 최소 0.05m.
  - → **멀면 셀 커서 애매, 가까울수록 정밀**. 정책은 $\hat{p}$(인지값)로만 조준, **success는 실제 표적**.

### 📦 payload 물리 수치 (v16/v19, `payload_physics_enabled=True`)
| 파라미터 | 값 | 의미 |
|------|------|------|
| `payload_phys_mass` | **0.1 kg** | payload 질량 |
| `payload_phys_radius` / `payload_phys_height` | **0.05 / 0.06 m** | 실린더(RigidObject) 치수 |
| `payload_phys_drag_k` | **0.005** N/(m/s)² | 낙하 항력계수 ($=\tfrac12\rho C_d A$, 전면적 ~0.008 m²) |
| `payload_mount_z` | **−0.14 m** | 드론 아래 매달림 오프셋 |
| `payload_ground_z` | **0.0 m** | 착지 판정 평면(local) |
- **낙하 물리** (`drone_bombard_env.py:_step_payload_physics`, 100Hz): 부착 중엔 드론에 kinematic으로 매달려 이동 → 투하 시 분리 → 자유낙하 항력 $F=k_{pl}\,\lVert v_{air}\rVert\,v_{air}$ ($v_{air}=[w_x-v_x,\ w_y-v_y,\ -v_z]$) → $z\le$ ground_z에서 착지, `_payload_impact_xy` 기록.
- (탄도 계산용 `payload_mass`=0.1, `drag_coef`=위 DR값도 별도로 예측에 사용.)

### 🎯 marker(표적) 랜덤 생성
- **중심**: cruise 방향으로 `marker_dist`=**20 m** 앞 → 기본 (20, 0).
- **랜덤**(`marker_random=True`, v12+): 중심 기준 **반경 `marker_spawn_radius`=5 m 원 안, 면적 균일**. (`_reset_idx` L406)
$$
r = R\sqrt{U[0,1]},\quad \theta = U[0,2\pi),\quad p_{target} = \text{center} + (r\cos\theta,\ r\sin\theta)
$$
  - **$\sqrt{\cdot}$가 핵심**: 그냥 $r=R\cdot U$면 중심에 몰림 → $\sqrt{}$로 **면적 균일**(원 전체 고르게).
  - 드론은 여전히 +X로 순항 스폰 → **축을 벗어난 표적으로 조향**해야 함.

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
