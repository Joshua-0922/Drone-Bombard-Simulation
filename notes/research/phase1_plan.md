---
date: 2026-04-17
tags: [research, plan, phase1, CCIP, SAC, ablation]
status: active
type: research
---

# Phase 1: CCIP 기반 자율 접근 비행 제어기 — 상세 연구 계획

> **연구 기간:** 2026-05-08 ~ 2026-06-30 (8주)
> **목표:** CCIP 예측 착탄점 기반 auto-drop + SAC 수렴 + 4-layer ablation + 보상 해킹 검증

---

## 배경 및 핵심 변경

현재 `drone_drop_env.py`의 auto-drop 조건은 **드론-표적 2D 거리(d_xy) ≤ 0.5m**이다.
즉 드론이 표적 바로 위에 도달해야 투하되며, CCIP 예측기(`_predict_impact_point`)는 코드에 존재하지만 **사용되지 않는다.**

**핵심 변경:** auto-drop 트리거를 `d_xy` → `d_impact` (CCIP 예측 착탄 거리)로 교체.

**CCIP의 효과:**
$$x_p = x + v_x \cdot t_f, \quad y_p = y + v_y \cdot t_f, \quad t_f = \frac{v_z + \sqrt{v_z^2 + 2gz}}{g}$$

속도 $v$로 직진 접근 시 $d_{impact} = |d_{xy} - v \cdot t_f|$이므로, 드론이 표적에서 $v \cdot t_f$ 전방에 도달하면 자동 투하. 예: 고도 5m, $v=5$ m/s → $t_f \approx 1.01$s → 표적 5.05m 전방에서 투하.

- 드론이 표적 위까지 갈 필요 없음 → **에피소드 단축, fly-by 궤적 허용**
- Layer 3 방향 보상은 직진 접근을 유도하므로 CCIP와 **상호 보완적** (변경 불필요)

---

## 현재 상태 요약

| 항목 | 현재 값 | 비고 |
|------|---------|------|
| 관측 공간 | Box(15,) | pos(3), vel(3), ang_vel(3), vision(3), attached(1), rel_target(2) |
| 행동 공간 | Box(5,) | vx, vy, vz, yaw_rate, drop(dummy — Phase 1) |
| Auto-drop | `d_xy ≤ 0.5m` | 드론 위치 기반, CCIP 미사용 |
| CCIP 예측기 | `_predict_impact_point()` (line 633) | 구현됨, 미사용 |
| 보상 Layer 3 | `w_dist*(d_xy_prev - d_xy)` + 방향 | 선형 거리 + 방향 보상 |
| 보상 해킹 대응 | speed gate, truncation -50, w_time=0.05 | 2026-03-22 패치, 미검증 |
| 마지막 run | 8otphxy8 (114K steps) | 패치 전 베이스라인 |
| RTF | 2 (avg 59.5 fps) | 1M steps ≈ 4.7h wall-clock |
| evaluate.py 버그 | `drop_error_m` ≠ `drop_error_actual_m` | line 62 수정 필요 |

---

## 코드 변경 목록

### 변경 1: Auto-drop 트리거 — d_xy → d_impact (CCIP)
- **파일:** `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`
- **위치:** `step()`, line 548
```python
# OLD
if d_xy <= self._cfg_auto_drop_threshold and not self.dropped:

# NEW
_, _, t_f, d_impact = self._predict_impact_point(pos, vel)
if d_impact <= self._cfg_auto_drop_threshold and not self.dropped:
```

### 변경 2: t_f 클램핑 — CCIP 예측 안정화
- **파일:** 같은 파일, `_predict_impact_point()`, line 667 이후
- **추가:** `t_f = min(t_f, 10.0)` — 저속 비행 시 예측 발산 방지

### 변경 3: 관측 공간 확장 (15 → 17)
- **파일:** 같은 파일, `__init__()` + `_get_obs()`
```python
# __init__: shape=(15,) → shape=(17,)

# _get_obs() 끝에 추가:
_, _, t_f_obs, d_impact_obs = self._predict_impact_point(pos_clean, vel_raw)
obs_d_impact = np.clip(d_impact_obs / self._cfg_pos_scale, 0.0, 1.0)  # [15]
obs_t_f      = np.clip(t_f_obs / 10.0, 0.0, 1.0)                      # [16]
```
> **주의:** obs space 변경 → 기존 체크포인트 호환 불가 → **fresh start 필수**

### 변경 4: Layer 3 보상 — d_xy + 방향 보상 유지 (확정)
- 거리 보상: `r3_dist = w_dist * (d_xy_prev - d_xy)` 유지
- 방향 보상: `r3_orient = w_heading * cos(heading) * speed_gate` 유지
- d_impact는 **관측(obs[15])으로만** 제공 — 보상 신호는 d_xy 기반 유지
- 수렴 안 되면 Week 3에서 d_impact 그래디언트 전환 검토

### 변경 5: d_impact WandB 로깅
- `drone_drop_env.py`, `_compute_reward()`: `info['d_impact'] = d_impact` 추가
- `train_sac.py`, `WandbMetricsCallback`: `env/mean_d_impact` 로깅 추가

### 변경 6: evaluate.py 버그 수정 + 메트릭 추가
- **line 62:** `'drop_error_m'` → `'drop_error_actual_m'` (키 불일치 수정)
- **line 192:** `--config` CLI arg + `DroneDropEnv(config_path=...)` 추가
- `_write_report()`에 CEP50, success_rate_05m 추가

### 변경 7: 안전 위반 카운터 (ablation용)
- `train_sac.py`, `WandbMetricsCallback`: crash/overspeed 카운트 → `env/safety_violation_rate`

### 변경 8: gradient_steps 설정 가능
- `hyperparams.yaml` sac 섹션에 `gradient_steps: 1` 추가
- `train_sac.py` SAC() 생성자에 연결

### 변경 9: speed_gate 플래그 (보상 해킹 재현용)
- `hyperparams.yaml`: `speed_gate_enabled: true`
- `drone_drop_env.py`:
```python
speed_gate = min(speed_xy / 2.0, 1.0) if self._cfg_speed_gate else 1.0
```

### 변경 10: Rule-based CCIP 베이스라인 스크립트
- **새 파일:** `ros2_ws/src/rl_navigation/rl_navigation/baseline_ccip.py`
- 고정 속도 직진 비행 + CCIP auto-drop, 50 에피소드
- 출력: mean_drop_error, CEP50, success_rate → JSON

---

## WandB 네이밍 규칙

```
exp{NNN}-{variant}-{YYYYMMDD}
```

예: `exp004-baseline-ccip-20260508`, `exp006a-ablation-noR1-20260604`

- WandB 그룹 태그: `phase1-ccip`
- 공통 태그: `["sac", "phase1", "ccip"]`

---

## 주별 계획

### Week 1 (5/8 — 5/14): CCIP 통합 + Rule-Based 베이스라인

| 날짜 | 작업 | 소요 | 실험명 |
|------|------|------|--------|
| 5/8 (목) | 변경 1-6 구현 + colcon build | 3h | — |
| 5/8 (목) | Dry-run 5500 steps — obs[15-16] 확인 | 1h | Exp 004-dryrun |
| 5/9 (금) | baseline_ccip.py 작성 + 50 에피소드 | 3h | Exp 004-baseline |
| 5/9 (금) 8pm | **야간:** CCIP-SAC 1M steps fresh | ~4.7h | Exp 005a |
| 5/10 (토) 6am | 005a 결과 확인 | 1h | — |
| 5/12 (월) | 005a 분석 + exp_005.md 작성 | 2h | — |

**리스크:** d_impact obs noisy → Week 2에서 smoothing 검토

---

### Week 2 (5/15 — 5/21): 보상 가중치 교정

| 날짜 | 작업 | 소요 | 실험명 |
|------|------|------|--------|
| 5/15 (목) | 변경 7-8 구현 + WandB 분석 | 3h | — |
| 5/16 (금) | k2 교정: `k2 = -ln(0.3) / E_baseline` | 1h | — |
| 5/17 (토) 8pm | **야간:** 교정 가중치 1M steps | ~4.7h | Exp 005b |
| 5/19 (월) 8pm | **야간:** w_time 조정 변형 | ~4.7h | Exp 005c |
| 5/21 (수) | 비교 분석 + reward_design.md 업데이트 | 2h | — |

**k2 교정 논리:** baseline mean_drop_error = $E$일 때 $k_2 = 1.2 / E$
예: $E=3m$ → $k_2 \approx 0.4$ (현재 5.0에서 대폭 하향)

---

### Week 3 (5/22 — 5/28): 하이퍼파라미터 튜닝

| Exp | learning_rate | gradient_steps | buffer_size |
|-----|--------------|----------------|-------------|
| 005d | **1e-4** | 1 | 100K |
| 005e | 3e-4 | **2** | 100K |
| 005f | 3e-4 | 1 | **300K** |

- 야간 3회 (Fri/Sat/Sun)
- `batch_evaluate.py` 작성 — 다수 체크포인트 일괄 비교
- **수렴 판단:** success_rate ≥ 0.5 at 1M → 정상. < 0.3 → d_impact 그래디언트 전환

---

### Week 4 (5/29 — 6/4): 수렴 검증 + Ablation 준비

- 5/29 8pm: **Exp 005-final** — 최적 HP 2M steps (~9.4h, 8pm→5:20am)
- 5/31 (토): `evaluate.py --episodes 50` 정식 평가
- 6/1 (일): Ablation configs 4종 dry-run

**수렴 기준:** `env/success_rate ≥ 0.9` 지속 100K steps

**Ablation configs:**

| Config | 비활성 레이어 | 변경 |
|--------|------------|------|
| `hyperparams_ablation_noR1.yaml` | R1 Safety | penalty_crash=0, penalty_overspeed=0 |
| `hyperparams_ablation_noR2.yaml` | R2 Stability | w_time=0, w_ang_vel=0, w_action_smooth=0 |
| `hyperparams_ablation_noR3.yaml` | R3 Approach | w_dist=0, w_heading=0 |
| `hyperparams_ablation_noR4.yaml` | R4 Terminal | w_drop_base=0, r_success_jackpot=0 |

---

### Week 5 (6/4 — 6/10): Ablation Part 1 (R1, R2)

- 6/4 8pm: **Exp 006a** — no-R1 (Safety 제거)
- 6/5 8pm: **Exp 006b** — no-R2 (Stability 제거)
- **예상:** no-R1 → crash↑; no-R2 → 불안정 행동, instability penalty↑

---

### Week 6 (6/11 — 6/17): Ablation Part 2 (R3, R4)

- 6/11 8pm: **Exp 006c** — no-R3 (Approach 제거) → success_rate ≈ 0 예상
- 6/12 8pm: **Exp 006d** — no-R4 (Terminal 제거) → 투하 정확도 무관심 예상
- 6/14 (토): 전체 ablation 비교 테이블

| 설정 | success_rate | mean_drop_error | CEP50 | ep_len_mean | safety_violations |
|------|-------------|-----------------|-------|-------------|-------------------|
| Full | — | — | — | — | — |
| no-R1 | — | — | — | — | ↑ |
| no-R2 | — | — | — | ↑ | — |
| no-R3 | ≈0 | — | — | — | — |
| no-R4 | — | ↑ | — | — | — |

---

### Week 7 (6/18 — 6/24): 보상 해킹 재현 및 검증

- 6/18 8pm: **Exp 007a** — Spiral Milking (`speed_gate_enabled: false`)
- 6/19 8pm: **Exp 007b** — Kamikaze Dive (`penalty_crash: 0`, `min_altitude: 0`)

**Spiral Milking 검증 기준:**
- ep_len_mean ≈ 500, mean_rew_orient >> baseline, success_rate ≈ 0
- 궤적: 표적 주변 원형/나선형

**Kamikaze Dive 검증 기준:**
- safety_violation_rate >> 0, 투하 시 고도 < 2m, 속도 >> baseline

---

### Week 8 (6/25 — 6/30): 결과 정리 + 중간보고서

- 최종 50-episode 평가
- WandB Report: 전체 실험 그룹 비교
- 중간보고서 구조: 문제 정의 → 방법론 → 베이스라인 → 학습 결과 → Ablation → 보상 해킹 → Phase 2 전망

---

## 자동화 전략 (군 복무 제약 대응)

```bash
# 야간 학습 시작 (9pm 전)
docker exec -d rtf-dryrun bash -c "
  source /root/ros2_ws/install/setup.bash && \
  source /workspace/ros2_ws/install/setup.bash && \
  ros2 run rl_navigation train_sac \
    --config /workspace/ros2_ws/src/rl_navigation/config/{CONFIG}.yaml \
    2>&1 | tee /workspace/ros2_ws/rl_checkpoints/{EXP_NAME}.log
"

# 다음 날 6am 확인
docker exec rtf-dryrun bash -c \
  "tail -50 /workspace/ros2_ws/rl_checkpoints/{EXP_NAME}.log"
```

- **VM 프리엠션:** `train_managed.sh` auto-resume + `_emergency_save()` + `wandb resume='allow'`

---

## 의존성 체크리스트

| 리소스 | 필요량 | 상태 |
|--------|--------|------|
| GCP Spot VM (L4 GPU) | ~110h | 확인 필요 |
| 디스크 | ~3GB | `train_managed.sh` 90% 임계값 감시 |
| Docker 이미지 | `drone-bombard-px4built:latest` | PX4 빌드 + airframes 포함 ✅ |
| WandB | team entity | 설정됨 ✅ |
| Network | Termius + Guacamole | 작동 중 ✅ |

## 전체 일정 요약

| 주차 | 야간 | 핵심 질문 |
|------|------|----------|
| 1 (5/8-14) | 1 | CCIP obs + 투하 작동하는가? |
| 2 (5/15-21) | 2 | 어떤 가중치가 맞는가? |
| 3 (5/22-28) | 3 | 어떤 HP가 빨리 수렴하는가? |
| 4 (5/29-6/4) | 1 | success_rate ≥ 0.9 도달? |
| 5 (6/4-10) | 2 | 안전/안정성 계층이 필요한가? |
| 6 (6/11-17) | 2 | 접근/투하 계층이 필요한가? |
| 7 (6/18-24) | 2 | 대응책이 효과적인가? |
| 8 (6/25-30) | 0 | 문서 완성 |
| **합계** | **~13** | |

---

## 관련 노트

- [[research/reward_design]] — 4-layer 보상 함수 상세
- [[research/rl_rules]] — RL 실험 규칙 (Rule 7: RTF=2)
- [[research/rtf_fps_analysis]] — RTF 최적화 연구 결과
- [[experiments/training_history]] — 전체 WandB 히스토리
- [[experiments/exp_002_reward_shaping_patches]] — 보상 패치 (fresh start 대기 중)
