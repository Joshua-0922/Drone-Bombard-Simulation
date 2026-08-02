---
date: 2026-08-02
tags: [research, overview, paper, ablation, roadmap, isaac-lab, gazebo, ccip, curriculum]
status: active
type: research
---

# 연구 전체 통합 개요 — 계보 · warm-start 체인 · 논문화 전략

> **목적:** 흩어진 `notes/`(제균 트랙) + archive 태그에만 남은 `*_junsang.md`(준상 v-track) +
> 코드(`v11_env.py`, `drone_bombard_env.py`)를 **한 문서**로 합쳐, "각 버전이 무엇을 추가했고 /
> 어디서 warm-start 했고 / 무엇이 증명됐는가"를 정리하고 **논문 ablation·차별점**을 설계한다.
> 근거 노트는 각 절에 wikilink. 허브: [[00_index]] · [[research/rl_rules]] · [[experiments/training_history]]

> ⚠️ **문서 위생 이슈**
> 1. ~~준상 v-track 연구노트가 `main`에 없다~~ → **✅ 2026-08-02 복원 완료.** `notes/**/*_junsang.md`
>    **34개**를 태그 `archive/main-pre-isaac_jk-promotion`에서 `main`으로 복원(구 main 아카이빙 시 같이
>    묻혔던 것). v11~v19의 1차 사료이므로 이제 논문에서 직접 인용 가능.
> 2. **`exp_NNN` 번호가 두 계열에서 충돌한다.** `exp_006`~`exp_016`이 제균 트랙(Gazebo/Isaac base)과
>    준상 v-track에 각각 존재 — 본 문서는 **제균 계열 = `exp_NNN`**, **준상 계열 = `v-exp_NNN`**으로 구분 표기.

---

## 0. 한 문단 요약

**문제:** 드론이 이동/정지 표적에 페이로드를 **정밀 투하**한다. 고전 CCIP(Constantly Computed
Impact Point) 탄도 예측을 조준 프리미티브로 두고, **RL은 (a) 릴리스 타이밍과 (b) 모델 오차
잔차(residual)만** 학습한다.

**궤적:** Gazebo/PX4 + SAC(단일 env, 실기 스택 근접) → Isaac Lab + PPO(2048 병렬 env)로 이전.
Isaac 이후 **두 갈래로 분기**해 병렬 진행: (B) *base env* = phase 커리큘럼 + 진단 중심,
(C) *v-track* = 능력을 한 축씩 쌓아 올리는 사다리(v11→v20). 두 갈래 모두 "**투하라는 이벤트
능력은 shaping이 아니라 구조(종단·게이트·보상 형태)로 만들어진다**"라는 같은 결론에 수렴했다.

**현재 최고 성능:** v-track `v19_precise` — 인지(부분관측+픽셀양자화) + 물리(DR 바람/드래그 +
기체 바람 + CCIP residual) + **실제 RigidObject 낙하**, obs 28-D / action 7-D,
**success 100% · release 100% · 실착탄 0.356 m** (단, §7의 선택 편향·단일 시드 주의).

---

## 1. 트랙 지도

| 트랙 | 시뮬레이터/알고리즘 | 브랜치·오너 | 상태 |
|---|---|---|---|
| **A. Gazebo/PX4 + SAC** | Gazebo Harmonic + PX4 SITL, SAC(SB3), num_envs=1 | `jekyun` / `Isaac-JS` | ⏸️ 2026-07-05 중단 (드라이버 불일치 + v15 회귀 의심, [[research/rl_rules]] Rule 25/26) |
| **B. Isaac Lab base env** | Isaac Lab `DroneBombardEnv`, PPO(rsl_rl), 2048 envs | `isaac_jk` (제균) | ✅ 진행 — 커리큘럼/진단/물리 페이로드/이동타겟 |
| **C. Isaac Lab v-track** | 같은 스택, `DroneBombardV{11..20}Env` (base 상속) | `Issac_JS` → `isaac_jk` (준상) | ✅ 진행 — v19_precise가 현행 best, v20 fresh 재학습 등록됨 |

**B와 C는 같은 코드베이스의 서로 다른 env 클래스**다(obs 14/21-D·action 6-D vs obs 24~28-D·action 6/7-D).
→ **수치를 직접 비교하면 안 된다.** 두 트랙의 결론(구조 개입이 답)만 상호 보강 관계다.

---

## 2. 공통 도메인 정의 (논문 §Method 재료)

**CCIP 예측(무바람 nominal):** 릴리스 지연 $t_d$, 낙하시간 $t_f=\sqrt{2h/g}$일 때

$$\hat{p}_{imp} = p_{xy} + v_{xy}\,(t_f + t_d),\qquad d_{imp} = \lVert \hat{p}_{imp} - p_{target}\rVert$$

**학습 residual 보정(v14+):** $\;p_{corr} = \hat{p}_{imp} + a_{[5:7]}\cdot s_{res}$ ($s_{res}$ = 2~3 m)
— **보정값이 obs·릴리스 게이트·조준 shaping을 전부 구동**하고, **채점은 실제(drag/wind) 착탄**으로 한다.

**릴리스 엔벨로프 게이트(v11+):** $d_{imp}\le r_{rel}$ ∧ $h\in[3,8]$ ∧ $\lVert v_{xy}\rVert\le5$ ∧ $|v_z|\le3$ ∧ tilt $\le0.35$ ∧ $\lVert\omega\rVert\le4$ ∧ payload 부착.
게이트가 닫힌 상태의 투하 시도 = **no-op·무페널티**(Gazebo v7 교훈).

**정책 구조:** PPO(rsl_rl), 물리 100 Hz / decimation 10 → **정책 10 Hz**, 캐스케이드 속도
컨트롤러(velocity P → thrust → attitude P → rate P), Crazyflie articulation, env_spacing 16 m.

---

## 3. 계보 A — Gazebo/PX4 + SAC (2026-03 ~ 07-05)

주로 **인프라·처리량·평가 신뢰도** 문제를 푼 구간. 논문에서는 *"실기 스택에서의 제약과 왜 대규모
병렬 시뮬로 옮겼는가"*의 근거로 쓰기 좋다.

| 버전 | 추가/변경 | 결과·교훈 |
|---|---|---|
| ~v8 | 선형 거리보상, RTF 튜닝, 물리 폭발 3중 방어 | RTF=2 최적(59.5 fps), 지수 포텐셜 포화 금지(Rule 4·7) |
| v9~v11 | Vision(YOLO X-마커) obs 전환, **전방 카메라** | v11: 404 successes, best d_xy 0.68 m. 단 CRUISE 타임아웃 28% |
| v12 | arming fix + **정하방 카메라** | **success 0 — 종단 overshoot 트랩**(핸드오프 ~1 m에서 가드가 step 1부터 무장). Rule 10 |
| v13 | 종단 보상 재설계(overshoot 1.5→0.6, success 0.5→0.8, action 8/5→4/3) | 157.7 K에서 plateau, deterministic eval의 **유효 에피소드 3/3 성공**(나머지는 EKF 흡수 루프 — 정책 아닌 시작상태 결함, Rule 12). 이후 기준선 80%로 인용 |
| v14 | 10 m 순항 + **탐지 게이트**(conf 0.5, 200→300 px) + **soft reset** | 핸드오프 2.7→5.0 m(Rule 13), 리셋 처리량 **~3.9×**(Rule 14). 196.5 K eval **65%** — 실패 전부 final-approach stagnation |
| v15 | wobble 교정: 출력 LPF(α=0.4) + 근접 속도댐핑 + smoothness↑ | jerk RMS **−45%**. 그러나 310 K에서 **X마커 미도달 회귀 의심**(미확정) — 댐핑 반경 3.0 m > success 0.8 m가 정체 구간을 재타격(Rule 25). 학습 중단은 GPU 드라이버 업그레이드로 인한 **강제 중단**(Rule 26) |

**A 트랙에서 확정된 것:** ①근접 성공에 필요한 종단 기하/액션 스케일 규칙(Rule 10) ②탐지 문제는
탐지 파이프라인에서 푼다(Rule 13) ③리셋 병목은 EKF 교란 회피로 푼다(Rule 14) ④댐핑 반경은
success 반경보다 작아야 한다(Rule 25).

---

## 4. 계보 B — Isaac Lab base env (제균, `isaac_jk`)

obs 14-D(vision 채널 포함) / action 4→6-D, phase 커리큘럼(1: 접근 → 2: CCIP+residual+DR → 3: 이동타겟).

| # | 실험 | 추가/변경 | warm-start | 결과 |
|---|---|---|---|---|
| exp_012 | Isaac 이식 | v13/v15 상수 parity 이식, SAC→PPO, analytic vision + YOLO-eval 이원화 | — (fresh) | 코드만, `test_math` 29/29 |
| exp_013 | 첫 완주 PPO | 2048 envs × 1000 it (65.5 M steps) | fresh | **eval 36%**. 실패 3종 규명: 비전 거리감쇠 누락 → **상승 farming**(Rule 17), farmer(+225)>finisher(+121)(Rule 18a), noise_std 0.8→3.92(Rule 18b) |
| exp_014 | **plant 수정 3종** + 비전 거리감쇠 | 스폰타임 MassAPI authoring, 로터 재주입 제거, **inertia 대반전**(rate loop ~1300× 저토크였음) | fresh (구 ckpt 무효) | **eval 100.0% (202/202)**, d_xy_min 0.665 m. 대조군 A0′(감쇠 OFF) 96.5% → **지배 요인 = plant 일관성**(Rule 19) |
| exp_016 | 지표 의미론 수정 (eval-only) | 스크립티드 CCIP referee(≤0.2 m 래치), 지표 전용·보상 bit-identical | A2 ckpt | 4.59 m는 투하오차가 아니라 **성공 종단 잔여속도의 탄도 캐리**. 발화 시 0.137 m이나 **release_rate 6%** → **근접 성공 ≠ 릴리스 능력**(Rule 21) |
| exp_017 | **Stage A**: 밀집 조준 보상 | $w_{aim}(1-\tanh(e/s))$, w∈{1,2}, knee∈{0.5,1.0} | P1 6-dim 기준선 → 체인 | det release_rate 2.5→**5.5**→3.5% — **판정 (b) 정체**. 보상-단독 개입 실패(Rule 22) |
| exp_018 | **Stage B**: 릴리스=종단 | 근접 종단(d_xy≤0.8) → **릴리스 발화 종단**으로 교체. 보상은 Stage A v1 그대로 | exp_017 v1 | **det release_rate 100.00%**, drop err **0.125 m**. 학습 내 23→99.6% 단조 상승 = Stage A 하락의 정확한 반전 → **인과 확정**(Rule 23) |
| exp_015 | phase 커리큘럼 실학습 | `--phases 1,2,3` 오케스트레이터, 6-dim 고정 warm-start | 페이즈 체인 | P1만 수렴(1.00). P2 drop 4.66→2.91 m·success≈0, P3 lead best 0.071 m·success≈0. **+2000 it 연장도 0.8 m 미돌파**(P3는 회귀) → **예산 확대는 답이 아님**(Rule 20e/f) |
| exp_019 | 물리 페이로드 attach/detach | GPU-복제 PhysX는 per-env 조인트 불가 → **kinematic weld** | — (검증 전용) | 4/4 PASS: 부착 추적 **1.1 mm**, 측정 착탄 vs 해석 CCIP **\|Δ\| ≤ 0.021 m**(Rule 24) |
| exp_020 | 물리 페이로드로 학습 | `physical_payload=True`가 유일 델타(보상 bit-match) | **exp018_B0** | det **100.00%**, drop err 0.169 m. release_rate **첫 롤아웃부터 100%** → **물리 페이로드의 학습 비용 = 0** |
| exp_021 | 이동 타겟(CV/CT/CA)을 v-track에 포팅 | `V11Env._step_moving_target()`, obs 28-D **불변** | **v19_precise 사본** | det: cv **44.5%** / ct 33.8% / ca 16.5%. **리드 부재가 1차 병목**(released-but-miss > success) |

**B 트랙의 핵심 기여:** exp_016 → 017 → 018로 이어지는 **3단 인과 사슬**.
문제 정의(지표가 아니라 능력이 없다) → 반증(보상 shaping 단독은 실패) → 해법(종단 구조 교체 단독으로 5.5%→100%).
단일 변수 개입이고 반전이 단조라, **논문에서 가장 방어 가능한 실험**이다.

---

## 5. 계보 C — v-track 사다리 (준상): 각 v가 추가한 것

설계 원칙: **삭제 없이 토글로만 쌓는다**(하위 버전 무손상), **한 번에 한 축**, 각 단계 smoke → dry-run.
전부 `v11_env.py`의 cfg 상속 체인으로 구현: V11 → V12 → {V13, V14} → {V15, V17} → V18 → V19 → V20.

| ver | 추가 능력 (한 줄) | obs / act | 핵심 cfg | warm-start 출처 | 결과 |
|---|---|---|---|---|---|
| **v11** | 완화 baseline — 커리큘럼 제거, **고정** marker 20 m 정면, 4 m/s 순항 핸드오프, **정책 drop_signal + 릴리스 엔벨로프** | 24 / 6 | `marker_dist=20`, `cruise_speed=4`, `release_radius=1.0` | fresh | ✅ 100%, 착탄 **0.43 m**† (iter≈88 수렴) |
| **v12** | 랜덤 marker — (20,0) 중심 **반경 5 m disk 면적균일** | 24 / 6 | `marker_random=True` | fresh | ✅ 100%, ~0.72–0.75 m — **암기 아님, 일반화 확인** |
| **v13** | **부분관측** — blind 순항, 수평 7 m 진입 시에만 marker obs 공개(비래치) + 미탐지 −0.2/step + 보상 detection 게이팅 | **25** / 6 | `reveal_radius=7.0` | fresh | ✅ 100%, ~0.8 m (reward 290 < v11 314 = 미탐지 페널티 몫) |
| **v14** | **DR + CCIP residual (Stage A)** — 에피소드별 wind/drag, action[5:7] 잔차 ±3 m, **바람을 obs로 제공** | **27** / **7** | `v14_dr/residual`, `wind_std=1.0` | fresh | ✅ residual **ON 0.694 m / OFF 0.823 m**, success 81.5 vs 72.2% — **대조군 있는 인과 실험** |
| **v15** | **바람이 기체를 민다** — 상대기류 2차 항력 $F=k\lVert v_{air}\rVert v_{air}$ | 27 / 7 | `wind_force_enabled`, `wind_drag_k=0.06` | fresh | wind-test **OFF 0.01° vs ON 3.50°**(해석 4.02°와 근접) PASS. wind 4.0에선 드리프트 3.7–7.7 m > residual ±3 m → **포화**로 정체 → 2.0으로 하향(dry-run 미실행) |
| **v16** | **실제 물리 payload** — RigidObject 0.1 kg가 실제 낙하, **착지가 종료**, 실착지점으로 채점 | 24 / 6 | `payload_physics_enabled`, `drag_k=0.005` | fresh | drop-test PASS(\|dz\| 0.10 m), dry-run success 0.80 · sub-meter |
| **v17** | **픽셀 양자화 vision** — 인지 위치를 셀 중심으로 스냅, **cell = k·slant**(멀면 애매·가까우면 정밀) | 25 / 6 | `pixel_cell_k=0.15` | fresh | ✅ success 0.67–0.83, ~0.75 m |
| **v18** | **통합 #1: 인지(v17) + 물리(v14/v15)** | **28** / 7 | 아래 부트스트랩 표 | P1 fresh → **P2는 P1 model_200** | 🐞 **투하 데드락**(초기 랜덤 residual이 게이트 봉쇄, residual OFF 시 release 0→100%로 원인 확정) → **2단 커리큘럼**으로 해소: P1 완화 success 1.0/0.53 m → P2 hard success 1.0/~0.65 m |
| **v19** | **통합 #3: v18 + 실제 물리 drop(v16)** | 28 / 7 | `payload_physics_enabled` | **v18-P2 model_300** | 초기 release 100%(데드락 없음), ~iter 75 수렴, 실착탄 0.56–0.67 m. **그러나 iter499에서 붕괴(release 0%)** |
| **v19_abd** | **붕괴 수정 A+B+D** — (A) CCIP 조준항을 **포텐셜(차분)형**으로, (B) 인엔벨로프 미투하 **누진 페널티**, (D) best-ckpt 보존 | 28 / 7 | `ccip_potential_shaping`, `v19_w_loiter=0.02`, `save_interval 25` | **v18-P2 model_300** (iter 300→600) | ✅ 붕괴 방지(release 96–100% 유지), success **76.7%**, 0.563 m. **재적응 딥(iter350–450, release 0)** 관측 → 조기종료 금지 |
| **v19_precise** ⭐ | **연속 착지보상** — 평평한 성공존 보상 제거, $300e^{-2e_{land}}+100\cdot\mathbf{1}[e\le1]$ | 28 / 7 | `precise_landing_reward`, `k_landing=2.0`, `success_bonus=100` | **v19_abd best (iter599)** (599→899) | ✅ best iter875: **success 100% · release 100% · 0.356 m** (0.563 → 0.356, **37%↓**) |
| **v20** | **파이프라인 재수립 — v19_precise와 bit-identical**, 단 **fresh from scratch 단일 run** | 28 / 7 | 변경 없음(등록만) | **fresh (의도적)** | 🕐 2026-08-01 등록, 미학습. → §8-E 참고: 사실상 **"커리큘럼이 필요한가" ablation** |

† v11 착탄값은 노트 간 불일치가 있다: `v-exp_006` 본문 표는 ~0.75 m, 로드맵/`v-exp_007` 대조는 0.43 m.
논문 인용 전 원 로그로 확정할 것.

**v18/v19 부트스트랩 파라미터(데드락 해소용 완화 → hard 복원):**

| 파라미터 | Phase 1 (완화, 기본) | Phase 2 (`--v18_hard`) |
|---|---|---|
| `release_radius` | **1.5** (게이트 열림 확률 9%→44%) | 1.0 |
| `v14_residual_scale` | 2.0 | 3.0 |
| `v14_wind_std` | 1.5 | 2.0 |
| `pixel_cell_k` | 0.12 | 0.15 |

---

## 6. Warm-start 계보도 (체크포인트 혈통)

```
[C] v-track (준상)
  v11/v12/v13/v14/v15/v16/v17 ── 각각 fresh (능력 축 격리가 목적)
  v18-P1 (완화, fresh)
      └─ model_200 ─► v18-P2 (hard 복원)
                         └─ model_300 ─┬─► v19        (물리 drop)  → iter499 붕괴
                                       └─► v19_abd    (A+B+D)      → iter599 best (76.7% / 0.563 m)
                                                          └─ model_best ─► v19_precise (연속 착지보상)
                                                                              └─ iter875 model_best ⭐
                                                                                   ├─► [B] exp_021 mt_cv / mt_ct / mt_ca  (사본만 사용, 원본 무접촉)
                                                                                   └─► (v20 = 같은 스펙을 fresh로 재학습, 비교군)

[B] base env (제균)
  exp_013 (fresh) ─► [plant 수정으로 무효] ─► exp_014 A2 (fresh)
  exp_017 P1 기준선 (fresh, 6-dim)
      └─ v1 (w_aim=1) ─┬─► exp_017 v2 (w=2)          [회귀]
                       └─► exp_018 B0/B1/B2/B3        [종단 교체 → 100%]
                                └─ exp018_B0_final ─► exp_020 (물리 페이로드)  → Stage C 예정
  exp_015 P1 ─► P2 ─► P3 (오케스트레이터 체인)  ─► P2ext / P3ext (+2000 it 각)
```

**warm-start 규칙 (확립됨):**
- obs/action 차원이 같으면 `runner.load()`가 **무손실**. 그래서 커리큘럼은 **6-dim 고정**(Rule 20a),
  이동 타겟 포팅도 **obs 28-D 불변**으로 설계했다(exp_021).
- `--target_kf`(obs 14→21)는 **차원이 바뀌므로 warm-start 불가 → fresh 필수**.
- **보상을 바꾸면 재적응 딥이 온다**(v19_abd에서 release가 iter350–450 동안 0까지). **딥에서 조기종료 금지 +
  best-checkpoint 보존**이 대응책. 옛 v19의 iter375(0.56 m) 유실이 이 규칙의 계기.
- 이어학습 = `model_final`(옵티마이저·iter 포함) / 새 실험 시드 = `model_best`. → `checkpoints/v19/WARMSTART.md`

---

## 7. 지금까지 확정된 "발견" 목록 (논문 Claim 후보)

방어 가능성이 높은 순:

| # | 주장 | 증거 | 강도 |
|---|---|---|---|
| **F1** | **임무 이벤트(릴리스)는 dense shaping이 아니라 종단 구조로 학습된다.** 조기 성공 종단이 조준 구간을 잘라먹는 것이 지배 요인 | exp_017(보상 단독: 2.5→5.5→3.5%) vs exp_018(**종단만 교체**: →100%, 학습 내 단조 하락→단조 상승 반전). 동일 보상·동일 warm-start | ⭐⭐⭐ 단일변수·인과 반전 |
| **F2** | **상주(standing) shaping은 수렴 후 국소최적 붕괴를 만든다.** "완벽 조준 + 영원히 호버"가 노이즈 큰 실제 투하보다 이득 → 포텐셜(차분)형이 원천 차단 | v19 iter499 release 0%인데 aim_err med 0.35 m(=능력은 멀쩡) → A 적용 후 iter600까지 96–100% 유지 | ⭐⭐⭐ 실패 시그니처가 깔끔 |
| **F3** | **자동 발화 referee는 탐험 노이즈를 "그래디언트 평탄화기"에서 "성공 샘플러"로 전환한다.** 같은 노이즈(CCIP ×1.5 s 증폭)가 Stage A에선 방해, Stage B에선 발견 메커니즘 | exp_017 §원인분석 ↔ exp_018 (Rule 23b) | ⭐⭐⭐ 이론적으로 예쁜 대비 |
| **F4** | **능력 통합에는 부트스트랩 커리큘럼이 필요하다 — 학습된 잔차와 그 잔차가 통과해야 하는 게이트 사이의 닭-달걀 데드락** | v18: from-scratch 300 it release 0 → residual 강제 OFF 시 release 100%(원인 확정) → 게이트 1.0→1.5 완화 후 warm-start로 hard 복원 | ⭐⭐⭐ 원인 격리 실험 있음 |
| **F5** | **정밀도는 보상에 명시해야 나온다.** 성공존 내부 평평 보상은 착탄오차를 정체시킨다 | v19_precise: flat → 연속 지수+이산 보너스로 0.563→**0.356 m**(37%↓), success 76.7→100% | ⭐⭐ 단일변수지만 §7 선택편향 주의 |
| **F6** | **이벤트 조건부 지표는 그 이벤트를 시뮬레이트한 순간에 측정해야 한다.** "success 100% + drop_error 4.59 m"는 지표 의미론 버그였다 | exp_016: 4.59 m = 잔여속도 3.0 m/s × 1.53 s의 탄도 캐리로 정확히 재현 | ⭐⭐⭐ 방법론 기여 |
| **F7** | **근접 성공 ≠ 임무 능력.** d_xy 근접 보상 정책의 CCIP 최근접은 경로 cross-track 오차와 동차수 | aim_err_min med 0.755 m ≈ d_xy_min 0.665 m, release_rate 6%(10 Hz)/11.5%(100 Hz) → 샘플링 아닌 cross-track 지배 | ⭐⭐⭐ |
| **F8** | **GPU-복제 물리에서 per-env 결합/분리는 kinematic weld로 구현 가능하고, 해석 경로와 cm-parity를 갖는다** | exp_019: 추적 1.1 mm, 측정 vs 해석 \|Δ\| mean 0.012 / max 0.021 m. exp_020: **학습 비용 0**(첫 롤아웃부터 release 100%) | ⭐⭐ 시스템 기여 |
| **F9** | **잔차 권한은 보정 대상 드리프트를 덮어야 한다** — 안 그러면 포화로 성능이 plateau | v15 wind 4.0: 드리프트 3.7–7.7 m > ±3 m → 착탄 ~3 m 정체 (residual OFF는 release 0으로 붕괴) | ⭐⭐ 스케일 법칙 |
| **F10** | **관측 근사 모델은 실패 특성까지 이식해야 한다** — 성공 특성만 이식하면 착취 가능한 보상 지형이 생긴다 | exp_013: 거리감쇠 없는 analytic conf → **상승 farming**(max_alt 33%). 감쇠 추가 후 R_alt 0.0000 | ⭐⭐ |
| **F11** | **iter 예산 확대만으로는 이벤트 능력이 형성되지 않는다** | exp_015 §8: P2 +2000 it → 2.91→2.87 m 정체·release 0.33→0.01, P3 → 3.20→5.31 m **회귀** | ⭐⭐ 음성 결과(구조 개입 필요성 뒷받침) |
| **F12** | 이동 타겟에서 **릴리스 능력은 이월되지만 명중은 리드 부재로 무너진다** | exp_021: release 63–83% 유지, 그러나 released-miss > success, 착탄 med 0.78–1.06 ≈ $E[\lVert v\rVert]\cdot t_f$ 스케일 | ⭐⭐ 다음 기여의 출발점 |

### ⚠️ 현재 데이터의 방법론적 약점 (논문 제출 전 반드시 처리)

1. **거의 전부 단일 시드(seed 42).** 다중 시드 없이 성능 차이를 주장하는 표가 다수.
   exp_017이 이미 "n=200에서 p≈0.13"으로 스스로 한계를 밝혔다.
2. **`select_best_checkpoint.py`의 선택 편향.** v19_abd/precise의 대표 수치는 **15개 체크포인트를
   같은 eval 지표(150 ep 실착탄 success)로 훑어 고른 best**다 → 낙관 편향. **독립 held-out 재평가 필요.**
3. **eval 시드 미고정.** `play.py`에 seed 옵션이 없어 동일 ckpt cv 평가가 **32.0% ↔ 44.5%** (binomial σ≈3.5 pp를 크게 초과). 모델 간 비교는 동일 표본/대표본으로.
4. **v-track은 wandb 없이 로컬 tensorboard.** 재현 가능한 아티팩트 링크가 약하다.
5. **rule-based CCIP 베이스라인이 없다.** `baseline_ccip.py`는 [[research/phase1_plan]]에 설계만 있고 미실행 →
   **"RL이 왜 필요한가"를 정량으로 못 말한다.** (현재 가장 큰 구멍)
6. **v19_abd는 A·B·D를 한꺼번에 적용했다.** 붕괴 방지의 귀속이 A인지 B인지 분리 안 됨(노트는 "A가 핵심"으로 추정).
7. **B/C 두 트랙이 서로 다른 env** — 수치 비교 불가. 논문에서는 한쪽을 본선으로 정하고 다른 쪽은 방법론 부록으로.

---

## 8. 논문화 전략 — 차별점과 ablation 설계

### 8.1 차별점(Contribution) 후보 — 우선순위

1. **"Classical guidance + minimal learning" 분해.** CCIP가 조준을 담당하고 RL은 **릴리스 타이밍 +
   2-D 잔차**만 학습한다. 학습 대상이 저차원·해석 가능하고, 잔차 권한(±s)이 **명시적 안전 경계**가 된다.
   → 순수 end-to-end 대비 "무엇을 학습에 맡겼는가"가 분명한 것이 세일즈 포인트.
2. **이벤트 능력의 구조적 학습(F1+F3).** "릴리스를 종단 이벤트로 만들고 자동 발화 referee를 두면,
   조준 노이즈가 방해에서 발견 메커니즘으로 바뀐다"는 주장은 **드론 밖에서도 일반화**된다
   (희소 이벤트 스킬 일반). exp_017/018이 단일 변수 반전이라 근거가 강하다.
3. **Shaping 병리 카탈로그 + 진단 시그니처.** 상주 shaping 붕괴(F2), farmer-vs-finisher 수지(Rule 18a),
   평평 보상 정체(F5), 지표 의미론 버그(F6)를 **증상→계측→처방** 형식으로 정리하면 실용 가치가 크다.
   각각 실패 로그와 반전 실험이 있다.
4. **능력 사다리 + 토글 아키텍처(v11→v20).** 모든 확장이 cfg 토글이고 하위 버전이 무손상이라
   **어떤 조합도 재현 가능**하다 — ablation 인프라 자체가 기여가 된다.
5. **GPU-병렬 물리에서의 attach/detach + parity 계측(F8).** 시스템 논문 각으로도 쓸 수 있다.

### 8.2 이미 확보한 ablation (추가 학습 없이 표로 만들 수 있음)

| 축 | 조건 | 근거 |
|---|---|---|
| 종단 구조 | 근접 종단 vs 릴리스 종단 (보상 동일) | exp_017 v1 vs exp_018 B0 |
| 조준 shaping 강도 | $w_{aim}$ ∈ {0, 1.0, 1.5}, knee ∈ {0.5, 0.75} | exp_018 B0/B1/B2/B3 (전부 ~100% → **종단 구조에서 shaping은 잉여**) |
| CCIP residual | ON vs OFF (DR·obs·action 동일) | v-exp_009 (0.694 vs 0.823 m) |
| 바람 작용점 | payload만 vs payload+기체 | v14 vs v15 (+ wind-test 0.01° vs 3.50°) |
| residual 권한 | scale 2.0 / 3.0, wind_std 1.5 / 2.0 / 4.0 | v18 부트스트랩 + v15 포화 |
| 인지 난이도 | 정확 obs(v12) / reveal(v13) / 픽셀양자화(v17, k=0.12·0.15) | v-exp_007/008/012 |
| 투하 모델 | analytic(v18) vs 물리 RigidObject(v19) / exp_018 vs exp_020 | 두 트랙 모두 있음 |
| shaping 형태 | 상주 vs 포텐셜(차분) | v19 vs v19_abd |
| 착지보상 형태 | 평평 vs 연속 지수+보너스 | v19_abd vs v19_precise |
| 비전 거리감쇠 | ON(A2) vs OFF(A0′) | exp_014 |
| 학습 예산 | 500 it vs +2000 it (P2/P3) | exp_015 §7 vs §8 (음성) |
| 타겟 모션 | 정지 vs CV vs CT vs CA | exp_021 (44.5 / 33.8 / 16.5%) |

### 8.3 추가로 돌려야 하는 실험 (우선순위)

| # | 실험 | 왜 필요한가 | 비용 |
|---|---|---|---|
| **N1** | **rule-based CCIP 베이스라인** — 스크립티드 접근 + 게이트 발화, 학습 없음. v19_precise와 동일 env·동일 200-ep 프로토콜 | **"RL이 왜 필요한가"의 유일한 정량 답.** 지금 논문에서 가장 큰 구멍 | 낮음 (env 재사용, 학습 없음) |
| **N2** | **다중 시드(≥3) + 고정 시드 held-out eval** — 헤드라인 구성(v19_precise, exp_018 B0, exp_021 cv)에 대해 | §7의 약점 1·2·3을 한 번에 해소. `play.py --seed` 추가 선행 | 중 (재학습 3×) |
| **N3** | **리드(lead) 개입 3분기 A/B** — (a) 리드 없음(현행) / (b) privileged target-vel obs 2-D(28→30) / (c) Singer-KF obs 포팅(28→35) / (d) `w_lead` 보상 | exp_021이 지목한 1차 병목. **"구조 개입 vs 관측 개입 vs 보상 개입"을 한 표로** — Rule 22의 재검증이기도 함 | 중 (b는 warm-start 가능, c는 fresh) |
| **N4** | **v20 (fresh 단일 run) vs v18→v19 warm-start 체인** | 이미 등록됨. **"커리큘럼이 필요한가"의 직접 ablation** — F4를 뒤집거나 확증 | 낮음 (등록 완료, 학습만) |
| **N5** | **v19_abd의 A/B 분리** — A만 / B만 / A+B | 붕괴 방지의 귀속. 현재는 번들이라 인과 주장 약함 | 중 |
| **N6** | **Stage B wind trap** — 바람 obs 제거 후 운동 이력에서 추정(frame-stack 또는 RNN) | "잔차가 관측 가능한 편향만 보정한다"는 주장의 완결. 로드맵에 설계돼 있으나 미구현 | 높음 |
| **N7** | **실제 vision(핀홀 u,v,conf → YOLO)** — 픽셀 양자화는 프록시 | sim-to-real 서사에 필수. 단 TiledCamera/annotator 이슈 선행 해결 필요 | 높음 |

**논문 1편으로 묶는 최소 세트: N1 + N2 + N4.** (베이스라인 대비 + 통계적 신뢰 + 커리큘럼 필요성)
**두 번째 기여를 넣는다면: N3** — 이동 타겟 리드는 아직 아무도 안 푼 축이고, 이미 F1(구조 vs 보상)
프레임을 그대로 재사용할 수 있어 서사가 이어진다.

### 8.4 논문 구조 제안 (초안)

1. **Introduction** — 정밀 투하 = 희소 이벤트 + 모델 오차. 순수 end-to-end도, 순수 고전 유도도 아닌 분해.
2. **Method** — CCIP 조준 프리미티브 / 릴리스 엔벨로프 게이트 / 학습 잔차 / **릴리스=종단 이벤트** 구조.
3. **Capability ladder & toggle architecture** — v11→v19, 축별 격리 원칙.
4. **Experiments**
   - 4.1 베이스라인 대비(N1) + 최종 성능(N2 다중시드)
   - 4.2 **핵심 ablation: 종단 구조 vs dense shaping**(F1/F3)
   - 4.3 shaping 병리: 상주→붕괴, 평평→정체(F2/F5)
   - 4.4 통합 데드락과 부트스트랩 커리큘럼(F4) + N4
   - 4.5 residual/바람/인지 축 ablation (8.2 표)
   - 4.6 이동 타겟과 리드(F12 + N3)
5. **Discussion — 진단 시그니처 카탈로그**(측정→해석→처방), 한계(sim-only, 시드, 선택 편향).

---

## 9. 남은 축 / 다음 액션 (기술 로드맵)

| 축 | 현재 | 다음 |
|---|---|---|
| 인지 | 픽셀 양자화(프록시) | 핀홀 카메라(u,v,conf) → 실제 YOLO → sim-to-real |
| 탄도/물리 | 정상풍 + drag DR + residual | 시변 바람·돌풍, payload 질량 랜덤화, 지형 고도≠0, 사출 임펄스, 고도 추정 노이즈(오차 하한) |
| 타겟 | 정지 / CV·CT·CA(리드 없음) | **리드 학습(N3)**, 기동·회피 타겟, 다중 타겟 선택 |
| 시나리오 | 고정 +X 4 m/s 핸드오프 | 랜덤 핸드오프(방향·속도·자세·고도), reveal 반경↓ |
| 통합 | v19_precise (최고) / v20 fresh 대기 | release_radius 1.5→1.0 커리큘럼, 다중시드 견고화 |
| Sim-to-real | 없음 | 도메인갭·지연·PX4/실기 |

**즉시 할 일 (문서·인프라):**
- [x] `*_junsang.md` 34개를 archive 태그에서 `main`으로 복원 (2026-08-02 완료)
- [ ] `play.py --seed` 추가 (§7 약점 3) — N2의 전제
- [ ] `exp_NNN` 번호 충돌 정리 또는 접두사 규칙 확정 (§0 이슈 2)
- [ ] v11 착탄값(0.43 vs 0.75 m) 원 로그로 확정

---

## 관련 노트

- 허브: [[00_index]] · [[research/rl_rules]] · [[experiments/training_history]]
- 트랙 B 핵심: [[experiments/exp_016_ccip_release_reeval]] · [[experiments/exp_017_stageA_aim_reward]] ·
  [[experiments/exp_018_release_terminal]] · [[experiments/exp_015_phased_curriculum]] ·
  [[experiments/exp_019_physical_payload]] · [[experiments/exp_020_o5jn9xzk_payload_training]] ·
  [[experiments/exp_021_v19_moving_target]]
- 연구: [[research/ccip_release_decoupling]] · [[research/ccip_aim_reward_stageA]] ·
  [[research/release_terminal_stageB]] · [[research/curriculum_phase_convergence]] ·
  [[research/physical_payload_attach]] · [[research/moving_target_models]] · [[research/phase1_plan]]
- 트랙 C 1차 사료(현재 archive 태그에만 존재): `notes/research/isaac_model_spec_junsang.md`,
  `isaac_expansion_roadmap_junsang.md`, `isaac_v19_collapse_nodrop_junsang.md`,
  `isaac_v18_curriculum_continuation_junsang.md`, `notes/experiments/exp_006~016_*_junsang.md`
- 코드: `isaac_lab/drone_bombard/v11_env.py` (v11~v20 cfg 체인), `drone_bombard_env.py` (base),
  `checkpoints/v19/WARMSTART.md`, `REVIEW_GUI.md`
