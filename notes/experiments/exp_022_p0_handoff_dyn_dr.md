---
date: 2026-08-03
tags: [experiment, isaac-lab, v20, p0, domain-randomization, handoff, generalization, ablation]
status: done
type: experiment
wandb_run: "none (tensorboard dry-run + deterministic evals)"
---

# exp_022 — P0: 핸드오프 랜덤화 + 동역학/센싱 DR (v20 env 신설)

> [[research/paper_research_plan]] §5 **P0-3 / P0-4 / P0-5(일부)** 실행.
> 목표: "v-track의 핸드오프가 완전 고정이라 일반화 주장이 불가능하다"는 방법론 약점(§2c-7)을
> env 레벨에서 제거하고, 논문 표가 요구하는 DR 축(질량/게인/탄도계수/관측·행동 노이즈)을 추가.
> **학습이 아니라 env 변경 + 게이트 검증 + 기존 정책의 분포 전이 측정**이 이번 실험의 범위.

## 1. 무엇을 바꿨나

### 1a. `DroneBombardHandoffCfg` (v11_env.py) — 핸드오프 랜덤화

v11~v19는 **매 에피소드 동일한 초기조건**을 준다: env 원점, 정확히 10 m, 정확히 +X로 4 m/s,
수평 자세, 각속도 0. 움직이는 것은 마커뿐(5 m 디스크).

| 축 | v11~v19 | v20 |
|---|---|---|
| 순항 방위 | +X 고정 | U[−180°, 180°] |
| 순항 속도 | 4.0 m/s | U[2, 6] m/s |
| 스폰 고도 | 10.0 m | U[8, 12] m |
| 진입 오프셋 | 없음 | 횡 ±3 m · 종 ±2 m (순항 프레임) |
| 속도 노이즈 | 없음 | N(0, 0.3) m/s ×3축 |
| 자세 | 수평 | roll/pitch N(0, 5°) |
| 각속도 | 0 | N(0, 0.2) rad/s |

마커는 **각 env 자신의 방위**로 `marker_dist` 앞에 놓이므로 공칭 기하는 보존된다.

### 1b. `DroneBombardDynDRCfg` (drone_bombard_env.py) — 동역학/센싱 DR

**설계 원칙: 런타임 PhysX 프로퍼티를 절대 건드리지 않는다**(Rule 19). 두 개의 등가성으로 우회:

- **컨트롤러 질량 *신념*을 랜덤화**(플랜트는 spawn authored 그대로). 참질량 $m$, 신념 $m(1+\delta)$일 때
  폐루프는 $a = (1+\delta)a_{des} + \delta g$ — 고정 신념으로 미지 질량을 나는 것과 형태가 동일.
- **페이로드 탄도계수(drag_k)를 랜덤화 ≡ 페이로드 질량 랜덤화**. 자유낙하는
  $\dot v = g + (k/m)\lVert v_{air}\rVert v_{air}$ 이므로 궤적은 $k/m$ 하나에만 의존.
  (단위테스트 `test_ballistic_coefficient_scaling_equals_mass_scaling`로 수치 검증)

| 축 | 범위 |
|---|---|
| 컨트롤러 질량 신념 | ±5% |
| 속도 P 게인 | ±10% |
| 자세 P + rate P 게인 | ±10% |
| 페이로드 탄도계수(=질량) | ±20% |
| 관측 노이즈 | per-step N(0, 0.01) + **per-episode 상수 bias** N(0, 0.001) |
| 행동 노이즈 | per-step N(0, 0.025) + **per-episode 상수 bias** N(0, 0.02) |

두-시간척도 노이즈 값은 *Learning to Throw* 그대로. **bias 항이 핵심** — 백색잡음은 에피소드 내에서
평균 0으로 씻겨 정책이 무시할 수 있지만, 에피소드 상수 bias는 캘리브레이션 오프셋 강건성을 강제한다.
행동 노이즈는 **속도 채널 [0:4]에만** 넣는다(드롭 신호·CCIP 잔차에 넣으면 논문이 측정하려는 두 메커니즘이 오염됨).

### 1c. 기타

- `Episode_Metric/deliver_time_s` 신설 — 에피소드 시작 → 투하 채점 이벤트까지의 시간
  (= *throw duration* 대응). 두 투하 메커니즘 모두 그 이벤트에서 종료하므로 **추가 상태 없이**
  종단 스텝 수로 계산. 호버-드롭 퇴화를 드러내는 유일한 지표.
- `--v20` / `--no_handoff_dr` / `--no_dyn_dr` (train.py), `--no_handoff_dr` / `--no_dyn_dr` /
  `--handoff_heading_deg` (play.py) — train/test 분포 불일치 프로토콜용.
- `Isaac-DroneBombard-V20-Direct-v0` 등록. **env 클래스는 `DroneBombardV19Env` 그대로**
  (obs 28-D / action 7-D 불변 → v19 체크포인트 무손실 warm-start).

## 2. 사전 검증 (게이트)

- **유닛테스트 66/66 PASS** (기존 57 + 신설 `tests/test_domain_rand.py` 9).
  핵심 성질: **OFF 경로는 identity이면서 RNG를 소비하지 않는다** — 안 그러면 P0 노브를 켜는 것만으로
  모든 이전 버전의 난수 스트림이 밀린다.
- **`_probe_p0_dr.py --variant v19`: 12/12 PASS** — 스폰 xy 오차 `0.00e+00`, 고도 `[10.0000, 10.0000]`,
  속도 `[4.0000, 4.0000]`, yaw/자세/각속도 정확히 0, DR 버퍼 전부 identity.
  → **v11~v19 무손상 실증.**
- **`--variant v20`: 15/15 PASS** — yaw std 1.60 rad(범위 [−3.12, 2.63]), 속도 [1.76, 5.85],
  고도 [8.03, 11.97], 진입 오프셋 max 3.38 m(예산 3.61), roll std 5.27°, 각속도 std 0.197 rad/s,
  **마커 방위 오차 max 22.2°(각 env 자기 방위 기준)**, 접근거리 [13.8, 26.6] m,
  질량신념/게인/탄도계수 전부 밴드 내, obs 28-D 유한, v19 대비 롤아웃 obs 차이 1.9976(DR이 실제로 정책까지 도달).
- **dry-run 3 iters (64 envs, v19 precise warm-start)**: `[INFO] Warm-starting from checkpoint` 성공
  (차원 불일치 없음), 3 iter 완주, `model_final.pt` 저장, NaN/에러 0, `deliver_time_s` 로깅 확인.

> ⚠️ 두 env를 한 프로세스에서 연달아 생성하면 두 번째 씬 빌드에서 멈춘다 → 프로브는
> **변이당 1 프로세스**로 돌리고 v19 롤아웃 트레이스를 파일로 넘긴다.
> `simulation_app.close()` 행 현상은 여전 → 로그 마커 확인 후 kill (기존 절차).

## 3. 판정 — v19 정책의 분포 전이 (deterministic, 200 ep, seed 42, num_envs 64)

같은 체크포인트(`v19_warmstart/precise/model_best.pt`), 같은 표본, 조건만 다름:

| # | 평가 조건 | success | release | drop err @release (med) | d_xy_min (med) | 지배 실패 |
|---|---|---|---|---|---|---|
| A | v19 (고정 핸드오프, DR 없음) — 기준 | **91.0%** | 100% | 0.364 m | 1.055 m | timeout 17 |
| B | + 동역학/센싱 DR만 (핸드오프 고정) | **91.5%** | 98.5% | 0.332 m | 1.064 m | timeout 13 · crash 3 |
| C | + 핸드오프 DR (**방위는 +X 고정**) | **77.1%** | 91.0% | 0.403 m | 1.124 m | bad_att 16 · timeout 19 |
| D | + 전체 v20 (방위 ±180°) | **7.5%** | 12.5% | 0.318 m | **16.78 m** | **out_of_range 108(54%)** · bad_att 43 |

**세 줄 결론:**

1. **동역학/센싱 DR은 사실상 공짜다** (A→B: 91.0 → 91.5%, 노이즈 범위 내).
   질량신념 ±5% / 게인 ±10% / 탄도계수 ±20% / 두-시간척도 노이즈를 한꺼번에 켜도 기존 정책이 버틴다
   → **커리큘럼 없이 바로 채택 가능**. 이 축의 난이도는 재학습으로 흡수될 여지가 크다.
2. **속도·고도·오프셋·자세 랜덤화는 회복 가능한 격차다** (B→C: −14.4 pp). p90 d_xy_min은 여전히 2.05 m.
3. **월드프레임 방위 랜덤화는 파괴적이다** (C→D: 77.1 → 7.5%, OOR 54%, d_xy_min med 16.8 m).
   정책이 표적에 **접근조차 하지 않는다** — "+X로 날아라"를 외운 것.

**투하 스킬 자체는 모든 조건에서 살아 있다**: 발화만 하면 착탄 중앙값은 0.32–0.40 m로 일정.
무너진 것은 조준/투하가 아니라 **회전된 프레임에서의 접근**이다.

## 4. 해석 — 왜 방위만 이렇게 치명적인가

v-track obs는 **월드프레임**이다: `rel_x/rel_y`, 속도, `ccip_err`, wind 전부 월드 좌표.
방위를 고정해 두면 "표적은 항상 +X 쪽" 이 관측 분포에 상수로 박히고, 정책은 그 상수를 흡수한다.
방위를 열면 같은 과제가 모든 회전에 대해 반복되는데, 이는 원리적으로는 회전 등변(equivariant) 문제라
학습 가능하지만 **표현을 처음부터 다시 만들어야** 한다(그래서 warm-start가 무의미해짐).

→ 이는 **일반화 실패가 아니라 관측 프레임 설계 문제**일 가능성이 높다.
계획서에 미리 적어둔 대로 관측 프레임 변경은 obs 레이아웃 변경 = warm-start 파기이므로 **P0 범위 밖**이다.
다음 갈림길은 §5.

## 5. 다음 결정 (P1 진입 전, 사용자 판단 필요)

| 안 | 내용 | 비용 | 리스크 |
|---|---|---|---|
| **(a)** v20 그대로 학습 | 방위 포함 전체 DR에서 fresh 또는 warm-start 학습 | 중 | 회전 일반화를 MLP가 표본으로 때워야 함 — 수렴 실패 시 방위 커리큘럼(±30°→±180°) 필요 |
| **(b)** 관측 프레임 불변화 | rel/vel/ccip를 **순항 방위 기준 body/bearing 프레임**으로 회전 | 중 | obs 의미 변경 → 기존 체크포인트 전부 warm-start 불가(fresh 필수). 대신 문제가 원천 소거되고 논문에서 "왜 이 프레임인가"가 하나의 ablation 행이 됨 |
| **(c)** 두 개를 ablation으로 | (a)와 (b)를 같은 예산으로 비교 | 높음 | 가장 논문 친화적 — **"고정 초기조건 → 표현 암기"를 정량화한 표가 나온다** |

권고: **(c)**, 단 순서는 (a) 먼저(현행 자산 활용) → 수렴 곡선을 보고 (b) 투입.
어느 쪽이든 §3의 A/B/C/D 4행은 그대로 논문 Table(초기조건 분포의 효과)로 쓸 수 있다.

## 6. 부기 — 기준선 수치 불일치

동일 `model_best.pt`가 준상 노트에서는 success 100%(150 ep, 자체 프로토콜)인데
여기 고정 시드 200-ep 평가에서는 **91.0%**로 나온다. 표본·프로토콜(성공 판정 반경, 에피소드 수,
시드 고정 여부)이 다르므로 이것만으로 선택편향을 단정하지 않는다 —
[[research/paper_research_plan]] §2c-2의 **held-out 재평가(P1-8)에서 정리할 항목**으로 등록.

## 7. 산출물

- 코드: `isaac_lab/drone_bombard/{mdp/domain_rand.py, drone_bombard_env.py, v11_env.py, __init__.py}`,
  `isaac_lab/{train.py, play.py}`
- 테스트: `isaac_lab/tests/test_domain_rand.py` (9), 게이트 프로브 `isaac_lab/_probe_p0_dr.py`
- dry-run ckpt: 컨테이너 `/workspace/logs/p0_dryrun/drone_bombard_ppo/2026-08-03_01-30-54_v20/`

## 관련

- [[research/paper_research_plan]] — P0 정의, Table 5(충실도 사다리)의 DR 축
- [[research/handoff_generalization_p0]] — 본 실험의 발견 노트 (Rule 27)
- [[research/research_overview_for_paper]] — §7 방법론 약점 7번
- [[experiments/training_history]] · [[research/rl_rules]]
