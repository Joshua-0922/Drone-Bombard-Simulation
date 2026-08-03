---
date: 2026-08-02
tags: [research, paper, plan, ablation, baseline, ccip, residual, vision, moving-target, sim2real]
status: active
type: research
---

# 논문 연구 계획 — Vision-conditioned CCIP-Residual RL for Precision Free-Fall Payload Delivery

> **이 문서의 역할:** [[research/research_overview_for_paper]]가 *"지금까지 무엇을 했는가"*의 사료라면,
> 이 문서는 *"논문 한 편을 위해 앞으로 무엇을 어떻게 바꿔서 무엇을 학습시키고 무엇을 비교하는가"*의 설계도다.
> 시행착오(인프라·드라이버·arming·EKF·컨테이너)는 여기서 **전부 버린다**(§2b).
> 문헌 조사(105편 + 2편 정독) 근거는 §1. 허브: [[00_index]] · [[research/research_overview_for_paper]]

---

## 0. 한 문단 포지셔닝 (논문 abstract의 씨앗)

멀티로터가 **순항 중에** 자유낙하 페이로드를 투하해 지상 표적을 맞춘다. 조준은 고전
**CCIP**(Continuously Computed Impact Point) 탄도 예측이 담당하고, RL은 **(a) 릴리스 시점**과
**(b) 예측 착탄점에 더해지는 유계(bounded) 2-D 잔차**만 학습한다. 표적의 절대좌표는 정책에
주어지지 않으며, 정책은 **탐지(detection) 형태의 부분관측**(reveal 반경, slant-range 비례 픽셀
양자화, confidence 거리감쇠)만 본다. 도메인 랜덤화된 바람·항력 아래 실제 착탄으로 채점하며,
정지·이동(CV/CT/CA) 표적을 모두 다룬다.

**세 문장으로 요약한 차별점:**
1. 잔차를 **제어 명령이 아니라 유도(guidance) 예측량**에 놓는다 — 학습 대상이 2-D·해석 가능·유계.
2. 표적 **절대좌표를 복원하지 않고** 탐지형 관측에 직접 조건화된 **학습된** 투하 정책이다.
3. 희소 임무 이벤트(릴리스)는 dense shaping이 아니라 **종단 구조**로 학습된다는 것을 통제 실험으로 귀속한다.

---

## 1. 문헌 지도 — "RL + CCIP로 드론이 물체를 투하하는" 연구는 어디까지 왔나

105편 스윕 + `Learning to Throw` / `AeroThrow` 정독 결과. **문제 공간이 두 진영으로 갈라져 있고,
우리 좌표(멀티로터 · 자유낙하 · 순항 릴리스 · 학습 · 탐지 조건화 · 바람 DR · 이동표적)는 비어 있다.**

### 1.1 진영 A — 케이블/암 기반 "던지기" (고가시성 연구가 전부 여기 있음)

| 논문 | 연도 | 방식 | 표적 | 핵심 |
|---|---|---|---|---|
| Foehn et al., *Fast Traj. Opt. w/ Cable-Suspended Payload* (RSS) | 2017 | TrajOpt | known | 릴리스 지점을 **최적화 변수**로 둔 원조 |
| Tang & Kumar, MIQP cable throwing | 2015 | MIQP | known | 하이브리드 모드 던지기 |
| Panetsos et al., NMPC cable release (ICRA) | 2024 | NMPC+UKF+Bézier | **vision, moving** | 하방 카메라로 **이동** 표적 추정 후 릴리스 — 고전 진영의 최근접 |
| Cao et al., time-optimal throw | 2025 | TrajOpt | known | LtT의 베이스라인 |
| **AeroThrow** (Li, Lyu et al.) | 2025 | MINCO+NMPC+NDOB+INDI | known(mocap) | 델타암 하드웨어로 추적오차 보상, **연속 착탄점 제약 → 릴리스 "창"**, **온라인 릴리스 시점 재평가(Alg.1)** |
| **Learning to Throw** (Zhai, Scaramuzza et al.) | 2026 | **RL(PPO)** | known(+vision 변형) | Warp 해석모델 ⊗ PhysX 로프의 **하이브리드 HIL-급 시뮬**, zero-shot 실기, TO+MPC 대비 착탄 −50%·시간 −30% |
| AD-Planner | 2026 | planning | moving | 이동 표적 casting (비학습) |
| FLARE | 2025 | RL | — | 서스펜디드 페이로드 **비행만**, 릴리스 없음 |

### 1.2 진영 B — 자유낙하 에어드롭 (훨씬 얇고 덜 엄밀)

| 논문 | 연도 | 방식 | 표적 | 핵심 / 우리와의 관계 |
|---|---|---|---|---|
| Mathisen & Johansen, *Approach methods for precision aerial drop* | 2017 | 고전 CARP/FARP | known | **자유낙하 릴리스점 재최적화**, FARP = 우리 릴리스 엔벨로프의 고전 조상 |
| Mathisen et al., *Autonomous ballistic airdrop … machine vision* | 2020 | 고전 + 머신비전 | **vision** | ⚠️ **N-b의 반례** — 탐지→**절대좌표 복원**→CARP. 고정익, 착탄 5.5 m |
| Li et al. (Sensors) | 2021 | RL(DDPG) | known | **접근 기동만** 학습, 릴리스 트리거 없음 |
| **APER-DDQN** (Ouyang et al.) | 2022 | **RL(Double DQN)** | known | **릴리스 결정을 이산 행동으로 학습한 유일한 선행** — success ~41% |
| Vadduri et al., *Precise Payload Delivery … Object Detection* | 2023 | 탐지 + 정렬 | **vision** | ⚠️ **N-b의 반례** — 탐지 후 **표적 위 호버→정지 투하**(비학습, 리드 없음) |
| CCRP + fuzzy RL 폭격원 | 2022 | RL(fuzzy) | known | **CCIP/CCRP와 RL을 결합한 유일 선행**(저품질 venue) |
| PSO/DE 기반 릴리스점 최적화 | 2022/2026 | 메타휴리스틱 | known | 2026년까지도 "바람 하 릴리스점"은 최적화 문제로 다뤄짐 |
| 군용 정밀공중투하(CARP/JPADS, Yakimenko 등) | 2005~ | 고전 | known | 바람 프로파일 동화 + 분산 최소화 CARP — **우리 wind-oracle 베이스라인의 근거** |

### 1.3 잔차(residual)를 **어디에** 두는가 — N-a의 정확한 좌표

- **제어 명령 레벨(정론):** Silver 2018 *Residual Policy Learning*, Johannink 2019 *Residual RL for Robot Control* →
  이후 삽입(Schoettler 2020), 데모 기반(Alakuijala 2021, Ankile 2025), 내비(Rana 2020), 레이싱(ResRace 2022) 전부 **행동/제어**에 잔차.
- **가장 높이 올라간 선행:** Davchev 2022 *rLfD* — DMP가 만든 **task-space 명령 궤적**에 잔차. 여전히 "명령"이지 "예측"이 아님. → **N-a 방어의 최우선 인용**.
- **동역학 모델 잔차:** NeuroBEM, NeuralFly, KNODE-MPC — 잔차가 **모델**에 붙음(우리와 다른 축).
- **유도 법칙 잔차(가장 가까운 이웃):** PN/biased-PN에 RL 바이어스·이득 증분(Xu Wang 2023, Zhao 2025 TAES, Zhu 2025), ZEM/ZEV 계수 적응(Furfaro 2020),
  **탄착점 예측 + 유도 파라미터 보정**(Xian 2022), 포탄 탄착점 예측 신경망(Wang 2019, Wu 2022).
  → **"고전 유도량에 학습 보정을 더한다"는 패턴 자체는 이미 존재한다. 인정하고 인용해야 한다.**
- **비어 있는 칸:** 위 전부가 **추력·공력으로 계속 조종하는 유도탄/포탄**이고 표적 좌표가 주어진다.
  **비추력 자유낙하 페이로드의 단발 릴리스 + 탄착점 예측 잔차 + 탐지 조건화**는 없음.
- **TossingBot(2019):** "탄도 해석해 위의 residual physics"의 원조지만 잔차가 **릴리스 속도(제어 파라미터)** → 우리는 **예측된 착탄점**.

### 1.4 이동 표적 / 리드

- RL 이동 플랫폼 착륙(Rodriguez-Ramos 2018, Goldschmid 2023/24, Cao meta-RL 2024, Shin 2026 RA-L vision-only)이 가장 잘 연구된 유사 문제.
  **관측은 항상 상대 상태 또는 이미지 특징**이며 리드는 (i) 이력에서 암묵 학습 (ii) 플랫폼 운동학 모델 주입 중 하나.
- 유도 문헌: Gaudet 2019/20(**각도만 관측으로 리드 학습 — 완전정보 ZEM과 동급**), Li 2021(보조 지도 헤드로 표적 가속 예측),
  Choi 2026·Wang 2025(**예측→PIP→정지 문제로 환원** = 우리 CCIP+리드 분해의 구조적 선례).
- **던지기+이동 표적:** *Throwing Objects into A Moving Basket* (2022, 지상 매니퓰레이터)가 최근접.

### 1.5 학습 결론 — 우리가 가진 칸

> **자유낙하 · 멀티로터 · 순항 중 릴리스 · 학습된 트리거 · 유도레벨 잔차 · 탐지 조건화(좌표 비복원) · 바람/항력 DR · 이동표적**
> 이 8개 속성을 **동시에** 만족하는 선행은 없다. 개별 속성은 대부분 선행이 있으므로,
> **주장은 "조합 + 메커니즘 귀속"으로 좁혀서 방어한다.**

---

## 2. 지금까지의 성과 중 논문에 쓸 것 / 버릴 것

### 2a. 살릴 것 (근거·재현 가능·단일 변수)

| ID | 주장 | 증거 | 논문 위치 |
|---|---|---|---|
| **F1** | 임무 이벤트는 dense shaping이 아니라 **종단 구조**로 학습된다 | exp_017(보상 단독 2.5→5.5→3.5%) vs exp_018(**종단만 교체** →100%) | 핵심 ablation §4.2 |
| **F3** | 자동 발화 referee는 탐험 노이즈를 **성공 샘플러**로 전환 | 같은 CCIP 노이즈가 Stage A 방해 / Stage B 발견 | §4.2 해석 |
| **F2** | 상주(standing) shaping → 수렴 후 붕괴("완벽 조준 + 영원 호버") | v19 iter499 release 0% (aim 0.35 m로 능력은 정상) → 포텐셜형으로 차단 | shaping 병리 §4.3 |
| **F5** | 정밀도는 **보상에 명시**해야 나온다(평평 보상 = 정체) | 0.563 → 0.356 m (37%↓) | §4.3 |
| **F6** | 이벤트 조건부 지표는 그 이벤트를 시뮬레이트한 순간에 측정 | 4.59 m = 잔여속도 3.0 m/s × 1.53 s 탄도 캐리로 정확히 재현 | 방법론 §3 / 부록 |
| **F7** | 근접 성공 ≠ 임무 능력 (cross-track 지배) | aim_min 0.755 ≈ d_xy_min 0.665, release 6% | §4.1 동기 |
| **F4** | 능력 통합의 **닭-달걀 데드락**(잔차 ↔ 게이트) → 부트스트랩 커리큘럼 | v18 release 0 → residual OFF 시 100%로 원인 격리 | §4.4 |
| **F9** | 잔차 권한은 보정 대상 드리프트를 **덮어야** 한다(안 그러면 포화) | wind 4.0에서 드리프트 3.7–7.7 m > ±3 m → 정체 | §4.5 스케일 법칙 |
| **F11** | iter 예산 확대만으로는 이벤트 능력이 안 생긴다 | P2 +2000 it → 정체, P3 → **회귀** | §4.2 음성 결과 |
| **F12** | 이동표적에서 릴리스는 이월되나 명중은 **리드 부재**로 붕괴 | released-miss > success, 착탄 med ≈ E‖v‖·t_f | §4.6 출발점 |
| **F8** | GPU 병렬 물리에서 per-env attach/detach = **kinematic weld**, 해석 경로와 cm-parity | 추적 1.1 mm, \|Δ\| ≤ 0.021 m; 학습 비용 0 | 시스템 부록 |
| **F10** | 관측 근사 모델은 **실패 특성**까지 이식해야 한다 | 거리감쇠 없는 conf → 상승 farming(max_alt 33%) | §4.3 병리 카탈로그 |

### 2b. 버릴 것 (시행착오·인프라·플랫폼 사고)

Gazebo/PX4 arming 거부·EKF 재수렴·soft reset 처리량(Rule 8/11/14) · RTF 튜닝(Rule 7) ·
docker/GPU 드라이버 충돌(Rule 26) · wobble LPF 튜닝(Rule 15, 상당 부분 저토크 plant 아티팩트) ·
v12 overshoot 트랩(Rule 10, env 특이 튜닝) · mass/inertia override 버그(Rule 19) ·
노트 번호 충돌·워크트리 위생. → **최대한 압축해 "시뮬레이터 검증(sanity gates)" 한 문단 + 부록 표 한 개**로만.
단 Rule 19/21/23은 *방법론적 교훈*으로 한 줄씩 살아남는다(측정 가능한 게이트 없이 학습 결과를 믿지 말 것).

### 2c. 반드시 고쳐야 할 방법론 약점 (제출 전 blocker)

1. **거의 전부 단일 시드(42).** → §3 프로토콜에서 3–5 시드 강제.
2. **`select_best_checkpoint.py` 선택 편향** (같은 eval 지표로 15개 훑어 best 선택) → **독립 held-out 재평가**.
3. ~~eval 시드 미고정~~ → **2026-08-02 `play.py --seed` 추가 완료**(commit `bb18dc0`). 이제 고정 표본 비교 가능.
4. **rule-based 베이스라인 부재** → §4 Table 1이 전부 이걸 메우는 실험.
5. **v19_abd는 A·B·D 번들** → 귀속 분리 필요.
6. **B/C 두 env 혼재** → **v-track(v19)을 본선 env로 단일화**, base env 결과는 포팅해 재현.
7. **v-track 핸드오프가 완전 고정**(4 m/s, +X, 10 m, 원점, 수평) → 일반화 주장 불가. 랜덤화 필수(§5 P0).

---

## 3. 실험 프로토콜 (모든 표가 이 규약을 따른다)

**환경 단일화:** 본선 = `DroneBombardV19Env` 계열(28-D obs / 7-D action, 물리 페이로드, 탐지 프록시).
base env 실험(exp_017/018)은 **v-track으로 포팅해 재현**한다(§5 P1). 두 env 수치 혼용 금지.

**학습:** PPO(rsl_rl), 2048 envs, seed ∈ {42, 43, 44}(핵심 표는 5시드), 동일 iter 예산, 동일 warm-start 규약.
보상 변경 → Fresh Start(replay 개념 없으나 warm-start 재적응 딥 주의, best-ckpt 보존).

**평가:** deterministic, **held-out 시드 집합**(학습에 쓰지 않은 eval seed 목록 고정), 방법 간 **동일 에피소드 표본**(paired).
n ≥ 500 episodes/조건. 체크포인트는 **학습 종료 시점 고정**(best 선택 금지) 또는 별도 validation 분할로 선택 후 test에서만 보고.

**보고 지표 (5종 — 지금은 2종뿐):**

| 지표 | 정의 | 왜 |
|---|---|---|
| $e_\text{land}$ | 실제 착탄점 − 표적, DR 물리 하 | 주 지표 |
| **CEP50 / CEP90** | 원형공산오차 | 에어드롭 문헌의 표준 → 고전 베이스라인과 언어 통일 |
| **$T_\text{deliver}$** | 정책 시작 → 페이로드 접지 시각 | *Learning to Throw*의 throw duration 대응. **호버-드롭 퇴화를 드러내는 유일한 지표** |
| release rate | 에피소드 내 발화 비율 | 이벤트 능력 |
| **feasible release window** | 예측 착탄오차가 임계 이하로 유지되는 구간 길이 | AeroThrow Fig.6a 대응, 타이밍 강건성 |

**통계:** 성공률은 Wilson 구간, 오차는 부트스트랩 CI, 방법 비교는 paired(동일 시드·동일 에피소드) + 시드 간 평균.
**단일 시드 수치는 본문 표에 쓰지 않는다.**

---

## 4. 논문의 뼈대와 표 설계

### Table 1 — "왜 RL이 필요한가" (§4.1, 최우선)

동일 env·동일 표본·학습 없는 arm 포함. **정지 표적 + DR 바람/항력.**

| arm | 릴리스 결정 | 조준 | 학습 | 근거 문헌 |
|---|---|---|---|---|
| **T0** hover-drop | 표적 위 정지 후 투하 | 시각 정렬 | ✗ | Vadduri 2023 |
| **T1** CCIP threshold | $\lVert \hat p_\text{imp}-p_\text{tgt}\rVert \le r$ 즉시 발화 | nominal CCIP | ✗ | CCIP/CCRP 교과서 |
| **T2** predictive-argmin | 수평선 내 예측 상태 전부 탄도 전파 → argmin이 현재 스텝일 때 발화 | nominal CCIP | ✗ | **AeroThrow Alg.1 포팅** |
| **T3** wind-oracle CARP | 참 바람을 주고 릴리스점 최적화 | 바람 보정 CCIP | ✗ | CARP/JPADS |
| **T4** ours − residual | **학습된 트리거** | nominal CCIP | ✓ | — |
| **T5** ours (full) | 학습된 트리거 | **CCIP + 학습 잔차** | ✓ | — |

읽는 법: **T1→T2** = 고전 타이밍 기법의 상한, **T2→T4** = 학습된 트리거의 순수 이득,
**T4→T5** = 유도레벨 잔차의 순수 이득, **T3 vs T5** = 우리 잔차가 "참 바람 정보"의 몇 %를 회수하는가.
T0의 $T_\text{deliver}$가 크게 나쁘다는 점이 "호버 드롭이면 되잖아"라는 리뷰어 반론의 답이다.

### Table 2 — 잔차를 어디에 두는가 (§4.5, **C1의 핵심 증거, 신규**)

동일 파라미터 수·동일 유계·동일 예산.

| arm | 잔차 적용점 | 대응 문헌 |
|---|---|---|
| R0 | 없음 | — |
| **R1** | **속도 명령**에 더함 (제어 레벨) | Silver 2018 / Johannink 2019 정론 |
| R2 | 명령 궤적(task-space)에 더함 | Davchev 2022 rLfD |
| **R3 (ours)** | **예측 착탄점**에 더함 (유도 레벨) | 빈 칸 |

> 지금 노트에 있는 v14 ON/OFF(0.823 → 0.694 m)는 R0 vs R3일 뿐이다. **R1이 없으면 C1은 주장이 아니라 인상이다.**
> 예상 서사: R1은 바람을 "날아서" 상쇄해야 하므로 비행 비용·불안정이 늘고, R3는 조준량만 옮기므로 싸다.
> R1이 이기면 → 논문은 "유도레벨이 더 낫다" 대신 **"유도레벨이 동등한 성능을 2-D·유계·해석가능하게 낸다"**로 프레이밍 전환(안전성 논거).

### Table 3 — 이벤트 능력의 구조 vs 보상 (§4.2, 이미 대부분 확보)

| arm | 종단 구조 | 보상 |
|---|---|---|
| S0 | 근접(d_xy) 성공 종단 | base |
| S1 | 근접 종단 + dense aim shaping (w∈{1,2}) | +aim |
| **S2** | **릴리스 = 종단 이벤트** | base (동일) |
| S3 | 릴리스 종단 + aim shaping | +aim (잉여성 확인) |
| **S4** | 릴리스 종단 + **트리거 주체 = 환경 referee vs 학습 행동** | — |

S0–S3는 exp_017/018로 이미 있음(base env) → **v-track 포팅 재현**만 하면 됨.
**S4가 신규**: 임계 referee(고전 규칙)와 학습 트리거를 같은 종단 구조에서 비교 → "트리거를 왜 학습하나"에 답.

### Table 4 — 보상 형태 병리 (§4.3, 대부분 확보)

상주 vs 포텐셜(F2) · 평평 vs 연속지수+보너스(F5) · 감쇠 ON/OFF(F10) · 잔차 권한 스케일(F9)
· **[신규] smoothness 형태: $\lVert\Delta a\rVert^2$ 단독(현행) vs $\eta\lVert\Delta a\rVert_1+\lVert\Delta a\rVert^2$**
(LtT의 근거: L2 단독은 저진폭 지터, L1 단독은 데드밴딩) · **[신규] 체공시간 페널티 $-k_h t_\text{fall}$ + 반송시간 비용 $-\gamma_t\Delta t$**.

### Table 5 — 민감도/충실도 사다리 (§4.7, **sim-only에서 정직한 sim2real 논거**)

우리는 실기가 없으므로 LtT처럼 "빼면 실기 오차 ×3"을 말할 수 없다. 대신 **train/test 충실도 불일치**로 측정한다:
*A 없이 학습 → A 있는 env에서 평가*의 성능 하락 = 그 요소의 **sim2real 위험 랭킹**.

| 요소 | 현재 | 추가 |
|---|---|---|
| 모터 1차 지연 | **없음**(토크 즉시 인가) | $\tau_\text{mot}$ 1차 응답 |
| 명령 지연 | **없음** | 지연 큐 $\tau_d$ (관측/행동 양쪽) |
| 릴리스 기구 지연 | 0.1 s 고정, jitter 0 | ±1 스텝 jitter + 지연 랜덤화 |
| 릴리스 시 질량 변화 | **없음**(항상 loaded) | 투하 순간 −0.1 kg 스텝 |
| 바람 시간 프로파일 | 에피소드 내 상수 | 돌풍/난류(Dryden) |
| 상태 추정 | 참값 | 위치/속도 노이즈 + **에피소드 상수 바이어스** |
| 페이로드 질량/항력 | 고정 0.1 kg / k 고정 | 랜덤화 |

부수 효과: LtT의 랭킹(식별된 LLC 3.7× > 공력 3.3× > 모터 지연 2.4×)과 **우리 과제(자유낙하·저속·10 Hz)에서의 랭킹이 다를 것**이라는 예측 자체가 논문 소재다.

### Table 6 — 인지 사다리 (§4.6, C2)

| 단계 | 관측 | 상태 |
|---|---|---|
| V0 | 정확한 상대 위치 | 확보(v12) |
| V1 | reveal 반경(블라인드 순항) | 확보(v13) |
| V2 | 픽셀 양자화(cell = k·slant) | 확보(v17) |
| **V3** | **핀홀 (u, v, conf)** + dropout/bias | 신규 |
| **V4** | **실 YOLO in-the-loop** | 신규(렌더 파이프라인 선행) |
| V0′ | **절대좌표 제공(oracle)** | 신규 — 좌표 비복원의 비용을 정량화 |

V0′가 **C2의 정량 근거**다: "좌표를 안 줘도 몇 % 손해로 되는가".

### Table 7 — 이동 표적과 리드 (§4.8, 두 번째 기여)

| arm | 리드 획득 방식 | 성격 |
|---|---|---|
| L0 | 없음(현행) | exp_021 재현 |
| L1 | privileged 표적 속도 obs (28→30) | 관측 개입 |
| L2 | Singer-KF tracker obs (28→35, fresh) | 추정기 개입 |
| L3 | 관측 이력/frame-stack 또는 RNN | 암묵 학습 (Gaudet 계열) |
| L4 | $w_\text{lead}$ 보상 | 보상 개입 (Rule 22 재검증) |
| L5 | 게이트 자체를 리드 조준으로 (구조 개입) | **구조 개입** |
| L-oracle | 참 표적 속도로 PIP 해석 계산 | 상한 |

**F1의 프레임(구조 vs 관측 vs 보상)을 그대로 재사용**하는 게 서사적 이득이다.
exp_021의 timeout 20–28%는 "게이트가 이동표적에서 안 열림"이므로 L5가 유력하다.

---

## 5. 실행 순서 (일정 아님 — 의존성 순서)

### P0. 프로토콜·env 정비 (학습 없음, 비용 최저, 나머지 전부의 전제)

1. ✅ `play.py --seed` (완료, `bb18dc0`)
2. **eval 하네스 확장**: held-out 시드 집합, paired 표본, CEP50/90, $T_\text{deliver}$, feasible-window, JSON 출력.
3. ✅ **핸드오프 랜덤화** (v-track): 방위 ±180° · 속도 U[2,6] · 고도 U[8,12] · 진입 오프셋(횡 ±3/종 ±2) ·
   자세 N(0,5°) · 각속도 N(0,0.2). `DroneBombardHandoffCfg`, 기본 OFF → v11~v19 무손상.
   → [[experiments/exp_022_p0_handoff_dyn_dr]]
4. ✅ **DR 확장**: 컨트롤러 질량 *신념* ±5% · 속도P/자세P·rateP 게인 ±10% · 페이로드 **탄도계수**(=질량 등가) ±20% ·
   관측·행동 per-step 노이즈 + **에피소드 상수 bias**. `DroneBombardDynDRCfg`, **런타임 PhysX 쓰기 0**(Rule 19 우회).
5. ✅ **$T_\text{deliver}$ 로깅**(`Episode_Metric/deliver_time_s`). 체공시간/반송시간 **페널티**는 보상 변경이므로 P2로 이관.
6. `baseline_drop.py` — T0/T1/T2/T3 4종을 **같은 env·같은 obs 제약**으로 구현(학습 없음).

> **P0 중간 판정 (2026-08-03, [[research/handoff_generalization_p0]] / Rule 27):** 동일 v19 ckpt를
> 4조건에서 평가하니 고정 91.0% → +동역학DR 91.5% → +속도/고도/자세 77.1% → **+방위±180° 7.5%**.
> **플랜트·센서 DR은 사실상 공짜, 월드프레임 방위 랜덤화만 파괴적**(발화 시 착탄오차는 4조건 모두
> 0.32–0.40 m 불변 → 조준/투하가 아니라 *접근*만 붕괴 = 관측 프레임 문제).
> → **P1 진입 전 갈림길:** (a) v20 그대로 학습 / (b) 방위불변 obs(레이아웃 변경 → fresh 필수) /
> (c) 둘을 같은 예산으로 비교해 ablation 행으로 승격. 권고 = (c), 순서는 (a) → 곡선 보고 (b).

### P1. 본선 재정렬

7. exp_017/018(S0–S3)을 **v-track으로 포팅 재현** → 모든 수치가 한 env에서 나오게.
8. 헤드라인 구성 **3–5 시드 재학습** + held-out eval (기존 best-ckpt 수치 폐기).
9. v19_abd **A/B 분리**(A만 / B만 / A+B) — 붕괴 방지의 귀속.
10. v20(fresh 단일 run) vs 커리큘럼 체인 → F4 확증/반증.

### P2. 기여 실험

11. **Table 2 (잔차 위치 R0–R3)** — C1의 사활.
12. **Table 1 완성** (T4/T5 학습 arm).
13. **S4** (referee 트리거 vs 학습 트리거).
14. smoothness L1+L2, 체공시간 페널티 → **속도–정확도 Pareto** 그림(LtT 대응).

### P3. 확장

15. Table 5 충실도 사다리(train/test 불일치 랭킹).
16. Table 6 V3/V4 인지 사다리 + V0′ oracle.
17. Table 7 리드 개입 L0–L5.

### P4. 선택

18. HIL/실기. **현재로선 논문 필수 아님** — 대신 §4.7의 불일치 랭킹으로 "무엇을 먼저 모델링해야 하는가"를 답한다.

---

## 6. 논문 구조 초안

1. **Introduction** — 정밀 투하 = 희소 이벤트 + 모델 오차 + 부분관측. end-to-end도, 순수 고전 유도도 아닌 분해.
2. **Related Work** — §1의 4갈래(케이블 던지기 / 자유낙하 에어드롭 / 잔차 학습 위치 / 이동표적 리드).
3. **Problem & Method** — CCIP 프리미티브, 릴리스 엔벨로프(안전 경계), 유계 잔차, **릴리스=종단 이벤트**, 탐지형 관측.
4. **Experiments**
   4.1 베이스라인 대비 (Table 1) — *왜 학습이 필요한가*
   4.2 이벤트 능력: 구조 vs 보상 (Table 3, F1/F3/F11)
   4.3 shaping 병리 카탈로그 (Table 4, F2/F5/F10)
   4.4 통합 데드락과 부트스트랩 (F4 + v20)
   4.5 **잔차 위치** (Table 2, C1) + 권한 스케일(F9)
   4.6 인지 사다리와 좌표 비복원의 비용 (Table 6, C2)
   4.7 충실도 민감도 = sim2real 위험 랭킹 (Table 5)
   4.8 이동 표적과 리드 (Table 7, F12)
5. **Discussion** — 진단 시그니처 카탈로그(증상→계측→처방), 한계(sim-only, 평면 지형, 프록시 인지).
6. **Appendix** — weld parity(F8), 지표 의미론(F6), 시뮬레이터 sanity gate.

**투고 후보:** ICRA / IROS / RA-L. 6페이지 압축 시 §4.7 또는 §4.8 중 하나를 부록으로.

---

## 7. 주장 문구 (방어 가능한 형태로 미리 고정)

- ❌ *"vision-based drop이 새롭다"* — **반증됨**(Mathisen 2020, Vadduri 2023).
  ✅ **"탐지형 부분관측에 직접 조건화되어 표적 절대좌표를 복원하지 않는 *학습된* 순항-릴리스 정책은 우리가 처음이다."**
  (Mathisen: 탐지→절대좌표→고전 CARP / Vadduri: 호버 정렬 후 정지 투하, 비학습)
- ❌ *"학습된 릴리스 결정이 처음이다"* — **반증됨**(APER-DDQN 2022).
  ✅ **"릴리스 능력이 dense shaping이 아니라 종단 구조에서 나온다는 것을 보상 고정 단일변수 개입으로 귀속한 것이 처음이다."**
- ❌ *"고전 유도량에 학습 보정을 더한 게 처음이다"* — **반증됨**(PN+RL bias, ZEM/ZEV 적응, IPP 보정).
  ✅ **"비추력 자유낙하 페이로드의 단발 릴리스에 대해, 탄도 탄착점 예측에 유계 2-D 잔차를 두고 릴리스 타이밍과 동시 학습한 것이 처음이며, 잔차 위치(제어 vs 유도) 비교로 그 이점을 정량화한다."**
- LtT 대비: **"저들의 기여는 시뮬레이터 충실도(로프+HIL), 우리 기여는 분해 구조(고전 유도 + 최소 학습) — 직교한다."**
- AeroThrow 대비: **"저들은 하드웨어(델타암)와 온라인 관측기로 오차를 상쇄, 우리는 유도레벨 학습 잔차로 상쇄. 저들의 Alg.1은 우리 T2 베이스라인이 된다."**

---

## 8. 리뷰어가 때릴 지점과 선제 대응

| 공격 | 대응 |
|---|---|
| "호버 후 투하하면 되는 것 아닌가" | T0 arm + $T_\text{deliver}$ + 속도-정확도 Pareto |
| "CCIP만으로 충분하지 않나" | T1/T2/T3 + 바람 DR 하 잔차 이득(T4→T5) |
| "잔차를 제어에 두면 더 낫지 않나" | **Table 2 R1/R2 arm** |
| "sim-only인데 의미가 있나" | §4.7 train/test 충실도 불일치 랭킹 + LtT 랭킹과의 대조 |
| "핸드오프가 고정이라 암기 아닌가" | P0-3 랜덤화 + OOD 거리/속도 평가 |
| "단일 시드 아닌가" | §3 프로토콜(3–5 시드, held-out, paired, Wilson/부트스트랩) |
| "best checkpoint를 골랐잖나" | 학습 종료 시점 고정 또는 validation/test 분리 |
| "이동표적은 실패 아닌가" | L-oracle 상한과 함께 제시 → 리드가 병목임을 **측정된 사실**로 |

---

## 관련 노트

- [[research/research_overview_for_paper]] — 계보·warm-start 체인·F1~F12 원사료
- [[research/rl_rules]] — Rule 19/20/21/22/23/24 (방법론 교훈)
- [[experiments/exp_017_stageA_aim_reward]] · [[experiments/exp_018_release_terminal]] — Table 3의 원 실험
- [[experiments/exp_019_physical_payload]] · [[experiments/exp_020_o5jn9xzk_payload_training]] — F8
- [[experiments/exp_021_v19_moving_target]] — Table 7의 출발점
- [[research/ccip_release_decoupling]] · [[research/moving_target_models]] · [[research/phase1_plan]] (baseline_ccip 설계 원안)
- 코드: `isaac_lab/drone_bombard/v11_env.py`(v11~v20), `drone_bombard_env.py`(base), `isaac_lab/play.py`(eval)
