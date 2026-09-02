---
date: 2026-04-14
tags: [index, dashboard]
status: active
type: index
---

# 드론 정밀 투하 연구 — Obsidian 대시보드

> **프로젝트:** CCIP 기반 잔차 강화학습 활용 드론 정밀 투하
> **스택:** ROS 2 Humble · Docker · PX4 SITL · SAC (SB3) · Gazebo Harmonic · L4 GPU

---

## 현재 상태 (2026-09-02)

### ⭐⭐ 오늘의 결론 — 일반화 감사: 두 축 통과, 한 축 실패

- **⭐⭐⭐ 정책 전이 실패: [[research/residual_policy_coupling]]**
  seed-1 정책으로 적합한 회귀기를 **seed-2 정책에 주입하면 순손실**이다 —
  CEP50은 −7%로 좋아 보이지만 succ@1.0 **96.0 → 89.3%**, CEP90 **0.542 → 0.830**.
  seed-2 자체 데이터로 재적합하면 −22.0%, CEP90 0.562로 복구.
  **obs→drift 사상은 물리가 아니라 정책의 성질이다** — tilt가 재는 것은
  $\bar\theta \approx F_{wind}/K_{policy}$ 이고 $K$는 컨트롤러마다 다르다 (Rule 39).
- **⭐⭐ 미지 사거리 통과: [[experiments/exp_029_l1_sl_generalization]] §3**
  26–30 m(적합은 18–22 m)에서 CEP50 0.299 → **0.218 (−27.3%)**,
  오라클 회수율 **85.3%** — 훈련 범위(85.0%)와 소수 둘째 자리까지 같다.
  기하를 외운 게 아니라 바람을 읽는다. 배달률은 세 arm 모두 73.83%로 동일.
- **⭐⭐ 오라클 간극의 정체 = 정보, 데이터 아님: [[research/residual_label_efficiency]]**
  같은 네트워크에 **참바람을 주면 100 에피소드에서 $R^2$ 0.982**. 관측만이면
  2,334 ep에서도 0.611(배증당 증가폭 0.081→0.032 감쇠, 점근선 0.65–0.70).
  **실기 보정 예산 = 1,000회 투하** (최종 이득의 96%).
- **⚠️ 새 함정 — 잘못 보정된 잔차는 꼬리를 먼저 망친다 (Rule 40).**
  라벨 100개 arm: CEP50 −5.5%(개선)인데 CEP90 **+43%**, succ@1.0 **−6.5 pp**.
  전이 arm과 **같은 서명**. **CEP50 단독 판정 금지** — CEP90·succ@1.0 동반 보고.
- **다음 1순위 = 게인 무관 특징 (가설).** 결합의 원인이 $\bar\theta \approx F_{wind}/K_{policy}$의
  분모이므로, 분모가 안 들어가는 양("지령 가속 − 실제 가속")을 입력으로 주면 사라질 수 있다.
  검증 ~15분, 성공 시 논문 한계 항목 하나 소멸. **RL보다 먼저.**
- ⚠️ **DAgger는 축 P를 고치지 못한다** — 축 P는 *배율* 오류, 잔차 on/off는 *커버리지* 부족으로
  **종류가 다르다.** DAgger는 후자만 고친다. 축 P가 DAgger 우선순위를 올린 것은 인과가 아니라 경고.

### 🔧 코드 (2026-09-02)

- `_fit_sl_residual.py --train_ep N` — 테스트 분할 고정한 채 학습 에피소드만 절단
- `_agg_sl_eval.py` — arm 자동 탐지(파일명 파싱) + `--wandb` (실험당 run 1개, Table 1장)
- `_sl_gen.sh` — 세 축 배치 (33 run, ~20분)
- wandb: `sl_gen_unseenR` / `sl_gen_policy_transfer` / `sl_gen_label_count` (job_type=eval)

---

## 현재 상태 (2026-09-01)

### ⭐⭐ 오늘의 결론 — 논문의 학습 행이 채워졌다 (PPO 없이)

- **⭐⭐⭐ 지도학습 잔차가 동작한다: [[experiments/exp_028_l1_sl_pilot]]**
  동결 L0 + 오프라인 회귀만으로 **CEP50 0.305 → 0.209 (−31.4%)**, DR 2.5에서도
  **0.498 → 0.334 (−32.9%)**. 특권 오라클 천장(0.192 / 0.288)의 **85% / 78%**를 회수한다.
  **PPO 없음, L0 재학습 없음, 보상 코드 미변경.** 배달률·추락 수는 전 팔 동일.
- **⭐⭐ 관측은 바람에 장님이 아니다: [[research/residual_observability]]**
  회귀 $R^2$ — 상수 0.000 · linear(obs) **0.127** · MLP(obs) **0.436** ·
  MLP(obs+tilt 누적) **0.611** · MLP(obs+참바람) **0.998**.
  정보는 있고, **비선형으로** 들어 있다. 두 학습 시드·두 DR 수준에서 순서가 같다.
- **⭐⭐ tilt 채널 판정 — 넣을 값어치는 있지만 L0 재학습은 필요 없다.**
  지도 잔차는 정책과 **분리된 네트워크**라 자기 입력을 만든다 (정책 26채널 / 잔차 36채널).
  RL 잔차라면 관측 확장 → `observation_space` 변경 → **Fresh Start**가 강제된다.
- **⭐⭐ 새 함정 — 결과 공간 잔차는 매끄러워야 한다: [[research/release_gate_jitter]]**
  RMSE가 멀쩡한 잔차가 CEP를 **2.5배 악화**시켰다. 게이트가 **첫 교차** 판정이라
  예측의 스텝간 요동(0.073~0.129 m/step vs 참값 0.018)의 **극값을 표집**하고,
  릴리즈가 계통적으로 빨라진다. **EMA α=0.3 한 줄로 0.462 → 0.209.**
  → RL 잔차에도 그대로 적용됨. 08-29 L1 붕괴의 새 원인 후보.
- **`success@1.0`은 또 안 움직였다** (93.83 → 93.50). `success@0.5`는 **78.83 → 86.50**.
  [[research/paper_metrics]]의 지표 결정이 세 번째로 확인됨.

### 🔧 코드 (2026-09-01)

- `play.py` — `--dump_sl` (관측·참드리프트 덤프), `--sl_residual` (지도 잔차 주입,
  오라클과 동일 경로), `--sl_ema` (시간 평활)
- `_SLResidual` — 인과적 tilt 누적 10채널을 온라인 유지. 정책이 보지 않는 입력이다.
- L1-RL의 위치 변경: 필수 경로 → **"RL이 지도학습을 넘어서는가" ablation**

---

## 현재 상태 (2026-08-30)

### ⭐ 오늘의 결론 (오후 재측정 이후)

- **⭐⭐ Table 1 확정 — 정확도는 동률, 시간·신뢰성으로 이긴다: [[experiments/exp_027_seen_unseen_3seed]]**
  투하 고도까지 튜닝한 공정한 baseline(T2@1.5 @3.5 m) 대비 **착지오차 0.330 vs 0.337(동률) ·
  배달시간 8.09 → 5.80 s(−28%) · 배달률 76.2 → 94.0%(+17.8 pp) · 투하속도 2.5배.**
  ⛔ 오전의 "파레토 전 축 지배"는 철회 — 튜닝된 T2가 CEP50에서 근소하게 앞선다.
- **⭐⭐ DR 축이 주 주장 그대로 그려진다.** CEP 격차가 DR=0의 **−0.027(모델 기반 승)** 에서
  2.5의 **+0.180**까지 **단조 증가**. 통제군이 깨끗해 *"RL이 더 정확하다"* 가 아니라
  *"모형 오차가 커질수록 RL이 앞선다"* 로만 쓰게 강제한다
- **⭐ 두 외삽 축에서 정반대다.** 모형오차 외삽(DR 2.5)에서 L0가 **가장 적게** 잃고(−13.8 pp,
  격차 +28.3 pp), 사거리 외삽(26–30 m)에서 **가장 많이** 잃는다(−20.3 pp).
  그런데 **CEP50은 0.30으로 불변**이고 추락만 4.3 → 26.8% — **조준은 외삽되고 비행 제어는 안 된다**
- **⭐ 지표 확정 — 성공률을 헤드라인에서 내린다: [[research/paper_metrics]]**
  참값을 다 아는 잔차조차 `success@1.0`을 92.7 → 91.8로 **못 움직인다**. 같은 잔차가 CEP50은 −48%.
  헤드라인 = **배달시간 · 착지오차 · CEP50/CEP90 · 배달률**. 보상 `success_radius`는 1.0 유지(재학습 없음)
- **⭐ 잔차의 진짜 상한선은 CEP50 −31%: [[research/residual_ceiling]]**
  전지적 오라클(−48%)은 참 릴리즈 지연·탄도계수까지 알아 **도달 불가**. 바람만 아는 변종이 공정 상한.
  **그 −31%가 전부 바람이다**(바람 기울기 0.0711 → 0.0089). DR 2.5에서 −35%로 커진다
- **오차 예산: [[research/error_budget_l0]]** — 바람 상관 $r$=0.495, 바람 성분 RMS 0.255/전체 0.415.
  ⭐ **L0는 바람을 추정한 게 아니라 바람에 둔감한 투하 기하(3.53 m·하강 −1.17)를 골랐고 그게 이득의 78%**
  ($t_{fall}^2$ 법칙, 실측이 2~3% 안에서 일치). 바람은 중앙값 세기에서 **관측 잡음에 묻힘(SNR 0.4)**
- **학습 시드 2개로 정리: [[research/training_seed_protocol]]** — 92.7 / 93.8%로 1.1 pp 차이.
  경쟁 논문(Zhai et al. 2026)은 학습 시드를 아예 안 쓴다. seed 3은 60/1000에서 중단

### ⛔ 오늘 철회한 결론 5개

| 철회 | 왜 |
|---|---|
| "릴리즈 속도 1.39 m/s, 정책이 감속해서 던진다" | `final_speed_xy`가 **착지 시점** 값이었다. 실측 **3.54 m/s → throw** (Rule 33) |
| "명시적 투하속도 보상항이 필요하다" | 막을 대상이 없었다 |
| "6개 팔을 전 축 파레토 지배" | baseline의 **투하 고도를 튜닝 안 했다.** 고도만 낮춰도 T2 CEP50 0.457 → 0.287 |
| "L0가 특권 오라클과 같은 바람 강건성에 도달" | T3의 기울기는 **스크립트 비행**의 값. 같은 정책 위의 바닥은 0.0056 — **바람 오차 92% 잔존** |
| "T3 55.5%" | 4 m/s 패스에서 잰 점. 동일 조건(3 m/s)에서 **72.5%** |

### 🔧 코드

- `request_release()`에서 **투하 순간의 속도·고도·$v_z$ 래치** (Rule 33) + 하네스가 `carry_m` 산출
- `--marker_dist` / `--release_alt` / `--release_descent` — baseline을 **최적 구성에서** 비교하기 위한 축
- `--oracle_residual` / `--oracle_residual_wind_only` — 잔차 상한선 팔
- **L1 레시피 4건**: `w_residual`(신설, 기존에 없었음) · `residual.scale` 2.0→1.0(+`oracle_scale` 분리) ·
  `--zero_init_residual` · `--freeze_nominal`. `tests/test_residual_head.py` 6건 통과
- ⚠️ `--freeze_nominal`은 잔차를 **L0 특징의 선형 결합**으로 제한한다 → 표현력 부족으로 인한
  false negative 위험. **동결/비동결 두 팔 병행**으로 완화

---

## 현재 상태 (2026-08-27)

- **⛔🔧 T3 "오라클"이 상한선이 아니라 하한선 아래였다 — 수정 완료: [[research/t3_oracle_entrainment]] / [[errors/err_20260827_payload_drag_body_frame]]** (Rule 31)
  - 해석식 바람 보정이 "즉시 완전 엔트레인먼트"를 가정 → 3.7~4.7배 과보정 → T3 47.5% < T0 hover 91.5%. 플랜트 동일 적분(`integrate_payload_impact`)으로 교체 후 T3 > T2 회복
  - **지배 오차는 바람이 아니라 페이로드 자기 속도에 대한 항력**(무풍에서도 −1.15 m). $v_z$ 누락과 같은 종류의 결손이 하나 더 있었음
  - 페이로드 항력을 월드 프레임으로 계산해놓고 링크 프레임 기본값으로 전달하던 버그 동시 수정 (`is_global=True`)
- **⚠️ DR_SCALE 스윕이 현 상태로는 성립하지 않음 (사전 계측으로 발견).** 랜덤화를 완전히 꺼도 오차 p50 0.44 m — 결정론적 바닥이 랜덤화 성분을 덮고 있음. 릴리즈 지연 제거 + 고도 규약 + 자기속도 항력항 보정으로 **0.44 → 0.015 m**, 그제서야 스윕이 단조·선형
- **🏗️ 환경 코드 재구축 완료** (`f0ef4b8`) — `v11_env.py`의 버전 상속 사슬(설정 10개/환경 7개)을 단일 `task_env.py`로 대체. **모든 DR을 `drone_bombard_env.py`로 집약하고 세 그룹으로 분리**: `model_err`(A: 바람·탄도계수·릴리즈 지연 — `scale`이 곱해지는 유일한 그룹) / `dyn_dr`(C: 센서·액추에이터, 스윕 내내 고정) / `handoff`(B: 시나리오). 버전 접두어 전면 제거, 해석식 채점 경로 삭제, **릴리즈 지연을 플랜트에 실제 구현**. v11~v20은 `register_retired_lineage()`로 opt-in 재현만 가능
- **재구축 후 T0~T3 재측정** (n=128, GT, DR_SCALE 1.0): T1 28.9% / T2 65.6% / **T3 77.3%** / T0 80.5%. **오라클 갭이 실재하는 양수**(CEP50 T2 0.646 → T3 0.511 m) — "잔차가 상한의 X%를 회복" 문장이 비로소 성립
- **⚠️ A그룹 축 무게 실측 — 계획 전제 하나 기각.** 바람 p50 0.170 m(~90%) / 릴리즈 지연 0.058 m(~31%) / **탄도계수 0.012 m(~6%)**. 탄도계수는 ±100%로 넓혀도 0.044 m — **논문에서 "관측 불가 외란"을 탄도계수로 지목하면 안 된다.** 그 역할은 릴리즈 지연(원리상 관측 불가 + 속도 비례)
- **✅⭐⭐ (08-30) L0 학습 성공 — 성공률 92.0%, baseline front 전 축 지배.** CEP50 **0.319 m**, 배달 **5.86 s** 로 스크립트 6개 팔을 배달시간·성공률·CEP50 동시에 앞선다. 보상 결함 2건(`k_landing` 지수 붕괴, 성공반경 0.5의 도달 불가 보너스) + 학습 구조 결함 6건(롤아웃 32<에피소드 97, 보상 항 6개 무효화, 리턴 미정규화 등) 수정 후. ⚠️ **L0가 T3(모수 참값)를 크게 이긴다** → 모수 불확실성이 주된 난이도가 아니었고 **L1의 여지를 위협**. → [[daily/daily_2026-08-30]]
- **⚠️ (08-30) 속도 교락 발견** — T2를 속도별로 재니 스크립트만으로도 1.30 m/s에서 65.0%. 단일 점 비교는 무효이고 **곡선으로 제시**해야 한다. 이 곡선이 논문 그림 2의 baseline front
- **⛔ (08-29) 첫 학습 파일럿 — 실패.** L1(잔차) 1200 iter 성공률 **0.9%**, L2(pure RL) **0.0%** — 학습 0인 스크립트 T2(**10.5%**)보다 한참 아래. 추세 없음 → **Rule 29 조기 중단**. 원인 추정: `residual.scale=2.0`이라 무작위 잔차가 조준을 초기에 3배 망가뜨림(잔차 헤드 0 초기화 부재). 다음: **L0 통제군 + ‖δ‖ 로깅** → [[daily/daily_2026-08-29]]
- **🔧 (08-29) 드론 메시가 물리와 5.4배 어긋나 있었다** — 물리는 X500(2.07 kg, 대각 500 mm)인데 렌더는 Crazyflie cf2x(92 mm). 영상에서 페이로드와 크기가 비슷해 보인 원인. `spawn.scale=5.4` 적용(질량·관성 불변 검증 완료)
- **✅⭐⭐ (08-27) Phase 1 완료 — 무학습 파트 전량 확보.** 보상 결함 6건 + 베이스라인이 구조적으로 place였던 문제까지 검출·수정. **Table 1 · 그림 2(속도-정확도) · 그림 4(인과 귀속) 확보.** 남은 것은 학습 행(L0/L1) 하나. → [[daily/daily_2026-08-27]]
- **⭐ DR_SCALE 스윕 (정속 패스, n=200)**: 귀속 갭 **−1.5 → +1.0 → +8.0 → +19.0 pp 단조**, scale 0에서 ≈0(통제군). T2 34.0→10.5% 단조 하락, T3 32.5→29.5% 평탄. **헤드라인 = 성공률@0.5 + CEP50 + 배달 시간** (CEP90은 단조 아님 — 꼬리는 제어 오차라 회복 안 됨)
- **Table 1 (DR 1.5, n=200)**: T0 place 35.0%/9.00 s · T1 throw 11.0%/6.77 s · T2 throw 10.5%/6.70 s · **T3 throw 29.5%/6.83 s**. 던지면 **24.5 pp 잃고 2.3 s 번다**. **T3조차 T0를 정확도로 못 이김** → 논문 축은 "속도 낸 상태에서의 정확도" 
- **✅⭐ 논문 아키텍처 v4 확정** ([[research/research_architecture]]) — 주장/실험/ablation/프로세스 결정 완료. **다음은 학습(Phase 1~2)**. ⚠️ T3 "상한선" 표현 전면 철회 → 정보 ablation
- **✅ 릴리즈 판정 100 Hz + 성공 반경 0.5 m** ([[experiments/exp_026_release_rate_100hz]]) — pickle-and-hold(정책은 커밋, 발사 시점은 물리 주파수 솔버). T2 CEP50 **0.762 → 0.395 m**. ⚠️ **T1과 T2가 완전히 동일해짐**(argmin의 이점 전부가 타이밍 해상도였음) · ⚠️ **오라클 갭이 중앙값→꼬리로 이동** → 헤드라인 지표를 CEP50에서 **CEP90/성공률**로 교체해야 함. 급기동 가능성 분석: [[research/agility_ceiling]]
- **✅ DR_SCALE 스윕 유효성 게이트 통과** ([[experiments/exp_025_dr_scale_sweep_gate]]) — 무학습 T2/T3만으로 논문 핵심 그림의 전제 확인. T2 CEP50 단조↑(0.570→0.815) · T3 평탄(0.573→0.491) · **오라클 갭 단조↑(−0.003→0.325 m)**. scale 0에서 갭 ≈ 0이라 통제군도 깨끗. **학습 착수 조건 충족**
- ⚠️ **scale 0에서도 CEP50 0.57 m** — 표적 참값 + 탄도 오차 0.010 m이므로 **전부 제어·릴리즈 판정 타이밍**(10 Hz × 4 m/s = 스텝당 0.4 m). 릴리즈 판정만 100 Hz로 올리는 값싼 개선의 우선순위를 올릴 것
- 아키텍처 문서 정정 5건 반영 완료 (바람 상수 유지 / 바람 미관측 / 팬텀 항력 채널 / 릴리즈 지연 랜덤화 / 주 실험 GT)

## 현재 상태 (2026-08-23)

- **⛔🔧 CCIP가 수직속도를 누락하고 있었다 — 수정 완료: [[research/ccip_vz_omission]] / [[errors/err_20260823_ccip_vz_omission]]**
  `ballistic_impact`가 $t=\sqrt{2H/g}$(정지 투하 특수화)를 쓰는 동안 **v16의 물리 페이로드는 드론의 실제 $v_z$를 상속**받아 낙하했고,
  v19가 `release_max_vz=3.0`을 허용하며 전제가 깨졌다. **릴리즈 엔벨로프에서 모델 오차의 ~70%가 이 누락**
  ($v_z$ 0.547 m vs 바람 0.197 m vs 항력 0.120 m, p50; $v_z{=}-3$ m/s·$H{=}8$ m·수평 6 m/s에서 **1.62 m overshoot**).
  → 논문 주 주장 *"CCIP는 정확한데 항력·바람이 틀리게 만든다"*가 **거짓이었음**. `vel_z`를 필수 인자로 승격해 호출부 11곳 갱신,
  테스트 69 passed + smoke 3종(v19/phase1/phase2) 완주. **exp_019 후속 #3에 이미 기록돼 있었으나 미이행이었다** → Rule 30.
- **📐 아키텍처 문서 v2 → v3 전면 개정: [[research/research_architecture]]**
  선행연구 독해만으로 쓰인 v2를 코드와 1:1 대조 → **전제 1개 거짓 + 코드 버그 4개 + 성립 불가 DR 축 3개 + 측정 결과와 충돌 4건** 발견.
  미수정 잔여: **T3 "wind-oracle"이 오라클이 아님**(해석식 바람항 3~7배 과보정 → T3 47.5% < T0 91.5%의 원인, B4·Oracle gap·Abstract가 의존),
  페이로드 항력 프레임(`is_global`), `_drag_coef` 팬텀 채널, `release_delay` 실체 없음.
  성립 불가 DR: 질량과 $C_d$는 **같은 축**($k/m$), CoM 오프셋은 kinematic weld라 **효과 0**(Rule 24b).
- **🗑️ 기존 학습 산출물 전량 폐기 결정.** v11~v20 체크포인트·exp_014~024 수치를 논문 근거로 쓰지 않음
  (v11~v19 DR 부재 → 일반화 검증 불가 / 가설 없는 warm-start 누적 → 귀속 불가).
  **유지**: Rule 16~30, `eval_harness.py`, `baseline_drop.py` T0~T3, kinematic weld 페이로드, 등가변환 DR 패턴, `math_utils.py` + 테스트.
  ⚠️ **정정**: "DR을 안 해서 robustness가 없다"는 절반만 맞다 — **v20은 DR을 켰고 실패했다**(exp_024).
  원인은 DR 부족이 아니라 **world-frame 관측**(Rule 27) + 틀린 prior warm-start(Rule 29). **fresh start만으로 안 풀린다.**

---

## 현재 상태 (2026-08-03)

- **❌ (a)안 실패 — v20 warm-start 학습은 페널티 회피로 수렴: [[experiments/exp_024_v20_warmstart_failure]]**
  reward −44.8→+10.8로 올랐지만 상승분 전부가 `out_of_range` 회피였고 **과제 지표(접근·조준·발화율)는 1000 iter 평탄**.
  σ 2.10→6.27 단조 + 액션 포화 93% → det eval에서 **v20 13.0%(7.5%에서 CI 겹침), v19 100.00%→8.50% catastrophic forgetting**.
  **관측 프레임 문제는 재학습으로 안 고쳐진다**는 확증 → Rule 29. 다음: (a′) 방위 커리큘럼 vs (b) obs 프레임 불변화 **판단 대기**.
- **📊 P0 완료 — Table 1 1차 실측: [[experiments/exp_023_table1_baselines]]**
  공유 평가 하네스(`eval_harness.py`: paired 평가·CEP50/90·Wilson/부트스트랩 CI·반송시간·feasible window·JSON)와
  무학습 베이스라인(`baseline_drop.py`: T0 hover / T1 CCIP 임계 / T2 AeroThrow argmin / T3 wind-oracle 잔차) 신설.
  **paired 200 ep: T0 91.5%(CEP50 0.358 m, 7.90 s) · T1 6.5%(1.606) · T2 35.5%(1.097) · T3 47.5%(1.010) · T5 ours 100.00%(0.370, 5.95 s)**
  → 학습이 **특권 정보 고전 arm을 성공률 2.1배·CEP 2.7배** 상회. 단 T0 대비 우위는 정확도가 아니라 **민첩성**.
  부수 발견: 착지 래치 버그 → [[errors/err_20260803_payload_landing_latch]] (Rule 28), v19 기준선 91.0%→**100.00%** 정정.
- **🧪 P0 착수 — 핸드오프 랜덤화 + 동역학/센싱 DR (`v20` env 신설): [[experiments/exp_022_p0_handoff_dyn_dr]]**
  v11~v19의 완전 고정 핸드오프(원점·10 m·+X 4 m/s·수평)를 토글로 제거하고, PhysX 런타임 쓰기 없이
  플랜트/센서 DR을 추가(질량→컨트롤러 *신념*, 페이로드 질량→*탄도계수* 등가). 유닛 66/66 + 프로브 27/27 PASS,
  v19 ckpt 무손실 warm-start 확인. **동일 ckpt 4조건 판정: 고정 91.0% → +동역학DR 91.5% → +속도/고도/자세 77.1%
  → +방위±180° 7.5%** — 동역학 DR은 공짜, **월드프레임 방위 랜덤화가 파괴적**(접근만 붕괴, 투하 스킬은 불변)
  → [[research/handoff_generalization_p0]] (Rule 27). 다음: v20 학습 vs 방위불변 obs 갈림길.
- **📄 논문 연구 계획 (앞으로의 설계도): [[research/paper_research_plan]]** — 문헌 105편 스윕 + `Learning to Throw`/`AeroThrow` 정독 기반. **비어 있는 칸 = 자유낙하·멀티로터·순항 릴리스·학습 트리거·유도레벨 잔차·탐지 조건화·바람 DR·이동표적의 조합.** 주장 3종(C1 유도레벨 잔차 / C2 좌표 비복원 탐지 조건화 / C3 릴리스=종단 구조 귀속)을 각각 최근접 선행에 대해 헷지한 문구로 고정. 표 7종 설계(T0~T5 베이스라인 · R0~R3 잔차 위치 · S0~S4 구조 vs 보상 · 보상 병리 · 충실도 사다리 · 인지 사다리 · 리드 L0~L5), 지표 5종(+CEP, 반송시간), 실행 순서 P0~P4, 리뷰어 공격 8종 선제 대응.
- **📄 연구 전체 통합 개요 (논문용): [[research/research_overview_for_paper]]** — 두 트랙(Gazebo/SAC, Isaac Lab) + 세 계보(base env 커리큘럼 / v-track 사다리 v11~v20 / 이동타겟)를 한 문서로 합침. 각 v가 추가한 기능·obs/action·warm-start 출처·결과 표, 체크포인트 혈통도, 확정 발견 12종(F1~F12), **이미 확보한 ablation 13축 + 추가로 필요한 실험 N1~N7**, 방법론 약점 7종(단일 시드·select_best 선택편향·rule-based 베이스라인 부재 등).
- **준상 v-track 연구노트 34개 `main` 복원 (2026-08-02).** 구 main 아카이빙 때 태그 `archive/main-pre-isaac_jk-promotion`에만 남아 있던 `notes/**/*_junsang.md`(v11~v19 1차 사료 + 모델 스펙 + 붕괴 진단 + SAC 초기 연구)를 전부 복원. 진입점: [[research/isaac_model_spec_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[research/isaac_v19_collapse_nodrop_junsang]] · [[00_index_junsang]]. ⚠️ `exp_006`~`exp_016` 번호가 제균 트랙과 충돌하니 인용 시 `_junsang` 접미사로 구분할 것.
- **`.gitignore` 정비 (2026-08-02).** `ros2_ws/` 접두사 누락으로 YOLO datasets/epoch 가중치 규칙이 무효였던 것 수정 + `rl_abtest_*`/`rl_dryrun_*`/`rl_softreset_test`(현재 untracked 148MB) 제외 → `git add .` 사고 방지. Obsidian 로컬 UI 상태(`workspace.json`)는 추적 해제. **이미 커밋된 367MB(datasets .jpg 173MB + epoch*.pt 180MB + ign_recording.mp4 14MB)는 `git rm --cached` 별도 판단 필요.**

## 현재 상태 (2026-08-01)

- **브랜치 정리: `main` = `isaac_jk`로 승격 (2026-08-01).** 그동안 `main`이 07-03 시점(`Isaac Lab migration Phase 1 skeleton`)에 정체돼 있었음 — `isaac_jk`가 실제 진행 중인 유일한 통합 브랜치임을 확인 후 승격. 구 `main`(junsang `_junsang` 연구노트 20여 개 + 초기 `isaac_lab_tasks/` 스켈레톤)은 태그 `archive/main-pre-isaac_jk-promotion`으로 보존. `Isaac-JS`(제균 개인 브랜치, 07-02 이후 Gazebo/SAC 트랙만 진행돼 Isaac Lab 코드 없음)는 고유 연구노트(`daily_2026-07-05_gazebo_v15_regression`/`daily_2026-07-07`, Rule 25/26)만 `isaac_jk`로 포팅 후 삭제. `Issac_JS`(junsang, 오타 아님)는 미변경 — 단, 세션 중 junsang이 새 커밋(v20 task 등록)을 푸시해 아직 `main` 미반영 상태. → [[daily/daily_2026-08-01]]
- **팀 브리핑(2026-07-28): [[daily/daily_2026-07-28_team_briefing]]** — isaac_jk 머지/이동타겟·KF 현황 + 팀원별 할 일
- **병행 트랙 — Isaac Lab: 이동 타겟(CV/CA/CT) + Singer-KF 트래커 구현 (2026-07-28).** `--target_motion {gm,cv,ca,ct}`·`--moving_target`·`--target_kf`(obs 14→21) 신설, 단위테스트 57/57 + 스모크 4종 PASS, KF 추적오차 0.09–0.12 m. **--target_kf는 14-dim ckpt warm-start 불가(Fresh Start).** → [[research/moving_target_models]]
- **병행 트랙 — Isaac Lab: exp020 시연 영상 파이프라인 + 렌더러 포렌식 (2026-07-24).** `record_episode.py` 재작성(metric-only referee로 발화→detach→실낙하 관측, trajectory NPZ 덤프) + `animate_episode.py`(matplotlib 3D/탑다운 mp4) + `sdg_dtype_patch.py`(annotator attach 차단 해제 핫패치). **실측 첫 데이터: CCIP pred err 0.111 m vs 실낙하 측정 miss 0.342 m** (seed 0, ×3 결정론 재현). **RTX 렌더 붕괴의 근본 원인 = 07-05 ultralytics pip이 교체한 numpy 2.4.6(omni ABI 파괴) — numpy 1.26.4 다운그레이드로 완전 해결, TiledCamera 진단도 같은 뿌리로 재해석.** 산출물: `logs/recordings/drone_bombard_exp020_payload_final.mp4`(실사 RTX 체이스캠) + `_anim.mp4`(matplotlib). → [[errors/err_20260724_isaac_render_frozen_fabric]]
- **병행 트랙 — Isaac Lab: exp_019 물리 페이로드 attach/detach 구현·검증 완료 (4/4 PASS, Rule 24).** 코드 전수 검토에서 결함 6종 발견(페이로드 물리 부재·`_payload_attached` 죽은 플래그·마커 env-0 게이팅·ctrl_mass 스칼라·릴리스=즉시 종단·CCIP vz 생략) → ①②③ 수정: 해석적 페이로드를 실제 per-env RigidObject(0.1 kg 실린더)로 교체. **GPU-복제 물리는 per-env 조인트 토폴로지 변경 불가 → kinematic weld 패턴**(부착 env만 매 physics step pose+vel write, CCIP 발화 → release_delay 카운트다운 → write 중단 = 자유낙하, z≤0.10 m 측정 착탄 래치). hover-drop 강제 릴리스 검증(8 envs, isaac-verify): 부착 추적 **1.1 mm**, 분리 0/8 잔류, 착탄 8/8, **측정 vs 해석적 CCIP |Δ| mean 0.012/max 0.021 m** — 물리↔해석 cm-parity 계측 증명. 보상·종단·referee bit-identical(순수 추가), `payload_impact_rate`/`payload_impact_err_measured_m` 지표 신설. 후속(별도 지시): 에피소드 착탄-연장, Phase-2 DR 힘 정합, vz 복원, per-env ctrl_mass. → [[experiments/exp_019_physical_payload]] / [[research/physical_payload_attach]] / Rule 24
- **(이전 2026-07-13) 병행 트랙 — Isaac Lab: exp_015 이어학습(2차) 완주 — P2/P3 extended 각 +2000 iters (P2_EXIT=0, P3_EXIT=0).** §7 baseline 체크포인트에서 페이즈별 단독 연장(`isaac-verify`, 2048 envs, `release_terminal` 미적용). **P2 ext(iter 1098→3097):** drop tail **2.87 m**(1차 2.91 m 대비 ~0.04 m 개선), best_min 0.008 m 스파이크, release_rate **0.33→0.01** 급락, success **0**. **P3 ext(iter 3097→5096):** drop tail **5.31 m**(1차 3.20 m **회귀**), reward 101.7→**74.5**, lead tail 평탄 0.35, success **0**. **0.8 m 돌파 ❌** — iter 예산 확대만으로는 릴리스-종단 명중 미형성(Rule 20f). exp_018 `release_terminal` 종단 재구조 필요 재확인. 산출물: `logs/exp015_cont/` (`exp015_phase{2,3}_ext_final.pt`, `summary_p{2,3}_ext.json`). → [[experiments/exp_015_phased_curriculum]] §8 / [[research/curriculum_phase_convergence]] §2(e) / Rule 20f
- **(이전 2026-07-12) 병행 트랙 — Isaac Lab: exp_015 실학습 — Phase 1→2→3 커리큘럼 첫 end-to-end 완주 (baseline, ORCH_EXIT=0).** 2048 envs·600/500/500 iters·seed 42·~65 min(~2.3 s/iter). plain `--phases 1,2,3`(release_terminal·w_aim 미사용 = exp_015 원본 릴리스 메커니즘). **Phase 1만 완전 수렴**: success 0.48→**1.00**, reward→107(exp_014 100% 재현). **Phase 2**(CCIP+Residual+DR): reward −0.8→**94.7** 회복, `drop_impact_error_m` **4.66→2.91 m ↓**, release_rate peak 0.98/tail 0.33(변동), success ~0(착탄 2.9 m ≫ 반경 0.8 m). **Phase 3**(이동타겟): reward→**102**, `lead_error_m` best **0.071 m**(tail 평탄 0.34), release 0.10, success ~0. warm-start 무손실 실증(페이즈 경계 reward 딥→빠른 회복, Rule 20e). **정직 평가: reward 우상향은 proximity 지배 — P2/P3 릴리스-종단 명중 능력은 베이스라인 500 iter로 미형성(추가 학습 또는 exp_018 release_terminal 구조 필요). exp_016/017의 "근접≠릴리스" 커리큘럼 스케일 재확인.** ckpt/로그/수렴그래프 host 영속화(`logs/exp015_orch/`). → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]] / Rule 20
- **(이전) 병행 트랙 — Isaac Lab: exp_018 Stage B — 릴리스-종단 구조로 release_rate 5.5% → 100% (Rule 23).** 근접 종단(d_xy≤0.8)을 릴리스-발화 종단으로 교체(`release_terminal` cfg, 실패 게이트·타임아웃 불변) + aim_err 보상 nominal-전용 고정(residual 채굴 차단 명문화). **종단 교체 단독(B0, 보상=Stage A v1 그대로): 학습 내 release_rate 23→99.6% 단조 상승**(Stage A의 단조 하락 반전 — Rule 22a 인과 확정), det 200-ep **100.00%**, drop err **0.125 m**(max 0.198), 호버-드롭 프로파일(종단 속도 med 0.11 m/s). 보상 스윕(단일 노브): w_aim 0/1.0/1.5 전부 100%(aim 항 사실상 잉여 — 자동 발화 referee가 노이즈를 +100 샘플러로 전환), knee 0.75만 근소 열화(98.5%). 적대 검증이 done-flag alias 버그 사전 발견(eval success 0% 위험, `.clone()` 수정, Rule 23d). 구조 요인 3종(잘림/노이즈/할인) 전부 해소·역전. **Stage C(DR+residual, 별도 지시) warm-start = `exp018_B0_final.pt`.** → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] / Rule 23
- **(이전) exp_017 Stage A — 밀집 CCIP 조준 보상(보상-변경-단독)은 release_rate를 못 올림 → 판정 (b)·Rule 22.** exp_016의 6% 갭에 대한 1차 개입: `w_aim·(1-tanh(aim_err/s))` dense 항(referee와 동일 aim_err, cfg 기본 off, 5-lens 적대 검증 후 학습). **HEAD 6-dim P1 기준선 신규 학습**(exp_015는 스모크만·exp_014 ckpt는 4-dim): 학습 내 release_rate 12→3.7% **단조 하락**(근접 최적화가 릴리스 능력을 능동 파괴), det 2.5%. v1(w=1/knee 0.5): det **5.5%**, aim_err_min med 1.15→**0.89 m**, speed 3.35→**2.72 m/s**(방향 실재, n=200 p≈0.13). v2(w=2/knee 1.0, 600 it): **회귀**(3.5%/1.10 m/3.45 m/s) — σ 1.55 폭등, 수동 소득화. 구조 원인 3종(γ-할인 완주 보너스·CCIP 노이즈 증폭 ×1.5 s·성공 조기 종단) → **릴리스는 Phase 2 종단 구조로**(w_aim은 Phase 2+에 그대로 이월 금지 — residual 채굴 경로). 체크포인트 3종 분리 보존(+호스트 `/opt/drone-bombard/checkpoints/exp017/`), farm 시그니처 0. → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] / Rule 22
- **(이전) exp_016 — "success 100% vs drop_impact_error 4.59 m" 디커플링 규명·수정 (Rule 21).** Phase 1엔 릴리스 트리거가 없었고(`DropCfg.release_tolerance=0.2` 정의만·미사용), `drop_impact_error_m`이 **d_xy-성공 종단 스냅샷**(잔여속도 ~3.0 m/s 포함)의 탄도 예측 = **속도 캐리 3.0×1.53 s ≈ 4.6 m**(투하 오차 아님). 수정: 스크립티드 CCIP referee(≤0.2 m 최초 충족 시 래치, **지표 전용 — 보상/종단 bit-identical**) + `drop_impact_error_m`=릴리스 시점 재정의(구 지표는 `_terminal_m` 보존) + `release_rate`/`aim_err_min` 신설. **A2 200-ep 재평가: 발화 시 0.137 m, 단 release_rate 6%(10 Hz)/11.5%(100 Hz) — CCIP 스윕 최근접 med 0.755 m ≈ d_xy_min 0.665 m(경로 cross-track 지배).** 근접 학습 정책은 릴리스 능력이 없다 → exp_015 Phase 2 몫. exp_013의 24 m도 동일 의미론(디커플링은 exp_012 도입부터 구조적). → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] / Rule 21
- **(이전) exp_015 — Phase별 순차 커리큘럼 코드 완료(미학습) (`feat/isaac-env-migration`).** 이미지 3단계(접근/nominal → CCIP+Residual/정지타겟 → 이동타겟)를 완전 구현. action 4→6(δx/δy CCIP residual), `phase` 단일 노브+파생 플래그, 릴리스 이벤트(nominal CCIP+δ 예측 트리거 → 실제 DR 낙하 착탄오차 터미널 보상), drag/wind 도메인랜덤화, Gauss-Markov 이동타겟+lead 보상, `train.py --phases 1,2,3` 서브프로세스 오케스트레이터(6-dim 고정 → `runner.load()` warm-start 무손실). **로컬 `py_compile` 12파일 통과**, `pytest`(+8 신규)·본 학습은 L4/컨테이너 대기(dev 박스 torch 없음). Phase 1 = exp_014 baseline과 동작 동일. → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20
- **(이전) exp_014 완료 — plant 수정 + 비전 거리감쇠 → deterministic 200-ep eval 100.0% (202/202), d_xy_min 0.665 m (exp_013: 36%/1.4 m).**
  - **plant 수정 3종** (커밋 `cd0c617`/`7d0e9b6`): ①속도킥 → 스폰타임 `UsdPhysics.MassAPI` authoring (`--zero-actions` 11.9 m FAIL→0.2 m PASS) ②로터 ±200 rad/s 리셋 재주입 제거 ③**inertia 대반전** — `set_inertias`는 solver에 전파되고 있었음(`_diag_inertia.py` 계측): **exp_013은 rate loop ~1300× 저토크 plant에서 학습**, 구 정책은 plant-overfit(재평가 bad_att 68%) → [[research/isaac_inertia_ctrl_mismatch]] / Rule 19 신설.
  - **exp_014 A2** (감쇠 ON, `v3qk07pg`): R_alt(300-400)=**0.0000**, success 99.85%, noise_std 0.80 안정(폭주 없음 — Rule 18b 재해석: σ 폭주도 plant 아티팩트). climb 창발(iter 150-199, 27.8%) 후 50 iter 내 완전 기각. **A0′** (감쇠 OFF 대조, `azoc1xp0`): R_alt 0.0365, success 96.5% → **지배 요인 = plant 일관성, 감쇠 = 잔여 꼬리 제거 + YOLO parity(유지)**. 상세: [[experiments/exp_014_A2_visionrange]].
  - 실 YOLO 캘리브레이션은 컨테이너 annotator 버그로 차단(하네스는 수리 완료, 커브는 분석값 — calibration-pending). reward_success·entropy_coef 불변(다음 페이즈, [[research/isaac_ppo_tuning_recommendations]]).
- (이전) **exp_013 — 첫 프로덕션 PPO 학습 완주 + 진단 완료 (2026-07-03).** 2048 envs×1000 iters(65.5M steps, 43분, wandb `wcjklw7a`) → **deterministic 200-ep eval = 36%**, d_xy_min 1.4m plateau. 시작 직후 **비전 사멸 버그**(env-origin 프레임 혼용, [[errors/err_20260703_vision_env_origin_frame]]) 발견·수정 후 재기동. 실패 원인 3종 규명: ①analytic conf 거리감쇠 누락→고도 상승 farming(max_alt 33%, Rule 17) ②farmer(+225)>finisher(+121) 보상 불균형(Rule 18a) ③noise_std 0.8→3.92 폭주(Rule 18b). **07-04 forensics 재정정: 리셋 속도킥은 프로세스당 1회(첫 물리 substep, m_eff=0.02504kg 계측 확정)로 실증 — 학습 오염 사실상 없음, max_alt 27-43%는 iter ~200 창발 학습된 attractor(비전 farming 1차 가설 복권), 36% 수치 유효.** entropy 실측: 실행 속도 궤적 σ-불변(vel-Δ σ3.9/det=1.01×) → 공짜 entropy 확정. **다음: exp_014 = conf 거리감쇠 + reward_success 300 + entropy_coef 0 + 킥 위생수정, fresh ([[research/exp014_ablation_protocol]] 참조).** 온보딩 문서 3종(`isaac_lab_reward_tuning`/`wandb_guide`/`experiment_workflow`) 신설. 상세: [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]].
- **메인 트랙 — SAC (jekyun 브랜치, Gazebo/PX4):**

## 현재 상태 (2026-07-05, Gazebo/SAC 트랙 마지막 갱신 — 이후 이 트랙은 미진행)

- **알고리즘:** SAC, `net_arch=[256,256]`, L4 GPU
- **현재 학습:** ⏸️ `rl_yolo_v15_bc_stable` — 반복 reset-recursion abort로 오토레쥼 서포바이저(`run_train_supervised.sh`) 추가 후 0→310K. **정정(07-05): 310K 정지는 계획된 stop이 아니라 07-03 GPU 드라이버 업그레이드(Isaac Lab용, 535→580)로 `drone-bombard-harmonic` 컨테이너가 깨지며 강제 중단된 것.** **사용자 관찰: 훈련된 에이전트가 X마커에 도달 못 함(v14 대비 회귀).** 원인 후보: (1) 근접-속도 댐핑(`w_vel`, `vel_damp_radius=3.0m`)이 v14의 final-approach stagnation 구간(0.5–1.2m)을 재타격, (2) crash-resume마다 replay buffer 초기화로 학습 불연속 — **둘 다 미확정, 인프라 정합 및 재평가 없이 이 트랙은 여기서 멈춤.** → [[daily/daily_2026-07-05_gazebo_v15_regression]] / [[daily/daily_2026-07-07]] / Rule 25 / Rule 26
- **이번 세션 (2026-07-01) — RL wobble 진단·교정:** 사용자 관찰(10 m 핸드오프 후 RL 인수하나 wobble). eval `deterministic=True`라 **탐험 노이즈 아님 = 학습된 bang-bang 정책.** LPF A/B: PX4 수신 속도명령 **jerk RMS 2.92→1.61(−45%)**, 평균 속력 1.13×(안 느려짐) → **smoothness-control 문제 확정.** 교정: (B) 근접-게이팅 속도 댐핑 `w_vel=0.15/R=4`, (C) `w_ang_vel 0.05→0.15`·`w_action_smooth 0.05→0.20`, 로직 LPF `velocity_lpf_alpha=0.4`(학습==배포). dry-run PASS → v15 fresh 기동. → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15
- **이번 세션 (2026-07-01) — RL wobble 진단·교정:** 사용자 관찰(10 m 핸드오프 후 RL 인수하나 wobble). eval `deterministic=True`라 **탐험 노이즈 아님 = 학습된 bang-bang 정책.** LPF A/B: PX4 수신 속도명령 **jerk RMS 2.92→1.61(−45%)**, 평균 속력 1.13×(안 느려짐) → **smoothness-control 문제 확정.** 교정: (B) 근접-게이팅 속도 댐핑 `w_vel=0.15/R=4`, (C) `w_ang_vel 0.05→0.15`·`w_action_smooth 0.05→0.20`, 로직 LPF `velocity_lpf_alpha=0.4`(학습==배포). dry-run PASS → v15 fresh 기동. → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15
- **이전 학습:** ⏸️ `rl_yolo_v14_softreset` (byxyaf4d) — 196.5K/500K에서 SIGINT stop, `rl_checkpoints/v14_backup/`에 보존. v13(iyhfy5ps 157.7K)는 `rl_checkpoints/archive/v13_iyhfy5ps_157k_20260622`에 백업.
- **이번 세션 (2026-06-23) — v14 195K eval = 65%(13/20):** plateau(70K부터 reward 평탄) 확인 후 stop → clean 20-ep deterministic eval **65%** (v13 80% 대비 회귀). 실패 7 전부 final-approach stagnation(0.5–0.8m, 0.50m gate 직전), EKF 귀책 0. **Soft reset 장기검증 ✅**(3096 resets, soft ~91%, EKF bounded, no teleport) → Rule 14 검증완료. 회귀=정책 미성숙(39% budget). 비디오 3/3 success 캡처(`rl_eval_results/v14_195k_flight_annotated.mp4`+`_raw.mp4`). v14 commit 결정 대기. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- **이번 세션 (2026-06-22) — 리셋 처리량 ~3.9× (soft reset):**
  - 문제: v14 fps≈2, ETA ~2.5일. 에피소드마다 CRUISE timeout(~42s) + full restart(~22s).
  - **근본원인 규명:** teleport+disarm 후 PX4 **EKF 추정기 재수렴** 대기(`pre_flight_checks_pass=False`). fresh restart도 동일.
  - **EKF2_GPS_CHECK 0 A/B = 음성**(COM_ARM_WO_GPS라 GPS는 게이트 아님; 실제 게이트=EKF 수렴, 바이패스 param 없음). param/timeout 레버 고갈.
  - **Soft reset(teleport 회피) = 성공:** flyable이면 날아서 출발점 복귀 후 FSM만 재시작. **throughput 0.93→3.61/min(~3.9×), fps 2→9, reset 65s→11s, soft 100%(32/32), EKF d_xy 안정 4.5–5.8m(발산 없음).** ETA ~2.5일→~15h.
  - **코드 미커밋**, full run으로 장기 검증 중(EKF drift bounded? fallback율?).
  - 상세: [[experiments/exp_009_softreset_throughput]] / [[research/reset_throughput_bottleneck]] / Rule 14
- **이전 (2026-06-22) — 핸드오프 윈도우 확장:**
  - 사용자 요청: "X마커가 늦게(거의 머리 위) 탐지돼 RL 핸드오프 후 학습 윈도우가 짧다." 가설 = 순항 고도↑.
  - **고도만 10 m(v1 dry-run): 실패.** clean 핸드오프 여전히 d_xy 2.7 m(베이스라인 동급) + 순항-시작 spurious(conf=0.00, d_xy≈11 m) → health gate abort. 원인: 마커 apparent size 절반(YOLO 늦게 lock) + 200 px 필터가 핸드오프를 머리 위로 클립. **고도는 레버 아님.**
  - **10 m + 탐지 게이트 수정(v2): 성공.** `vision_callback` conf 게이트(`min_detection_conf=0.5`) + 공간 필터 200→300 px(`detection_pixel_radius`). **핸드오프 2.7→5.0 m(윈도우 ~2배), spurious 0, EKF-drift 0.** VISION 로그: FP conf 0.29–0.45 reject / real 0.73–0.95 accept.
  - **코드 변경 미커밋**(사용자 직접 커밋 예정). 보상 공식 아님(기하+탐지) → fresh 필수 아니나 초반 재적응 예상.
  - 상세: [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13
- **이전 세션 (2026-06-20) — v13 정책 평가:**
  - 학습 점검: 157.7K(~32%), **ep_rew_mean ~100 plateau(80K부터 평탄), success ~82%, target_lost 0**, fps≈0(restart 병목).
  - plateau 확인 → eval 위해 학습 SIGTERM stop → deterministic eval(`sac_drop_preempt.zip`, 20-ep 요청/13 실행).
  - **정책 양호:** 깨끗하게 시작한 ep 1–3 전부 0.8m 성공(reward 126/114/132, step 41–72; 평균 124 > 학습 ~100).
  - **eval EKF divergence 흡수 루프:** ep 4–13 전부 step1 `d_xy≈11.9m`(=home→target) → −15 truncation. 카메라는 마커 봄(TRACKING OK)이나 EKF position만 발산. 연속 full-restart가 EKF 수렴 못 시킴 → 자체 회복 불가. **정책 아님, 시작 상태 결함** (06-17 EKF 재수렴과 동일 뿌리).
  - **harness 결함 2종:** `evaluate.py` miss-distance/CEP=NaN(env 미emit 키); v13 env는 0.8m 종료(탄도 투하 없음) → CEP 비실재.
  - **다음:** 에피소드 시작 EKF↔카메라 health gate(drift면 retry) + `evaluate.py` 지표(success-rate/step) 교체 후 재평가.
  - 상세: [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12

### 이전 세션 (2026-06-17) — v13 처리량 병목 진단 & 수정
- **이번 세션 (2026-06-17) — v13 처리량 병목 진단 & 수정:**
  - v13(46y4xtiw) ~10h에 29.9K(6%)뿐, fps≈0.83, ETA ~6.5일. 지배적 싱크 = `PX4 not armed after 10s` bail.
  - **진단(armdiag dry-run, xgzum51v):** 컨트롤러에 `PREFLIGHT-PASS` dt 계측 추가 + `arm_bail_timeout=25s`.
    EKF 재수렴이 **bimodal — 0.0s(7/12) 또는 13–16s(5/12 ≈ 42%)**. 25s 창에서 **bail 0 / SUCCESS 4**.
  - **결론:** v12의 `arm_bail_timeout=10s`가 복구(13–16s) 직전에 멀쩡한 PX4를 단두대질 → full restart 강제.
    stuck-EKF는 *full-restart-only가 아니라 recoverable-with-time*.
  - **Fix:** `hyperparams_v13.yaml` `arm_bail_timeout: 10.0 → 20.0`.
  - **⚠️ 인시던트:** armdiag dry-run이 **YAML 중복 `checkpoint_dir` 키**로 메인 dir에서 실행 → v13 30K 체크포인트 파괴(복구 불가). → fresh 재시작(iyhfy5ps). [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]]
  - **프로덕션 검증(iyhfy5ps):** bail 0, late-EKF 14.1/14.8/15.5s ×3 전부 회복(구 10s면 bail).
  - 상세: [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / [[research/cruise_timeout_arming]] / Rule 11
  - ⚠️ **여전히 OPEN:** YOLO `target_lost_rate` ~29%; teleport 후 EKF 13–16s 재수렴 자체 (별도 과제)

### 이전 세션 (2026-06-16)
- **방식:** Vision 기반 — YOLO X마커 탐지 → TRACKING, RL이 시각 서보잉 학습
- **이번 세션 (2026-06-16) — v12 정체 진단 + v13 종단 보상 재설계:**
  - v12(93K): ep_rew≈-20.5(추세 없음), ep_len 40→16, success **0회**, d_xy~1.0m는 잘 도달
  - 근본 원인: **종단 overshoot 트랩** — 정하방 카메라(commit 24135e9)로 핸드오프가 ~1m가 되면서
    overshoot 가드가 step 1부터 무장 + 8 m/s 액추에이터가 0.5m 성공원을 지나침 → 매 에피소드 -20
  - v11→v12 회귀 (v11은 전방 카메라로 접근 활주로 확보 → 404 successes)
  - **v13 수정 (config-only, Fresh Start 필요):** overshoot threshold 1.5→0.6, success_radius 0.5→0.8,
    action vx/vy 8/5→4/3, w_proximity 0.3→0.6, radius 5→2 → `hyperparams_v13.yaml`
  - 상세: [[research/terminal_overshoot_trap]] / Rule 10
  - ⚠️ **여전히 OPEN:** YOLO `target_lost_rate` ~29% bimodal — 미해결
- **선행 (2026-06-15):** Arming throughput fix — teleport 후 stale EKF arm 거부 수정 → [[research/cruise_timeout_arming]]
- **Phase 1 계획:** CCIP 기반 자율 접근 비행 제어기 → [[research/phase1_plan]]

---

## 실험 현황

| # | Run ID | Steps | 상태 | 비고 |
|---|--------|-------|------|------|
| 001 | 8otphxy8 | 114K | ✅ 완료 | 선형 보상 + CRUISE retry |
| 002 | — | 0 | ✅ 완료 | 보상 패치 적용 (학습 없음) |
| 003 (dry-run) | mtx7ud6o/x8jq9fsy/u8w3xn0w | 5500×3 | ✅ 완료 | RTF 1/2/4 비교 → RTF=2 최적 |
| 004 | esmtny0a | 33K+ | ✅ 폐기 | Vision YOLO 접근 + EKF East 버그 수정 → [[experiments/exp_004_rl_yolo_debug_vision]] |
| 005 | rl_yolo_v12_arm_fix | 0→500K | ⛔ 중단 | Arming fix는 작동하나 종단 overshoot 트랩으로 success 0 → v13으로 대체 → [[research/terminal_overshoot_trap]] |
| 006 | rl_yolo_v13_terminal_reward (46y4xtiw→iyhfy5ps) | 0→500K | 🔄 fresh 재시작 | 종단 보상 재설계. 46y4xtiw 30K에서 처리량 진단 위해 stop → ⚠️ armdiag dry-run이 30K 체크포인트 파괴 → **fresh 재시작 iyhfy5ps (arm_bail=20)** → [[research/terminal_overshoot_trap]] / [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] |
| 006b (dry-run) | v13_armdiag_dryrun (xgzum51v) | 1000 | ✅ 완료 | arm_bail 진단: EKF 재수렴 bimodal(0s/13–16s), 25s에서 bail 0. **Fix: arm_bail_timeout 10→20** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11 |
| 007 (eval) | iyhfy5ps (157.7K 평가) | 13 ep | ✅ 완료 | deterministic eval. 유효 3-ep 100% 성공(reward 124>학습 ~100); ep 4–13 EKF divergence 흡수 루프(시작 상태 결함). harness: evaluate.py NaN, v13 탄도 투하 없음 → [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12 |
| 008 (dry-run) | dryrun_alt10 (uqy7lmny / _gated, offline) | 1500×2 | ✅ 완료 | 핸드오프 윈도우↑. 고도만 10 m=실패(레버 아님), 10 m+탐지 게이트(conf 0.5 + 200→300 px)=성공(핸드오프 2.7→5.0 m, spurious 0). 미커밋 → [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13 |
| 009 | EKF A/B + softreset (byxyaf4d) | proto+full | ✅ 완료 | 리셋 처리량 ~3.9×. EKF param A/B=음성, **soft reset(teleport 회피)=성공**(0.93→3.61 handoffs/min, fps 2→9, reset 65s→11s, soft 100%, EKF 안정). → [[experiments/exp_009_softreset_throughput]] / [[research/reset_throughput_bottleneck]] / Rule 14 |
| 010 (eval) | rl_yolo_v14_softreset (byxyaf4d, 195K) | 20 ep | ✅ 완료 | **195K eval = 65%(13/20)**, v13 80% 대비 회귀(실패 전부 final-approach stagnation). EKF 귀책 0. **Soft reset 장기검증 ✅**(3096 resets, soft ~91%, EKF bounded) → Rule 14 검증완료. 비디오 3/3 success 캡처. commit 결정 대기. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]] |
| 011 | wobble A/B + v15_bc_stable | eval×2 + dry-run + 0→300K | ▶️ 학습 중 | **RL wobble = smoothness-control 문제 확정.** LPF A/B: 속도명령 jerk RMS 2.92→1.61(−45%), 평균 속력 불변. 교정(B 근접 속도 댐핑 + C smoothness↑ + LPF 0.4) dry-run PASS → v15 fresh 기동(v14 백업). → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15 |
| 012 | isaac_migration_phase2 (`feat/isaac-env-migration`) | 코드만 (미학습) | ✅ 코드 완료 | **Isaac Lab env+PPO 이식.** `isaac_lab/` 신설, v13/v15 obs·action·리워드·터미네이션 상수 그대로 포팅(parity 표 포함), SAC→PPO(rsl_rl), target/spawn 랜덤화 신규, vision=analytic+YOLO-eval 이원화. `pytest test_math.py` 29/29 통과. L4 VM 미기동 → env 스모크 미실행. → [[experiments/exp_012_isaac_migration_phase2]] / [[research/isaac_velocity_controller]] |
| 013 | exp013_v2_visionfix (wcjklw7a, Isaac PPO) | 65.5M steps (2048 envs×1000 iters) | ✅ 완료 | **Isaac 첫 완주 학습 + 200-ep deterministic eval = 36%.** v1은 비전 사멸 버그(env-origin 프레임 혼용)로 중단·수정 후 재기동. 곡선 plateau(iter ~700), d_xy_min 1.4m 정체. 실패 분해: max_altitude 33%(analytic conf 거리감쇠 누락 → 상승 farming, Rule 17) + crash 27%; farmer>finisher 보상 불균형(Rule 18a); noise_std 0.8→3.92 폭주(Rule 18b); **사후 --zero-actions FAIL(11.9m) → 리셋 속도킥이 run 전체 오염(§4d, 1차 용의자).** **다음: exp_014 = 0순위 킥 수정 → conf 거리감쇠+success 300+entropy 0, fresh.** → [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]] / [[errors/err_20260703_vision_env_origin_frame]] |
| 014 | exp014 A2 (v3qk07pg) + A0′ (azoc1xp0), Isaac PPO | 각 26.2M steps (2048 envs×400 iters) | ✅ 완료 | **plant 수정(킥·로터·inertia) + 비전 거리감쇠 → deterministic 200-ep eval = 100.0% (202/202)**, d_xy_min 0.665m. A2 R_alt=0.0000(창발-기각), A0′ 대조로 귀속 분리(지배=plant 일관성). → [[experiments/exp_014_A2_visionrange]] / [[research/isaac_inertia_ctrl_mismatch]] |
| 015 | isaac_phased_curriculum (`feat/isaac-env-migration`) | 코드만 (미학습) | ✅ 코드 완료 | **Phase별 순차 커리큘럼 구현.** 이미지 3단계(접근/nominal → CCIP+Residual/정지 → 이동타겟) 완전 구현: action 4→6(δ residual), phase 단일 노브+파생 플래그, 릴리스 이벤트(nominal CCIP+δ 트리거 → 실제 DR 낙하 착탄오차 터미널 보상), drag/wind DR, Gauss-Markov 이동타겟, lead 보상, `train.py --phases 1,2,3` 서브프로세스 오케스트레이터(6-dim 고정 → warm-start 무손실). `py_compile` 12파일 통과, pytest는 L4/컨테이너 대기. → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20 |
| 015-train | exp015 실학습 (baseline, Isaac PPO, tensorboard) | 2048 envs × 600/500/500 iters (~104M steps, ~65 min) | ✅ 완주 (ORCH_EXIT=0) | **첫 커리큘럼 end-to-end. Phase 1만 완전 수렴.** P1 success 0.48→1.00·reward→107(exp_014 재현); P2 reward −0.8→94.7·drop 4.66→2.91m↓·release peak 0.98/tail 0.33·success~0; P3 reward→102·lead best 0.071m(tail 평탄)·success~0. warm-start 무손실 실증(Rule 20e). reward 우상향=proximity 지배 — P2/P3 명중 능력 미형성(추가 학습 또는 exp_018 종단구조 필요). ckpt/로그/그래프 host 영속화. → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]] / Rule 20 |
| 015-cont | exp015 이어학습(2차) P2/P3 ext (+2000 iters each) | P2: 1098→3097 · P3: 3097→5096, 2048 envs, ~3h | ✅ 완주 (P2_EXIT=0, P3_EXIT=0) | **iter 예산 확대 검증 — 0.8m 돌파 ❌.** P2 ext: drop 2.87m(정체), release 0.33→0.01 급락, success 0. P3 ext: drop 5.31m(회귀), reward 74.5, success 0. 구조 개입(exp_018 release_terminal) 필요(Rule 20f). → [[experiments/exp_015_phased_curriculum]] §8 / [[research/curriculum_phase_convergence]] §2(e) |
| 016 | exp016_ccip_release_reeval (eval-only, A2 ckpt) | 200-ep deterministic eval | ✅ 완료 | **4.59 m 디커플링 규명 = 지표 의미론 버그(Rule 21).** 릴리스 트리거 부재 → 지표가 성공-종단 잔여속도(3.0 m/s)의 탄도 캐리 측정. CCIP referee(≤0.2 m) 수정 후: 발화 시 0.137 m, release_rate 6%/11.5%(10/100 Hz), aim_err_min med 0.755 m ≈ d_xy_min(cross-track 지배). 구 지표 4.649 m 재현 ✓. 보상/종단 bit-identical(지표 전용). → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] |
| 017 | exp017 Stage A (750gpldr/6z0gpnhy/fv5qqmtz, Isaac PPO) | 3 runs × 26–39M steps | ✅ 완료 — 판정 (b) | **밀집 CCIP 조준 보상(보상-변경-단독)은 release_rate 못 올림.** P1 6-dim 기준선 신규 학습(det 2.5%, 학습 내 12→3.7% 단조 하락 — 근접 최적화가 릴리스 능력 파괴). v1(w=1): det 5.5%·aim 0.89 m·speed 2.72(방향 실재, p≈0.13). v2(w=2/knee 1.0): 회귀. 원인 3종=γ-할인 완주 보너스·CCIP 노이즈 증폭·성공 조기 종단 → 릴리스=Phase 2 종단 구조 몫. ckpt 3종 분리 보존. → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] / Rule 22 |
| 018 | exp018 Stage B (xt0hrr1c/0ns10yso/4vaodj0o/kk06wsbx, Isaac PPO) | 4 runs × 400 iters (v1 warm-start) | ✅ 완료 | **릴리스-종단 구조 → det release_rate 100.00%, drop err 0.125 m.** 종단 교체 단독으로 5.5%→100%(학습 내 23→99.6% 단조 상승 — Rule 22a 인과 확정). aim 보상 노브 불감(w 0/1.0/1.5 전부 100%; knee 0.75만 98.5%) — 자동 발화 referee가 노이즈를 +100 샘플러로 전환. done-flag alias 버그 사전 수정(Rule 23d). 호버-드롭 수렴. Stage C warm-start=B0. → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] / Rule 23 |
| 020 | exp020 물리 페이로드 부착 학습 (o5jn9xzk/vryuc6mu, Isaac PPO) | 400 iters (B0 warm-start, 보상 bit-match) | ✅ 완료 | **물리 페이로드의 학습 비용 = 0 — det success/release 100.00%, drop err 0.169 m.** release_rate 첫 롤아웃부터 100%(재학습 과도기 없음), exp_019 parity의 학습-스케일 확증. σ 드리프트 1.41→1.71 모니터 대상. **wandb eval-figure 파이프라인 신설**(`play.py --wandb`). 컨테이너 빈 WANDB_API_KEY 함정 → `--env-file` 필수. → [[experiments/exp_020_o5jn9xzk_payload_training]] / [[errors/err_20260723_wandb_key_empty]] |
| 021 | exp021 v19 + 이동 타겟 CV/CT/CA (a6saa42b/29jqq1lu/ntumqwoz, Isaac PPO) | 3 runs × 1000 iters (준상 v19 precise 사본 warm-start) | ✅ 완료 (det eval 포함) | **이동 타겟 모션을 v19에 obs-보존 포팅 → warm-start 학습 3종.** `V11Env._step_moving_target()` + V11/V16/V19 wire, obs 28-D 불변(lossless 로드 실증). det 200-ep: cv success 44.5%/drop med 0.775 m · ct 33.8%/0.783 · ca 16.5%/1.063 — 리드 부재가 1차 병목(개선안 §3c). `--target_kf`+v-track은 명시적 에러(KF는 base env 전용). → [[experiments/exp_021_v19_moving_target]] / [[research/moving_target_models]] §5 |
| 022~024 | (exp_022 / exp_023 / exp_024) | — | ✅ 완료 | P0 랜덤화 env · Table 1 1차 · v20 warm-start 음성 결과 → 각 노트 참조 |
| 026 | — (무학습 스크립트 베이스라인) | 64 ep × 4 + 128 ep × 4 | ✅ 개선 확인 | **릴리즈 판정 10 → 100 Hz + 성공 반경 1.0 → 0.5 m.** T2 CEP50 0.762 → **0.395 m**(타이밍 성분 제거). 새 Table 1(n=128, r=0.5): T0 65.6% / T1 53.1% / T2 53.1% / **T3 71.9%**. ⚠️ T1≡T2 · 오라클 갭이 CEP50 0.023 m / **CEP90 0.206 m / 성공률 18.8 pp**로 꼬리 이동 → [[experiments/exp_026_release_rate_100hz]] |
| 025 | — (무학습 스크립트 베이스라인) | 128 ep × 8 run | ✅ **게이트 통과** | **DR_SCALE 스윕 유효성 검증** — T2 CEP50 단조↑(0.570→0.815) · T3 평탄(0.573→0.491) · **오라클 갭 −0.003→0.325 m 단조↑**. scale 0에서 갭 ≈ 0(깨끗한 통제군). 학습 착수 조건 충족. 부수: scale 0에서도 CEP50 0.57 m = 전부 제어·릴리즈 타이밍 → [[experiments/exp_025_dr_scale_sweep_gate]] |

## 에러 현황

| 파일 | 상태 | 요약 |
|------|------|------|
| [[errors/err_20260827_payload_drag_body_frame]] | ✅ 해결 | 페이로드 항력을 월드 프레임으로 계산해놓고 `set_external_force_and_torque`의 `is_global` 기본값(False, 링크 프레임)으로 전달 → 드론 자세만큼 회전된 힘. 에러 없이 통과. 같은 파일 기체 쪽은 올바르게 변환 중이었음 (Rule 31) |
| [[errors/err_20260823_ccip_vz_omission]] | ✅ 해결 | CCIP `ballistic_impact`가 $v_z$ 누락($t=\sqrt{2H/g}$ 특수화) → 릴리즈 엔벨로프 모델오차의 ~70%. 에러 없이 조용히 틀림, 잔차가 흡수해 은폐. `vel_z` 필수 인자 승격 (Rule 30) |
| [[errors/err_20260723_wandb_key_empty]] | ✅ 해결 | 컨테이너 baked-in `WANDB_API_KEY` 빈 값 → wandb 학습 silent 실패(isaaclab.sh exit 0 삼킴). `--env-file /opt/drone-bombard/.wandb.env` 필수 |
| [[errors/err_20260703_vision_env_origin_frame]] | ✅ 해결 | Isaac `_update_vision` env-origin 프레임 혼용 → 벡터화 학습에서 비전 채널 완전 사멸(conf≡0). num_envs=1 검증으론 구조적으로 못 잡음 |
| [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] | ✅ 해결 | armdiag dry-run이 YAML 중복 키로 v13 30K 체크포인트 파괴. 재발방지: startup `Checkpoints:` 로그로 격리 검증 |
| [[errors/err_20260615_cruise-timeout-arming]] | ✅ 해결 | CRUISE 타임아웃 = teleport 후 PX4 arm 거부 (stale EKF) |
| [[errors/err_20260320_physics_explosion]] | ✅ 해결 | ODE 물리 폭발 3중 방어 |
| [[errors/err_20260319_ode_aabb_crash]] | ✅ 해결 | 드론 스폰 고도 ODE AABB 크래시 |

---

## 빠른 참조

| 주제 | 파일 |
|------|------|
| Guacamole/VNC 설정·접속 | `notes/Environment/README.md` |
| 자주 쓰는 명령어 | `notes/sessions/commands.md` |
| RL 규칙 | `notes/research/rl_rules.md` |
| VM 완전 복구 가이드 | `notes/Environment/README.md` |
| 보상 함수 설계 | `notes/research/reward_design.md` |
| 전체 시스템 아키텍처 (Gazebo/PX4/ROS2, `jekyun`) | `notes/research/system_overview.md` |
| 전체 시스템 아키텍처 (Isaac Lab, `feat/isaac-env-migration`) | `notes/research/isaac_lab_architecture.md` |
| Isaac Lab 다른 연구자용 온보딩 (보상/하이퍼파라미터/WandB/실험 절차) | `notes/research/isaac_lab_reward_tuning.md` · `notes/research/isaac_lab_wandb_guide.md` · `notes/research/isaac_lab_experiment_workflow.md` |

---

## 폴더 구조

| 폴더 | 용도 | 네이밍 규칙 |
|------|------|------------|
| `research/` | 이론·설계·아키텍처 | `{topic_slug}.md` |
| `experiments/` | 학습 실험 (WandB 연동) | `exp_{NNN}_{wandb_id}_{title}.md` |
| `errors/` | 에러 해결 기록 | `err_{YYYYMMDD}_{slug}.md` |
| `sessions/` | 세션별 작업 일지 | `session_{YYYY-MM-DD}.md` |
| `references/` | 논문·문서 메모 | `ref_{slug}.md` |
| `Environment/` | 인프라 설정 백업 | `{service}.{ext}` |

---

## 노트 인덱스

### 연구 (research/)
- [[research/residual_policy_coupling]] — **(09-02) 잔차 회귀기는 물리가 아니라 정책에 결합되어 있다 — 전이하면 순손실, 재적합으로 복구 (Rule 39)**
- [[research/residual_label_efficiency]] — **(09-02) 라벨 1,000개면 이득의 96%. 오라클과의 간극은 데이터가 아니라 정보다 (Rule 40)**
- [[research/residual_observability]] — **(09-01) 관측이 바람을 담고 있는가 — 지도 회귀 $R^2$ 0.44(obs) / 0.61(+tilt) / 0.998(참바람). tilt 채널에 L0 재학습이 필요 없는 이유 (Rule 37)**
- [[research/release_gate_jitter]] — **(09-01) 첫 교차 게이트 앞에서 예측 요동은 계통 편향이 된다 — 정확도와 매끄러움은 별개 요구조건, EMA로 CEP 0.462→0.209 (Rule 38)**
- [[research/research_architecture]] — **(08-23) 최종 아키텍처 v3 — 코드 실측 대조 개정판. 착수 전 필수 수정 B1~B5, DR 축 정정, 관측 프레임 결정**
- [[research/t3_oracle_entrainment]] — **(08-27) T3 오라클이 상한선이 아니었다 — 즉시 엔트레인먼트 가정, 그리고 지배 오차는 바람이 아니라 자기속도 항력 (Rule 31)**
- [[research/ccip_vz_omission]] — **(08-23) CCIP 수직속도 누락 — 잔차가 배우던 것은 바람이 아니라 공식 결손이었다 (Rule 30)**
- [[research/handoff_generalization_p0]] — **(08-03) 고정 초기조건은 표현을 암기시킨다 — 랜덤화 축 분류(표현 vs 강건성), Rule 27**
- [[research/paper_research_plan]] — **(08-02) 논문 연구 계획 — 문헌 지도·차별점 3종·표 7종(베이스라인/잔차 위치/구조 vs 보상/충실도/인지/리드)·실행 순서 P0~P4**
- [[research/research_overview_for_paper]] — **(08-02) 전 연구 통합 개요 — 계보·warm-start 체인·발견 F1~F12·ablation 설계(논문용)**
- [[research/vision_obs_refactor]] — Vision 기반 obs 리팩토링 (GPS 제거, YOLO 전환)
- [[research/phase1_plan]] — Phase 1 CCIP 기반 자율 접근 연구 계획 (8주, 5/8-6/30)
- [[research/reward_design]] — 4-layer 보상 함수 (LaTeX 수식)
- [[research/architecture]] — Method A (1-World-4-Payload) 아키텍처
- [[research/system_overview]] — 전체 시스템 (패키지, 토픽, 좌표계, 브리지) — `jekyun`(Gazebo/PX4/ROS2) 브랜치
- [[research/isaac_lab_architecture]] — Isaac Lab 전체 구조 (`isaac_lab/` 레이아웃, 데이터 흐름, Gazebo 대비 구조 차이) — `feat/isaac-env-migration` 브랜치
- [[research/rl_rules]] — RL 실험 규칙, WandB 메트릭, Known Failure Modes
- [[research/physical_payload_attach]] — **(07-21) 물리 페이로드 kinematic weld 패턴** — per-env 조인트 불가 제약, 분리=write 중단, 물리↔해석 parity 계측, 트레이드오프(팬텀 질량·DR 갈라짐). Rule 24.
- [[research/ekf_east_reversal]] — ⚠️ RETRACTED: "EKF East 반전"은 오진이었음 (실제 PX4 East=+Gazebo East). 정정: [[coordinate-frames]] / [[daily/daily_2026-06-14]]
- [[research/rtf_fps_analysis]] — RTF vs FPS 분석. RTF=2 최적, Python 루프 병목 규명
- [[research/cruise_timeout_arming]] — CRUISE 타임아웃 = teleport 후 EKF 재수렴 bimodal(0s/13–16s). **06-17 정정: v12의 10s 컷이 복구 직전 단두대질 → arm_bail 10→20s** (Rule 11).
- [[research/terminal_overshoot_trap]] — v12 종단 정체 = overshoot 해자 트랩 (정하방 카메라 핸드오프 ~1m). v13 보상 재설계.
- [[research/eval_terminal_env_metrics]] — v13 eval EKF divergence 흡수 루프 + evaluate.py 지표 비정합 (탄도 투하 없음 → CEP 비실재). 시작 health gate 필요 (Rule 12).
- [[research/detection_gate_vs_altitude]] — 핸드오프 윈도우의 진짜 레버 = 탐지 게이트(conf + 공간 필터), 고도 아님. 고도↑는 마커 가시성 깎아 역효과 (Rule 13).
- [[research/reset_throughput_bottleneck]] — 리셋 병목 = teleport 후 EKF 재수렴(param으론 못 고침). soft reset(teleport 회피)으로 ~3.9× (Rule 14).
- [[research/control_smoothness_wobble]] — RL 인수 후 wobble = smoothness-control 문제(정책, 탐험 아님). LPF+근접 속도댐핑+smoothness 가중 (Rule 15).
- [[research/isaac_velocity_controller]] — Isaac Lab 캐스케이드 속도 컨트롤러, PX4 게인 매핑. **PX4 대비 미검정**(게인 초기값) — 7-포인트 스텝응답 검정 계획.
- [[research/isaac_lab_reward_tuning]] — Isaac Lab 보상·하이퍼파라미터 레퍼런스 (다른 연구자용 온보딩). cfg 필드별 의미·튜닝 시 주의(overshoot moat, fresh-start 판단 등).
- [[research/isaac_lab_wandb_guide]] — Isaac Lab WandB 메트릭 가이드. `Episode_Termination/*`·`Episode_Reward/*` 등 신규 네임스페이스, Gazebo 트랙과 대조표. 첫 실 학습 전이라 rsl_rl 표준 키(§5)는 미검증 표시.
- [[research/isaac_lab_experiment_workflow]] — Isaac Lab 실험 실행 절차 (dry-run 사다리, fresh/resume 판단, WandB run 관리, 실험 로깅).
- [[research/isaac_ppo_tuning_recommendations]] — exp_013 결론: 무엇을 바꿔야 하는가 (conf 거리감쇠·reward_success 300·entropy 0 우선; 스폰 고도는 유지). Rule 17·18의 근거 문서.
- [[research/exp014_ablation_protocol]] — exp_014 ablation 설계(07-04): 킥 vs attractor 원인 분리 arms/임계값/무학습 probes. 07-05 실행 완료 — probe 수렴으로 A1 생략, A2+A0′ 실행.
- [[research/isaac_inertia_ctrl_mismatch]] — **(07-05) set_inertias는 solver에 전파된다(계측 ×1031) — exp_013은 rate loop ~1300× 저토크 plant에서 학습.** 구 정책 plant-overfit 실증(bad_att 68%), 런타임 물리 오버라이드 금지 + 토크 응답 게이트 (Rule 19).
- [[research/phased_curriculum]] — **(07-05) Phase별 순차 커리큘럼 설계·수식.** 6-dim 고정 action으로 warm-start 무손실, 릴리스/model-mismatch residual 보정, Gauss-Markov 이동타겟·lead, 서브프로세스 오케스트레이션 (Rule 20).
- [[research/ccip_release_decoupling]] — **(07-05) success 100% vs drop_impact_error 4.59 m의 실체 = 지표 의미론 버그.** 릴리스 트리거 부재 → 성공-종단 잔여속도 탄도 캐리(3.0 m/s×1.53 s). CCIP referee 수정 + aim_err_min 진단, 근접 성공 ≠ 릴리스 능력 (Rule 21).
- [[research/ccip_aim_reward_stageA]] — **(07-06) Stage A: 밀집 CCIP 조준 보상 실패 분석.** dense 사이드 보상은 γ-할인 완주 보너스·CCIP 노이즈 증폭(×1.5 s)·성공 조기 종단을 못 이김 — 릴리스는 종단 구조(Phase 2)로. 5-lens 사전 적대 검증·farm 경제 계산 포함 (Rule 22).
- [[research/release_terminal_stageB]] — **(07-06) Stage B: 릴리스-종단 구조로 5.5%→100%.** 종단 교체 단독이 지배 요인(Rule 22a 인과 확정); 자동 발화 referee가 탐험 노이즈를 발견 메커니즘으로 전환; aim shaping은 종단 구조에서 잉여; done-flag alias 함정 (Rule 23).
- [[research/curriculum_phase_convergence]] — **(07-12 baseline §7 + 07-13 이어학습 §8)** warm-start 무손실 실증, reward 우상향 ≠ 임무 능력, P2/P3 릴리스 명중은 500 iter로 미형성 — **+2000 iter 연장도 0.8m 미돌파(P2 정체, P3 회귀)**. 해법=exp_018 종단구조 (Rule 20e/f).

### 실험 (experiments/)
- [[experiments/training_history]] — 전체 WandB 학습 히스토리
- [[experiments/exp_029_l1_sl_generalization]] — **(09-02) 일반화 감사 — 미지 사거리 ✅ 85.3% 회수 / 정책 전이 ⛔ 순손실 / 라벨 1,000개면 96%**
- [[experiments/exp_028_l1_sl_pilot]] — **(09-01) L1-SL 파일럿 — PPO 없이 CEP50 −31.4%(DR1.5) / −32.9%(DR2.5), 오라클 천장의 85%**
- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 거리 보상 + CRUISE retry
- [[experiments/exp_002_reward_shaping_patches]] — 보상 패치 Fresh Training (대기 중)
- [[experiments/exp_003_rtf_dryrun]] — RTF 1/2/4 dry-run 비교. RTF=2 최적 확정.
- [[experiments/exp_004_rl_yolo_debug_vision]] — Vision YOLO TRACKING + EKF East 좌표 버그 수정.
- [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] — Arming-rejection throughput fix. v11 분석 + 수정 3종 + dry-run 검증.
- [[experiments/exp_006_xgzum51v_armdiag_dryrun]] — arm_bail 진단. EKF 재수렴 bimodal 계측 → 10s 컷이 진짜 병목. Fix: arm_bail 10→20s.
- [[experiments/exp_007_iyhfy5ps_v13_eval]] — v13 deterministic eval. 유효 3-ep 100% 성공(reward 124); ep 4–13 EKF divergence 흡수 루프. harness 지표 비정합 발견.
- [[experiments/exp_008_dryrun_alt10_handoff_window]] — 핸드오프 윈도우↑ dry-run. 고도↑ 실패→탐지 게이트 수정(conf+300 px)으로 핸드오프 2.7→5.0 m. 미커밋.
- [[experiments/exp_009_softreset_throughput]] — 리셋 처리량. EKF param A/B(음성) + soft reset 프로토(~3.9×, EKF 안정). full run 검증 중.
- [[experiments/exp_010_byxyaf4d_v14_195k_eval]] — v14 195K eval 65%(13/20, v13 회귀, final-approach stagnation) + soft reset 장기검증(3096 resets, Rule 14 완료) + 비디오 산출물.
- [[experiments/exp_011_wobble_lpf_reward_damping]] — RL wobble = smoothness-control 문제 확정. LPF A/B(jerk RMS −45%) + 보상 댐핑(B+C) → v15 fresh 기동. **Postmortem(07-05): v15 310K에서 X마커 미도달 회귀 의심(미확정) — 댐핑 반경 재타격 또는 crash-resume buffer 초기화, Rule 25/26.**
- [[experiments/exp_012_isaac_migration_phase2]] — Isaac Lab env+PPO 이식. v13/v15 상수 parity 이식, `pytest test_math.py` 29/29 통과, L4 VM 미기동.
- [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] — Isaac 첫 완주 PPO(65.5M steps) + eval 36%. 비전 사멸 버그 수정, 상승-farming attractor·보상 불균형·noise_std 폭주 규명 → Rule 17·18.
- [[experiments/exp_014_A2_visionrange]] — **(07-05) plant 수정(킥·로터·inertia) + 비전 거리감쇠 → eval 100.0%(202/202).** A2 R_alt=0.0000(창발-기각 시그니처), A0′ 대조로 귀속 분리(지배=plant 일관성, 감쇠=꼬리 제거+parity). noise_std 안정 → Rule 18b 재해석.
- [[experiments/exp_015_phased_curriculum]] — **(07-05 구현 + 07-12 §7 + 07-13 §8)** action 4→6 residual, 릴리스+DR, 이동타겟+lead. **§7: baseline 완주 — P1 success 1.00, P2 drop 2.91m, P3 lead best 0.071m. §8: P2/P3 각 +2000 iters — 0.8m 미돌파, P3 회귀** (Rule 20e/f).
- [[experiments/exp_016_ccip_release_reeval]] — **(07-05) 4.59 m 디커플링 규명·수정 + A2 200-ep 재평가.** 릴리스 트리거 부재(지표 의미론 버그) → CCIP referee 수정: 발화 시 0.137 m, release_rate 6%/11.5%, aim_err_min ≈ d_xy_min(cross-track 지배). Rule 21.
- [[experiments/exp_017_stageA_aim_reward]] — **(07-06) Stage A: 밀집 CCIP 조준 보상 3-run(기준선/w=1/w=2).** det release_rate 2.5→5.5→3.5% — 판정 (b) 정체. P1 6-dim 기준선·warm-start 체인(399→1399)·det eval 전표·ckpt 아티팩트 경로. Rule 22.
- [[experiments/exp_018_release_terminal]] — **(07-06) Stage B: 릴리스-종단 4-run(B0 종단만/B1 w↑/B2 knee↑/B3 aim 제거).** det 100/100/98.5/100%, drop err ~0.13 m, 학습 내 단조 상승. 호버-드롭 수렴, Stage C warm-start=B0. Rule 23.
- [[experiments/exp_019_physical_payload]] — **(07-21) 물리 페이로드 attach/detach — kinematic weld 구현·검증 4/4 PASS.** 결함 6종 발견·3종 수정, 측정 착탄 vs 해석적 CCIP |Δ| ≤ 0.021 m, 보상/종단 bit-identical. Rule 24.
- [[experiments/exp_020_o5jn9xzk_payload_training]] — **(07-23) 물리 페이로드 부착 첫 학습 — 학습 비용 0 확증.** B0 warm-start + 보상 bit-match, det 200-ep 100.00%/drop 0.169 m. σ 드리프트 1.41→1.71 관찰. `play.py --wandb` eval-figure 파이프라인 신설.
- [[experiments/exp_022_p0_handoff_dyn_dr]] — **(08-03) P0: 핸드오프 랜덤화 + 동역학/센싱 DR, v20 env 신설.** 유닛 66/66 · 프로브 v19 12/12 + v20 15/15 · v19 warm-start 무손실. 분포전이 4조건 91.0/91.5/77.1/**7.5%** → Rule 27.
- [[experiments/exp_026_release_rate_100hz]] — **(08-27) 릴리즈 판정을 물리 주파수로 + 성공 반경 0.5 m.** T2 CEP50 0.762 → 0.395 m. T1≡T2(타이밍이 argmin 이점의 전부였음), 오라클 갭이 중앙값에서 꼬리로 이동 → 헤드라인 지표 교체 필요
- [[research/sim2real_gap]] — **(08-27) 실기 대비 결손 목록.** 경쟁 논문 ablation: 공력·모터지연·컨트롤러 이상화가 각각 zero-shot 오차를 **2.4~3.7배** 바꾼다. 우리에게 없는 것: 로터 유입류 · 모터 1차 지연 · 추력-전압 맵 · **게인 실측 캘리브레이션**(`tilt_clamp_deg=35`가 임의값). **지금은 구현하지 않고 Limitations에 크기와 함께 명시**
- [[daily/daily_2026-08-30]] — **(08-30) L0 학습 성공과 그 함의.** 보상 지형 매핑 · 속도 교락 · 전수 감사 6건 · 파레토 지배 · 바람 실측 · T3를 이긴 것의 문제
- [[daily/daily_2026-08-29]] — **(08-29) 첫 학습과 실패.** 파일럿 수치·원인 진단·다음 실험 순서 / wandb·스트리밍 인프라 / 메시 5.4배 / 급강하 vs 순항통과 프레이밍 판단
- [[sessions/session_2026-08-27]] — **(08-27) 세션 기록**: 작업 순서, **다음 세션 재개 명령**, 판정 기준, 반복하지 말 것 5가지
- [[research/reward_operating_point]] — **(08-27) 보상이 hover-drop을 17.7점 선호하고 있었다** — 실측 에피소드에 보상을 적용해 확인. `w_time` 0.01 → 1.0(무차별점). ⚠️ 스크립트 베이스라인도 구조적으로 place($k_p t_{fall}=0.66<1$, 릴리즈 $v_{xy}$ 0.76 m/s) — `--pass_speed` 채택 여부 대기
- [[errors/err_20260827_free_exit_termination]] — **(08-27) dry-run이 잡은 결함 2건**: `bad_attitude`가 벌점 없는 무료 탈출구 / 접근 보상이 탐지 뒤에 갇힌 닭-달걀
- [[research/research_architecture]] — ⭐⭐ **(08-27 v4) 주장·실험·ablation·프로세스 전면 확정.** 주 주장 = AeroThrow식 규칙의 투척→배치 퇴화를 잔차로 깬다(원저자 인정 근거). T3는 **상한선이 아니라 정보 ablation**(실측에서 T2에게 짐). 헤드라인 = 성공률@0.5·CEP90·**배달 시간**. 주 조건 = DR_SCALE 1.5. Ablation A1~A9, **A1+A2만으로 논문 성립**. Phase 1~5, 11~16일
- [[research/related_work_survey]] — **(08-27) ⚠️ 선행연구 조사: Scaramuzza(UZH RPG)가 2026-06-25에 거의 같은 문제를 실기·비전·오픈소스까지 포함해 발표.** 정확도/민첩성 경쟁은 진다. 단 **그들의 future work 3항목(항력 모델링·페이로드 공력 랜덤화·이동 중 투하)이 정확히 우리가 이미 구현한 것**이고 그들 논문에 **바람이 없다.** 방어 가능한 기여는 ① 탄착 공간 잔차(주입 지점) ② **회복 가능성의 정량화(오라클 갭 vs DR_SCALE)** — ②는 실기 없이 성립하는 유일한 기여
- [[research/agility_ceiling]] — **(08-27) 급기동은 지금 불가능하고 막는 층이 4개.** 최대 천장은 코드가 아니라 인지 기하: `reveal_radius = 7 m`가 순항 속도를 **5.8 m/s로 못박음**. 민첩해질수록 오차 예산이 잔차가 고칠 수 있는 쪽으로 이동 → 논문에 유리. 진짜 dive-bomb에는 CTBR 필요(08-27 오전 기각 판단 번복)
- [[experiments/exp_025_dr_scale_sweep_gate]] — **(08-27) DR_SCALE 스윕 유효성 게이트 — 무학습 T2/T3만으로 논문 §7.4 핵심 그림의 전제를 확인.** 오라클 갭 −0.003 → 0.325 m 단조 증가, scale 0에서 ≈ 0. **학습 착수 조건 충족.** 부수 실측: 잔여 오차 0.57 m는 전부 제어·릴리즈 판정 타이밍
- [[experiments/exp_021_v19_moving_target]] — **(07-30) 이동 타겟(CV/CT/CA) v19 obs-보존 포팅 + 준상 v19 warm-start 학습 3종 완주.** obs 28-D 불변 lossless 로드 실증, 난이도 cv<ct<ca(ca reward 음수 잔존), det eval 후속. wandb a6saa42b/29jqq1lu/ntumqwoz.

### 에러 (errors/)
- [[errors/err_20260803_payload_landing_latch]] — **(08-03)** 물리 페이로드 착지 래치가 정지한 페이로드를 영원히 놓침(실린더 half-height 0.03 > 판정면 0.0, 래치 0/32). 성공이 타임아웃으로 위장 → v19 기준선 91.0%→100.00% 정정. Rule 28.
- [[errors/err_20260703_vision_env_origin_frame]] — Isaac `_update_vision` world/env-local 프레임 혼용 → 벡터화 학습 비전 완전 사멸. "정확히 0.0000인 보상 성분 = 채널 사멸 신호" 규칙.
- [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] — armdiag dry-run이 YAML 중복 `checkpoint_dir` 키로 v13 30K 체크포인트 파괴. fresh-start 삭제 footgun + 격리 검증 규칙.
- [[errors/err_20260615_cruise-timeout-arming]] — CRUISE 타임아웃 = teleport 후 PX4 arm 거부 (stale EKF). arm 게이팅 + early-bail.
- [[errors/err_20260320_physics_explosion]] — Gazebo ODE 물리 폭발 3중 방어
- [[errors/err_20260319_ode_aabb_crash]] — 드론 스폰 고도 ODE AABB 크래시

### 연구 일지 (daily/)
- [[daily/daily_2026-08-27]] — **T3 오라클 재정의**(즉시 엔트레인먼트 → 플랜트 동일 적분) · 페이로드 항력 프레임 버그 · **DR_SCALE 스윕 사전 검증에서 결정론적 오차 바닥 발견**(0.44 → 0.015 m) · 환경 재구축 계획 승인 · Rule 31 신설
- [[daily/daily_2026-08-23]] — **CCIP $v_z$ 누락 발견·수정**(모델오차의 ~70%) · 아키텍처 문서 v3 전면 개정(전제 1개 거짓 + 버그 4개 + 성립불가 DR 3개) · 기존 학습 산출물 폐기 결정 · Rule 30 신설
- [[daily/daily_2026-08-03]] — **P0 전 항목 완료**(v20 랜덤화 env + 공유 평가 하네스 + 무학습 베이스라인 T0~T3) · Table 1 1차 실측 · 착지 래치 버그 수정(v19 기준선 91→100% 정정) · **(a)안 학습 음성 결과**(페널티 회피 수렴). Rule 27/28/29 신설
- [[daily/daily_2026-08-01]] — 브랜치 정리: 10개 브랜치 계보 조사, push 용량초과 원인 규명(실수로 커밋된 SAC/영상 바이너리), `Isaac-JS` 고유 노트 `isaac_jk`로 포팅 후 삭제, `main`을 `isaac_jk`로 승격(구 main은 태그로 보존)
- [[daily/daily_2026-07-30]] — exp_021: 이동 타겟(CV/CT/CA) v19 포팅 + 준상 v19 warm-start(사본) 학습 3종 완주, wandb 3 runs
- [[daily/daily_2026-07-23]] — exp_020 물리 페이로드 부착 학습 완주(100%, 학습 비용 0) + wandb eval-figure 파이프라인 + wandb 키 공백 함정 해결
- [[daily/daily_2026-07-13]] — exp_015 이어학습(2차) P2/P3 ext 완주: iter 예산 확대로 0.8m 미돌파, P3 회귀, VM 종료
- [[daily/daily_2026-07-07]] — **(Gazebo/SAC 트랙, isaac_jk 분기 전 기록)** 보상 함수 전체 리뷰(10단계 변천사) + 차기 설계 제안(v16/Stage C) — 기록 전용, 코드 변경 없음
- [[daily/daily_2026-07-06]] — exp_017 Stage A(보상 단독 정체, Rule 22) + exp_018 Stage B(릴리스-종단 → 100%, Rule 23): 같은 날 문제 확정과 해결. 5-lens/4-lens 적대 검증, alias 버그 사전 수정
- [[daily/daily_2026-07-05]] — plant 수정 실행 + inertia 대반전(Rule 19) + exp_014 A2/A0′: eval 36%→100%, R_alt 0, noise_std 안정. 캘리브레이션은 이미지 버그로 차단
- [[daily/daily_2026-07-05_gazebo_v15_regression]] — **(Gazebo/SAC 트랙, isaac_jk 분기 전 기록)** v15(310K) X마커 미도달 회귀 진단 + `drone-bombard-harmonic` 컨테이너 GPU 드라이버 불일치(535 vs 580) 발견 — 원인 미확정, Rule 25/26
- [[daily/daily_2026-07-04]] — exp_013 forensics: 킥=프로세스당 1회 확정(m_eff 계측), max_alt=학습된 attractor 재귀속, entropy σ-불변 실측, eval 8ep 해명, exp_014 ablation 설계
- [[daily/daily_2026-07-03]] — Isaac Lab migration Phase 2: env+PPO 코드 이식, 워크트리 분리, test_math.py 29/29 통과, Rule 16 신규
- [[daily/daily_2026-06-23]] — v14 plateau stop @196.5K → 195K eval 65%(v13 회귀, final-approach stagnation) + soft reset 장기검증(Rule 14 완료) + 비디오 캡처
- [[daily/daily_2026-06-22]] — 핸드오프 윈도우 확장: 고도↑(10 m) 실패 → 탐지 게이트(conf 0.5 + 200→300 px)로 핸드오프 2.7→5.0 m
- [[daily/daily_2026-06-20]] — v13 정책 평가: 유효 ep 100% 성공(정책 양호) + eval EKF divergence 흡수 루프 + harness 지표 비정합
- [[daily/daily_2026-06-16]] — v12 정체 진단(종단 overshoot 트랩) + v13 종단 보상 재설계 prep
- [[daily/daily_2026-06-14]] — 06-12 이후 종합: 보상 함수 재설계(v9) + YOLO hold(v9b) + throughput 최적화(v9c·v9d)
- [[daily/daily_2026-06-12]] — EKF East 반전 수정 + proximity trigger 정상화 + rl_yolo fresh start
- [[daily/daily_2026-05-30]] — Isaac Sim migration Phase 1 인프라 구축 (Dockerfile·startup·deploy 재작성)
- [[daily/daily_2026-04-26]] — Vision obs 리팩토링 계획 + 카메라 파이프라인 복원
- [[daily/daily_2026-04-23]] — Spot VM 이전 완료 (startup.sh + watchdog CF + create_spot_vm.sh) + IP 변경 대응
- [[daily/daily_2026-04-17]] — Phase 1 코드 전체 구현 (변경 1-10, obs 15→17, CCIP auto-drop)
- [[daily/daily_2026-04-16]] — WandB 연결 + RTF dry-run 실험 + 인프라 고장 해결
- [[daily/daily_2026-04-14]] — Guacamole HTTPS + Obsidian 설치 + wikilink 정비

### 세션 (sessions/)
- [[sessions/session_2026-04-16]] — RTF dry-run, docker commit, airframe 수정
- [[sessions/session_2026-04-14]] — Obsidian 시스템 초기화 + 파일 간소화

### 환경 설정 (Environment/)
- [[Environment/README]] — VM 완전 복구 가이드
- [[Environment/docker-compose.yml]] — guacd + guacamole + postgres + nginx
- [[Environment/nginx.conf]] — WebSocket 리버스 프록시 설정
- [[Environment/vncserver.service]] — TigerVNC systemd 유닛
