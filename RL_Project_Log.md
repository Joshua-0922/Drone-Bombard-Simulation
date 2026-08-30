# RL Training Pipeline — Project Log

> **Branch:** `main` | **VM 프리엠션 후 컨텍스트 복구용 로그**

---

# 1. Current State

**업데이트:** 2026-08-30

> **✅⭐⭐ (2026-08-30) L0 학습 성공 — baseline front 를 전 축에서 지배.**
> n=200, DR_SCALE 1.5, r=1.0, 결정론 정책, T2 와 같은 하네스/시드:
>
> | 팔 | 릴리즈 v | 배달시간 | 성공률 | CEP50 | CEP90 |
> |---|---|---|---|---|---|
> | T0 place | 0.05 | 9.00 s | 61.0% | 0.511 | 1.217 |
> | T2 @1.30 | 1.30 | 8.01 | 65.0% | 0.479 | 1.015 |
> | T3 참값 | 3.12 | 6.83 | 55.5% | 0.581 | 1.475 |
> | T2 @2.61 | 2.61 | 6.32 | 56.0% | 0.688 | 1.406 |
> | **L0 (제안)** | 1.39 | **5.86** | **92.0%** | **0.319** | **0.633** |
>
> **6개 팔 전부를 배달시간·성공률·CEP50 세 축 동시에 지배.**
>
> **여기까지 온 수정 (보상 2 + 구조 6):**
> ① `k_landing` 2.0→1.0 — 지수형 종단이 학습 정책이 사는 영역(1~2.5 m)에서
>    기울기가 죽어 "4.0 s 에 1.50 m 대충 투하(-77)"가 "6.7 s 에 0.84 m 정확히
>    투하(-98)"보다 나았다
> ② `success_radius` 0.5→1.0 — 보너스 100점이 도달 불가 영역에 있어 기여 0.
>    정확도 마진 +0.5 → +100.5
> ③ `num_steps_per_env` 32→96 — 에피소드 97 스텝인데 롤아웃 32라 종단 보상이
>    창 밖. 배회의 원인은 보상이 아니라 크레딧 전달이었다(72→97 악화 → 112→59 개선)
> ④ 보상 항 6개 무효화 해소 — `w_time` 230배 인상 시 미조정. `w_ccip` 0.5(전체의
>    0.2%), `w_loiter` -0.1(배회 방지 항인데!) 등
> ⑤ `reward_scale` 0.1 — 리턴 [-500,+350] 미정규화. value loss 184.7 → 89.4
> ⑥ 관측에 "남은 시간" 채널(25→26) + 계측 3종 + `play.py --no_residual`
>
> **⚠️ 성공의 방식이 논문에 문제를 던진다:**
> - 우위의 대부분이 **릴리즈율**(73→94%)에서 온다. 투하했을 때 명중률은
>   89.0% → 97.9% 로 +9 pp — "조준"보다 "아예 던지기"가 크다
> - 정책이 **"빠른 접근(평균 4.2 m/s) + 감속 + 느린 투하(1.39 m/s)"** 를 찾았다.
>   배달시간 축에서는 최우수지만 릴리즈 속도로 말하는 주장에는 불리.
>   `w_time` 은 이걸 막지 못한다 → **명시적 릴리즈 속도 항 필요**
> - **L0 가 T3(참 바람·탄도계수·릴리즈 지연 전부 주입)를 크게 이긴다**
>   (92.0 vs 55.5%). 모수 불확실성이 주된 난이도가 아니었다는 뜻이고
>   **L1(잔차)의 여지를 직접 위협한다**
>
> **속도 교락**: T2 를 속도별로 재니 1.30 m/s 에서 65.0%, 3.07 에서 47.0%.
> 단일 점 비교는 무효. 곡선이 논문 그림 2의 baseline front.
>
> **바람 실측**: 축당 N(0, 2.25) m/s, 중앙값 |wind| 2.65 m/s(보퍼트 2~3등급).
> 착탄 이동 m/s 당 0.13 m — 중앙값에서 0.35 m, 90 퍼센타일 0.78 m.
> 에피소드의 10~15%가 바람만으로 실패권.
>
> **다음**: ① 바람 vs L0 잔여오차 상관(L1 마진 예측) ② A9 unseen(15/25 m)
> ③ 릴리즈 속도 항 설계 ④ L1 을 Ma&Hutter 표준 레시피로
> → [[daily/daily_2026-08-30]]


> **⛔⭐ (2026-08-29) 첫 학습 파일럿 — 실패. Rule 29 조기 중단.**
> DR_SCALE 1.5, 2048 envs, seed 1, cold start, 1200 iteration.
>
> | | 성공률 | 릴리즈율 | 릴리즈 속도 |
> |---|---|---|---|
> | **T2 (학습 0, 스크립트)** | **10.5%** | 72.5% | 3.07 m/s |
> | T3 (참값 주입, 학습 0) | 29.5% | 70.5% | 3.12 m/s |
> | **L1 (잔차, 1200 iter)** | **0.9%** | 80.2% | 2.10 m/s |
> | **L2 (pure RL, 1060 iter)** | **0.0%** | 60.1% | 3.28 m/s |
>
> 성공률이 0~10%를 무작위로 튈 뿐 **오르는 추세가 없다.** 시드를 늘리지 말고
> 설계로 복귀할 것.
>
> **⚠️ 릴리즈율은 학습 건강 지표가 아니다** — 80%가 나와도 성공률은 0.9%다.
> "던졌다"와 "맞췄다"는 다른 사건이고, 성공 반경 0.5 m에 조준 오차가 1.5~5.1 m다.
>
> **원인 추정**: `residual.scale = 2.0`이라 초기 무작위 잔차가 축당 RMS 1.1 m
> (반경 1.6 m)를 조준점에 얹는다. T2의 착탄 오차가 0.84 m이므로 **L1은 초기에
> 조준 오차가 3배인 지점에서 출발**해서, 과제를 배우기 전에 "잔차를 0으로 만드는 법"
> 부터 배워야 한다. 표준 residual RL은 **잔차 헤드를 0 근처로 초기화**해 이를 피하는데
> 우리는 하지 않았다. L1의 릴리즈 속도가 2.10 m/s로 T2(3.07)보다 느린 것도
> **place 퇴화 조짐**이다.
>
> **다음 순서**: ① **L0(잔차 없음) 통제군** — L0 > L1이면 잔차가 해를 끼치는 것 확정
> ② **‖δ‖ 로깅 추가** — 지금 추정에 머물러 있다 ③ 잔차 헤드 0 초기화
> ④ `residual.scale` 2.0 → 0.5~1.0 검토
>
> **부수 수정**: 드론 메시가 물리와 5.4배 어긋나 있었다(물리 X500 2.07 kg·대각 500 mm
> vs 렌더 Crazyflie cf2x 92 mm) → `spawn.scale=5.4`, 질량·관성 불변 검증.
> wandb 키 없이 `--logger wandb`가 조용히 tensorboard로 떨어지던 것 → 사전 검사로 차단.
> 헤드리스 VM 투하 영상 = 오프스크린 렌더(`--video`), TigerVNC는 Vulkan 표면 불가.
>
> **프레이밍 판단**: 순항 통과 투하 유지(급강하 도입 안 함). 단 속도 3 → 10 m/s
> 상향은 **학습을 고친 뒤에** 검토. → [[daily/daily_2026-08-29]]


> **✅⭐⭐ (2026-08-27 종료 시점) Phase 1 완료 — 논문의 무학습 산출물 전량 확보.**
> Table 1, 그림 2(속도-정확도), 그림 4(인과 귀속)가 모두 나왔다. **남은 것은 학습 행 하나.**
>
> **Table 1** (n=200, DR_SCALE 1.5, r=0.5, 판정 100 Hz, GT):
> T0 place 35.0% / 0.511 / 9.00 s / v_rel 0.05 · T1 throw 11.0% / 0.830 / 6.77 s / 3.09
> · T2 throw 10.5% / 0.840 / 6.70 s / 3.07 · **T3 throw 29.5% / 0.581 / 6.83 s / 3.12**
>
> **DR_SCALE 스윕** (정속 패스, n=200): 귀속 갭(T3−T2) **−1.5 → +1.0 → +8.0 → +19.0 pp**
> 단조 증가, scale 0에서 ≈0(통제군). T2 34.0→10.5% 단조 하락, T3 32.5→29.5% 평탄.
>
> ⚠️ **헤드라인 지표 최종 확정: 성공률@0.5 m + CEP50 + 배달 시간.**
> CEP90은 단조가 아니다(DR 1.5에서 0.097 < DR 1.0의 0.165) — 모수 참값은 분포 **중심**을
> 좁히지만 **꼬리**는 강풍이 기체를 미는 제어 오차라 조준점 보정으로 회복되지 않는다.
> 오전에 "CEP50은 죽었다"고 판단했던 것은 **베이스라인이 전부 place라 중심이 이미
> 같았기 때문**이지 지표 결함이 아니었다.
>
> **Phase 1에서 잡은 결함 6건 + 베이스라인 결함 1건** (전부 집계 지표는 침묵했고
> 성분별·조건별 계측을 넣은 뒤에야 보였다):
> ① 보상이 hover-drop을 17.7점 선호 ② `bad_attitude`가 벌점 없는 무료 탈출구
> ③ 접근 보상이 탐지 뒤에 갇힌 닭-달걀 ④ 스폰 고도 ∉ 릴리즈 밴드
> ⑤ 고도 규제 항 부재 ⑥ crash 바닥 = 릴리즈 바닥
> ⑦ **접근 P-컨트롤러가 모든 팔을 place로 강제** (v = kp·r 이면 kp와 무관하게
> 표적 위 정지에서만 투하 → 정속 패스로 교체, 릴리즈 속도 0.76 → 3.07 m/s)
>
> **확정 조건:** DR_SCALE 1.5 · 판정 100 Hz · GT · r=0.5 · 방위각 ±30° ·
> `w_time` 2.3/step(무차별점) · 에피소드 20 s · `release.alt_max` **8 m**(⚠️ 12로 올렸다고
> 보고했으나 편집이 실제로 적용되지 않았다 — 모든 측정은 8 기준, 값 자체는 유효) ·
> `w_alt_band` 0.5(부트스트랩을 실제로 뚫은 항) ·
> `termination.min_altitude` 1.5 m · T0=place / T1·T2·T3=throw
>
> **다음: Phase 2** — L1 파일럿 1개로 장기 거동 확인 후 L1/L0 × 3 seed @ DR 1.5.
> ⛔ Rule 29 조기 중단 게이트: L1이 T2(10.5%)를 유의하게 못 넘으면 시드를 늘리지 말고
> 설계로 복귀. → [[daily/daily_2026-08-27]] §11


> **✅⭐ (2026-08-27) 릴리즈 판정을 물리 주파수(100 Hz)로 + 성공 반경 1.0 → 0.5 m.**
> exp_025가 DR_SCALE 0에서도 CEP50 0.57 m를 측정했는데, 표적 참값 + 탄도 모델 오차
> 0.010 m이므로 **전부 제어 + 릴리즈 타이밍**이었다. 10 Hz 판정 = 4 m/s에서 스텝당 0.4 m
> 이동 = "언제 던질까"의 선택지가 0.4 m 격자 위에만 존재.
> **처방(pickle-and-hold)**: 정책은 10 Hz로 *커밋*하고(`action[4]`를 구간 유지 신호로 재해석),
> 발사 *시점*은 100 Hz 솔버가 고른다 — 예측 탄착점이 표적을 통과하는(조준 오차가 감소를
> 멈추는) 서브스텝에 발사. 실제 CCIP 폭격 시스템의 동작.
> **실측: T2 CEP50 0.762 → 0.395 m** (n=64, DR_SCALE 1.0, r=0.5).
>
> **부수 효과 둘 다 논문에 영향:**
> 1. **T1(단순 임계) ≡ T2(AeroThrow argmin)** — 100 Hz에서 성공률·CEP50·CEP90 전부 동일.
>    argmin은 "통과 순간"의 10 Hz 샘플링이므로 당연한 귀결. **AeroThrow 이점의 전부가
>    타이밍 해상도였다.** 10 Hz에서는 1.129 vs 0.762로 분리되므로 `wants_drop`이 무시되는
>    버그가 아님도 같은 표가 증명. 물러설 길 = `--release_10hz` (§7.4 ablation 축).
> 2. **오라클 갭이 중앙값 → 꼬리로 이동** — CEP50 갭 0.136 → 0.023 m인데
>    **CEP90 갭 0.206 m, 성공률 갭 18.8 pp.** ⚠️ **헤드라인 지표를 CEP50에서
>    CEP90/성공률(0.5 m)로 교체해야 한다.**
>
> **새 Table 1** (n=128, GT, DR_SCALE 1.0, r=0.5, 판정 100 Hz):
> T0 hover 65.6% / 0.323 / 9.32 s · T1 53.1% / 0.371 / 7.14 s ·
> T2 53.1% / 0.378 / 7.32 s · **T3 oracle 71.9% / 0.355 / 6.93 s**.
> T3가 T0를 정확도·시간 양쪽에서 이겼다 — 학습 잔차의 목표는 **53.1% → 71.9%의 18.8 pp**.
> → [[experiments/exp_026_release_rate_100hz]]

> **✅⭐ (2026-08-27) DR_SCALE 스윕 100 Hz 재측정 — 주 실험 조건 = DR_SCALE 1.5 확정.**
> n=128, GT, r=0.5, 판정 100 Hz.
> **오라클 T3 CEP50이 전 구간 0.350/0.356/0.355/0.356 — 스프레드 0.006 m로 사실상 완전 평탄.**
> T2는 0.284 → 0.378 → 0.594 (CEP90 0.496 → 0.743 → 1.162), 성공률 77.3 → 53.1 → 32.0%.
> 갭: CEP90 0.206 m(DR 1.0) → **0.536 m**(DR 1.5), 성공률 18.8 pp → **23.4 pp**.
> ⚠️ **CEP50은 헤드라인 지표로 죽었다** — DR 1.0에서 갭 0.023 m, DR 0.5에서는 음수.
> 모델 오차는 타이밍 제거 후 **꼬리에만 산다.** → CEP90 / 성공률로 교체.
> ⚠️ DR ≤ 0.5의 음수 갭(−0.02~−0.07 m, −4.7~−5.5 pp)은 **n=128 잡음**(CI ±8 pp).
> 갭 ≈ 0이라는 통제군 성질은 유지. 최종 표는 n ≥ 500.
> T3도 DR 1.5에서 성공률 71.9 → 55.5%로 떨어지는데, CEP50은 고정이고 CEP90만 오른다 —
> **오라클은 조준점만 고치지 비행을 못 고친다.** 강풍이 기체를 밀어 생기는 추종 오차는
> 잔차로도 회복 불가능한 바닥의 일부다.
>
> ⚠️ **"회복률(recovery ratio)" · "회복 가능분" · "상한선" 표현은 철회.**
> T3는 T2와 같은 릴리즈 규칙에 공칭 파라미터만 참값으로 치환한 **정보 ablation**이지
> 상한이 아니다 — DR 0/0.5에서 **T2가 T3를 이겼다**(74.2 vs 69.5%, 77.3 vs 71.9%).
> 상한선은 질 수 없다. 스윕의 역할은 **"인과 귀속"**이다: 개선폭이 모델 오차와 함께
> 커지고 모델 오차 0이면 개선도 0 → 개선의 원인이 모델 오차 보정임을 확정.
> 학습 팔 목표선(DR 1.5): CEP90 1.162 → **0.894 m**, 성공률 32.0 → **43.7%**
> (= T2와 T3의 중간 지점).
> → [[experiments/exp_026_release_rate_100hz]] §3.6–3.7

> **📐 (2026-08-27) 급기동(dive-bomb)은 현재 불가능 — 막는 층 4개 + 인지 기하 천장.**
> ① 작동 권한(`vx_scale` 4 m/s, `tilt_clamp_deg` 35° = 0.70 g, `rate_limit` 0.2 → 전폭 반전 1.0 s;
> 기체는 T/W 2.0으로 1.73 g까지 가능한데 컨트롤러가 40%만 허용),
> ② 종료 조건(`limit_ang_vel` 2.0 rad/s가 단일 최대 장애물, `min_altitude` 3.0 m),
> ③ 릴리즈 엔벨로프(`max_speed` 5, `max_tilt` 0.35 rad = 20°),
> ④ 보상(`w_ang_vel`/`w_tilt`/`w_action_smooth`가 민첩성에 세금).
> **⭐ 최대 천장은 코드가 아니라 기하:** $d \ge v\sqrt{2L/a_{\max}}$에서
> `reveal_radius = 7 m` + 틸트 35°는 순항 속도를 **5.8 m/s로 못박는다.**
> `handoff.speed_range = (2, 6)`이 이미 그 한계에 붙어 있다 — **속도를 올리려면 발견 거리를
> 같이 올려야 한다.**
> **논문에는 유리하다:** 릴리즈 지연 오차 $\sigma_\tau v$는 4 m/s에서 0.20 m, 12 m/s에서 0.60 m로
> 커지는데 **이것은 A그룹 성분이라 잔차가 고칠 수 있다.** 민첩해질수록 오차 예산이
> 잔차가 손댈 수 있는 쪽으로 이동한다.
> 운반 페이로드가 **월드 프레임 z 오프셋**으로 매달려 있어 틸트를 넓혀도 `_nominal_impact`
> 기하가 깨지지 않는다(플랜트·예측기 일관).
> **진짜 dive-bomb에는 CTBR 필요** — 속도 루프 대역폭 0.29 Hz로는 1초 인출이 불가능.
> 08-27 오전의 CTBR 기각은 "축은 정확도"라는 전제 위였으므로, 축이 민첩성으로 바뀌면 번복된다.
> → [[research/agility_ceiling]]


> **⛔🔧 (2026-08-27) T3 "오라클"이 상한선이 아니라 하한선 아래에 있었음 — 수정 완료 (커밋 `59755e7`).**
> `baseline_drop.py`의 오라클 보정이 `ballistic_impact`의 해석식 바람항을 썼는데, 이 식은
> **"페이로드가 분리 즉시 바람 속도에 100% 실린다"**(즉시 완전 엔트레인먼트)를 가정한다.
> 실제 플랜트는 드론 속도로 출발해 항력으로 서서히 끌려가며 시정수 tau = m/(k|v_air|) ≈ 2.5 s인데
> 낙하는 ≈1.0 s라 **바람의 1/4 정도만 엔트레인**된다. 결과: **3.7~4.7배 과보정**,
> 게이트와 조종 목표점 양쪽에 그 값을 써서 상풍 1.5 m로 날아가 투하 → 1.24 m 상풍 착탄.
> **보정을 아예 안 하는 것보다 나빴다.** T3 47.5% < T0 hover 91.5%의 정체.
> **수정**: `math_utils.integrate_payload_impact` 신설 — 플랜트와 같은 ODE·같은 k/m·같은 dt로
> 전방 적분(구성상 정확, 닫힌 형태를 일부러 쓰지 않음). 재측정 n=64에서
> **T1 3.1% < T2 37.5% < T3 59.4% < T0 68.8%** — 상한 회복.
> 동시에 **페이로드 항력 프레임 버그** 수정(월드 프레임 힘을 `is_global` 기본값=링크 프레임으로
> 전달 → 드론 자세만큼 회전). `payload_bc_over_m` 프로퍼티로 k/m 정의를 한 곳에 묶음.
> **검증: 단위테스트 74 passed** (회귀 5종 신설).
> → `notes/research/t3_oracle_entrainment.md` / `notes/errors/err_20260827_payload_drag_body_frame.md`
> / **Rule 31** 신설
>
> **⭐ (2026-08-27) 부수 발견 — 지배 오차는 바람이 아니라 페이로드 자기 속도에 대한 항력.**
> 무풍에서도 실제 착탄이 예측보다 **1.15 m 짧다.** 6 m/s 투하 시 자기 속도 항력만 −1.00 m
> (바람은 +0.52 m). 드론이 자기 속도를 아는 이상 **원리상 계산 가능한 항**이므로,
> Rule 30의 vz 누락과 **같은 종류의 결손이 하나 더** 있었던 것.
>
> **⛔ (2026-08-27) DR_SCALE 스윕이 현 상태로는 성립하지 않음 — 학습 전 사전 계측으로 발견.**
> **랜덤화를 완전히 꺼도(DR_SCALE=0) 오차 p50이 0.44 m.** 결정론적 바닥이 랜덤화 성분을
> 덮고 있었다. 원인 셋: 릴리즈 지연 유령 캐리 / 낙하고도 규약(mount −0.14, latch +0.10) /
> 자기 속도 항력. **셋을 고치면 0.44 → 0.015 m**로 붕괴하고 그제서야 스윕이 단조·선형
> (p50: 0.013 → 0.085 → 0.171 → 0.268 m for scale 0/0.5/1.0/1.5).
> **`residual_scale = 2.0`은 적정** — 문서 스윕 범위에서 포화율 0.1% 이하 (§5.4 사이징 답).
> ⚠️ **전략 경고**: DR_SCALE 1.5에서도 p90 0.72 m < success_radius 1.0 m. 탄도 오차만으로는
> 이진 성공률이 거의 안 움직임. 인지(픽셀 양자화 ~0.28 m)와 제어(~0.2–0.3 m)가 더 큼
> → **주 실험은 GT로**, 연속 지표 병기 필수, 논문 축을 "정확도"가 아니라
> **"속도를 낸 상태에서의 정확도"**로 잡을 것.
>
> **🏗️ (2026-08-27) 환경 코드 재구축 완료.** `v11_env.py`의 버전 상속 사슬(설정 클래스 10개,
> 환경 클래스 7개, `V11Cfg -> ... -> V20`)을 **단일 `task_env.py`**로 대체했다.
> `Isaac-DroneBombard-Task-v0` 하나만 등록되고 v11~v20 태스크 등록은 제거(파일 자체는
> exp_022/024 재현용으로 보존, 새 코드에서 import 안 함).
> - **모든 DR을 `drone_bombard_env.py`로 집약**하고 세 그룹으로 분리:
>   `model_err`(A: 바람·탄도계수·릴리즈 지연 — **`scale`이 곱해지는 유일한 그룹**),
>   `dyn_dr`(C: 질량신념·게인·센서/액추에이터 노이즈 — 스윕 내내 고정),
>   `handoff`(B: 시나리오). 이전에는 `v14_dr`/`dyn_dr`/`handoff` 세 스위치가
>   A와 C를 섞어 덮고 있어 스윕 자체가 표현 불가였다.
> - **버전 접두어 전면 제거**: `v14_wind_std` → `model_err.wind_std`,
>   `v19_w_loiter` → `task_reward.w_loiter` 등.
> - **해석식 채점 경로 삭제** — `ballistic_impact`는 이제 예측기 전용, 채점은 실측 착탄점만.
> - **릴리즈 지연을 플랜트에 실제 구현** (`request_release` + FIFO 카운트다운).
>   예측기는 공칭 0.22 s만 모델링하므로 편차가 진짜 관측 불가 모델 오차가 된다.
> - **nominal 예측 단일 진입점 `_nominal_impact()`** — 지연 전파 + 페이로드 낙하 기하
>   + 자기 속도 항력항($C_d$=0.07). 결정론적 오차 바닥 **0.44 → 0.010 m**.
> - 검증: **단위테스트 83 passed**(신규 9) + dry-run 2 iter 완주.
>
> **⚠️ (2026-08-27) 계획 전제 하나 기각 — A그룹 축 무게 실측.**
> 릴리즈 엔벨로프 전체 분해(DR_SCALE 1.0): **바람 p50 0.170 m(~90%)**,
> **릴리즈 지연 0.058 m(~31%)**, **페이로드 탄도계수 0.012 m(~6%)**.
> 탄도계수는 ±100%로 넓혀도 p50 0.044 m — 이 축이 조절하는 전체 효과(자기 속도 항력)가
> 엔벨로프 안에서 최대 0.33 m이기 때문. **논문에서 "잔차가 회복하는 관측 불가 외란"을
> 탄도계수로 지목하면 안 된다.** 그 역할은 릴리즈 지연(원리상 관측 불가 + 속도 비례)이며,
> 바람은 기체를 밀기 때문에 부분적으로 추론 가능하다.
> 부수 정정: 8/27 오전에 적은 "자기 속도 항력 −1.00 m, 지배 성분"은 수평 6 m/s로
> **릴리즈 엔벨로프 밖**(`release_max_speed=5.0`) 계산이었다. 엔벨로프 안 최대 0.33 m.
>
> **📐 (2026-08-27) 환경 코드 재구축 계획 승인.** 학습 산출물을 버리기로 한 이상
> 그 결과를 만든 코드 구조도 함께 버린다. `v11_env.py`의 버전 상속 사슬(설정 클래스 10개,
> 환경 클래스 7개)을 **단일 과제 환경으로 대체**, **모든 DR을 `drone_bombard_env.py`로 집약**,
> 버전 접두어(`v14_wind_std` 등) 전면 → 의미 기반 이름. 해석식 채점 경로 완전 삭제
> (예측기 역할만 남김). 아키텍처 문서 정정 5건 반영 완료.
> → 계획: `~/.claude/plans/reactive-mapping-dijkstra.md`


> **⛔🔧 (2026-08-23) CCIP 공식이 수직속도를 누락하고 있었음 — 수정 완료.**
> `math_utils.ballistic_impact`가 낙하시간을 `t = sqrt(2H/g)`로 계산했다. 정식은
> `t = (vz + sqrt(vz^2 + 2gH))/g` (vz = ENU UP-positive). 이 특수화는 **Gazebo 시절
> 릴리즈가 호버 근방에서만 일어났을 때 참**이었고 docstring에 전제가 명시돼 있었으나,
> **v16(exp_019)이 물리 페이로드를 도입**하면서 페이로드가 드론의 실제 선속도를
> 상속받기 시작했고 **v19가 `release_max_vz=3.0`을 허용**하며 전제가 깨졌다.
> **크기: 릴리즈 엔벨로프에서 모델 오차의 약 70%** (vz 0.547 m vs 바람 0.197 m vs
> 항력 0.120 m, p50). vz=-3 m/s·H=8 m·수평 6 m/s에서 **1.62 m overshoot**
> (하강 중이면 낙하가 짧아지므로 구 공식은 착탄점을 멀리 예측).
> **논문 영향**: 주 주장 "CCIP는 정확한 닫힌 형태 해인데 항력·바람이 모델 오차를
> 만든다"가 거짓이었다 — 잔차가 배우던 것의 70%가 외란이 아니라 공식 결손.
> **수정**: `vel_z`를 `ballistic_impact`/`predict_impact_nominal`/`time_to_fall`의
> **필수 인자로 승격**(기본값 미제공 → 조용한 레거시 경로 차단), 호출부 11곳 갱신
> (base env 4 + v-track 5 + `baseline_drop.py` 3 + 진단 1, 스냅샷에 `final_vel_z` 추가).
> **검증**: `tests/` **69 passed**(회귀 3종 신설: 폐쇄형 일치 / vz=0 환원 / 1.62 m 크기 고정),
> smoke 3종 완주(`--v19` 물리 페이로드+잔차+DR, `--phase 1` 스크립트 referee,
> `--phase 2` release+residual+lead). **exp_019 후속 #3과 `ccip_release_decoupling` §4에
> 이미 해야 할 일로 기록돼 있었으나 이행되지 않았다** → **Rule 30** 신설.
> → `notes/research/ccip_vz_omission.md` / `notes/errors/err_20260823_ccip_vz_omission.md`
>
> **📐 (2026-08-23) 아키텍처 문서 v2 → v3 전면 개정** (`notes/research/research_architecture.md`).
> 선행연구 독해만으로 작성된 v2를 코드와 1:1 대조 → **전제 1개 거짓 + 코드 버그 4개 +
> 물리적으로 성립 불가한 DR 축 3개 + 이미 측정된 결과와 충돌 4건**.
> **미수정 잔여 (우선순위 순)**:
> ① **T3 "wind-oracle"이 오라클이 아님** — 해석식 바람항이 "즉시 완전 실림" 가정이라
>    보정량 p50 2.00 m vs 실제 드리프트 0.29~0.76 m, **3~7배 과보정**, 잔여오차 1.88 m로
>    무보정보다 나쁨. T3 47.5% < T0 hover 91.5%의 원인. **B4(Oracle 상한)·Oracle gap·
>    Abstract 문장이 전부 이 위에 서 있음.**
> ② 페이로드 항력이 월드 프레임 계산인데 `is_global=False`로 전달 (기체는 변환하는데
>    페이로드만 누락) → `is_global=True` 한 단어
> ③ `_drag_coef`가 관측에 들어가지만 v19/v20 물리에 영향 0 (팬텀 채널)
> ④ `release_delay`가 예측 상수일 뿐 실제 지연 아님 → 5 m/s에서 0.5 m 유령 캐리
> **성립 불가 DR**: 페이로드 질량과 Cd는 **같은 축**(자유낙하는 k/m 비율로만 결정),
> CoM 오프셋은 kinematic weld라 **효과 0**(Rule 24b — 부착 중 하중 전달 없고 드론 질량은
> `loaded_mass`로 authored되어 릴리즈해도 안 줄어듦).
> **좋은 소식**: 비대칭 actor-critic은 `rsl-rl-lib 3.1.2`가 이미 지원 → **두 줄**.
> 단 무증상 실패 함정 3종(특히 `_get_states()`는 Isaac Lab 2.3.2에서 **호출되지 않는 죽은 훅**).
>
> **🗑️ (2026-08-23) 기존 학습 산출물 전량 폐기 결정 (사용자 판단).**
> v11~v20 체크포인트와 exp_014~024 성능 수치를 논문 근거로 쓰지 않음 —
> v11~v19 DR 부재로 일반화 주장 검증 불가 + 가설 없는 warm-start 누적으로 귀속 불가.
> **유지**: `rl_rules.md` Rule 16~30 · `eval_harness.py` · `baseline_drop.py` T0~T3(무학습) ·
> kinematic weld 페이로드 · spawn-time 질량 authoring · 등가변환 DR 패턴 ·
> `math_utils.py` 순수 함수 + 테스트.
> ⚠️ **원인 정정**: "DR을 안 해서 robustness가 없다"는 절반만 맞다. **v20은 DR을 켰고
> 실패했다**(exp_024). 원인은 DR 부족이 아니라 ① world-frame 관측이라 방위각이
> 강건성이 아니라 **새 과제**였던 것(Rule 27) ② 틀린 prior warm-start(Rule 29).
> **①은 fresh start만으로 풀리지 않는다** — 관측 프레임 변경이 필수.

---

## 이전 상태

**업데이트:** 2026-07-30

> **🔀 병행 트랙 (2026-07-30): Isaac Lab exp_021 — 이동 타겟(CV/CT/CA) v19 포팅 + 준상 v19 warm-start 학습 3종 완주** —
> `isaac_jk` 워크트리, `isaac-verify`/L4. 07-28 브리핑의 "이동타겟/KF v19 포팅 여부" 실행:
> `V11Env._step_moving_target()`(base `_advance_phase_dynamics`와 동일 지점) + V11/V16/V19
> `_get_dones` wire — **obs 28-D 불변 = 준상 v19 ckpt lossless warm-start**. train.py 이동타겟
> 플래그 전 env 공통화, `--target_kf`+v-track 명시적 에러(KF는 base env 전용 유지). warm-start는
> `checkpoints/v19/precise/model_best.pt`의 **사본**(`/opt/drone-bombard/checkpoints/v19_junsang_copy/`,
> 컨테이너 `/workspace/v19_warmstart/`, md5 검증) — **원본 무접촉**. 검증: 유닛 57/57 +
> `_probe_moving_v19.py` 3모션 PASS + 2-iter 스모크. 학습 `exp021_mt_{cv,ct,ca}` 각 2048 envs ×
> 1000 iters(~4.05 s/iter), wandb `a6saa42b`/`29jqq1lu`/`ntumqwoz` (project drone-bombard-isaac).
> **det 200-ep eval(동일 플래그, wandb `exp021_eval_{cv,ct,ca}`=1nvvuogg/prdqujah/gdow3vfg):
> cv success 44.5%/release 81.5%/drop med 0.775 m · ct 33.8%/82.6%/0.783 m ·
> ca 16.5%/63.5%/1.063 m.** 판정: 릴리스 능력은 이월되나 **리드 부재로 명중이 성공반경
> 경계에 몰림**(released-miss>success; CCIP=현재 위치 조준, 낙하 t_f 동안 타겟 이동 |v|·t_f
> ≤2–3 m 구조적 편차). ca는 가속 이탈 OOR 25% 추가 병목. **개선 1순위 = 리드 개입**(Singer-KF
> obs v19 포팅 28→35 / privileged target-vel obs 28→30 분리검증 / Phase-3 w_lead 이식),
> 이후 target_accel 커리큘럼·게이트 리드 반영, iter 확대는 구조 개입 후(Rule 20f).
> play.py `--wandb` 잠복 NameError(`impacted` 미정의, a099de3 머지 이후) 수정. eval seed
> 미고정 → 동일 ckpt 32.0↔44.5% 표본 변동 확인(seed 옵션 추가 권장).
> ckpt: 컨테이너 `/workspace/exp021/` + 호스트 `/opt/drone-bombard/checkpoints/exp021/`.
> → [[experiments/exp_021_v19_moving_target]] / [[research/moving_target_models]] §5

> **🔀 (이전 2026-07-23): Isaac Lab exp_020 — 물리 페이로드 부착 첫 학습 완주, 학습 비용 0 확증 + wandb eval-figure 파이프라인** —
> `isaac_jk` 워크트리, `isaac-verify`/L4. Phase 1 `--release_terminal`, 2048 envs × 400 iters,
> **warm-start `exp018_B0_final.pt` + `--w_aim 1.0 --aim_reward_scale 0.5`(B0 보상 bit-match)**
> → `physical_payload=True`(cfg 기본)가 유일한 델타. wandb `o5jn9xzk`(train)/`vryuc6mu`(eval).
> **det 200-ep eval: success/release 100.00% (200/200), drop err mean 0.169 m(max 0.200),
> 호버-드롭 유지(final speed med 0.068 m/s).** release_rate 첫 롤아웃부터 100% 고정(B0의
> 23→99.6% 상승과 대조 — 재학습 과도기 없음) = exp_019 parity의 학습-스케일 확증.
> ⚠️ σ 1.41→1.71 드리프트(release 포화로 sharpen 압력 부재 — 장기 fine-tune 시 1순위 모니터),
> drop err 분포 tolerance 경계 이동(med 0.178). **`play.py --wandb` 신설**(eval 요약 스칼라 +
> 히스토그램 5종 + 종단 원인 bar chart, `job_type="eval"` — 학습 커브와 판정 figure 병치).
> 1차 기동은 컨테이너 빈 `WANDB_API_KEY`로 silent 실패(isaaclab.sh exit 0 삼킴) →
> **`docker exec --env-file /opt/drone-bombard/.wandb.env` 필수**. ckpt: 컨테이너
> `/workspace/logs/isaac_lab/drone_bombard/exp020_payload_final.pt` + 호스트
> `/opt/drone-bombard/checkpoints/exp020/`. **Stage C(DR+residual, 별도 지시 대기)는 물리
> 페이로드 포함으로 진행 가능 — 단 Phase-2 DR 힘 정합 선행 필요.**
> → [[experiments/exp_020_o5jn9xzk_payload_training]] / [[errors/err_20260723_wandb_key_empty]]

> **🔀 (이전 2026-07-21): Isaac Lab exp_019 — 물리 페이로드 attach/detach 구현·검증 완료 (4/4 PASS, Rule 24)** —
> `isaac_jk` 워크트리. 사용자 목표("페이로드를 실제로 달고 CCIP 근접 시 분리") 대응 코드
> 전수 검토 → 결함 6종 발견(①페이로드 물리 부재 ②`_payload_attached` 죽은 플래그 ③마커
> env-0 게이팅 ④ctrl_mass 스칼라 ⑤릴리스=즉시 종단 ⑥CCIP vz 생략), ①②③ 수정: 해석적
> 페이로드 → **실제 per-env RigidObject(0.1 kg 실린더) + kinematic weld**(GPU-복제 물리는
> per-env 조인트 토폴로지 변경 불가 → 부착 env만 매 physics step pose+vel write, CCIP 발화
> → release_delay 카운트다운 → write 중단 = 자유낙하, z≤0.10 m 측정 착탄 래치).
> hover-drop 강제 릴리스 검증(8 envs, isaac-verify): 부착 추적 **1.1 mm**, 분리 0/8 잔류,
> 착탄 8/8, **측정 vs 해석적 CCIP |Δ| mean 0.012/max 0.021 m**(물리↔해석 cm-parity).
> 보상·종단·referee **bit-identical**(순수 추가), `payload_impact_rate`/
> `payload_impact_err_measured_m` 신설, `physical_payload=False`로 구 경로 보존.
> 후속(별도 지시 대기): 에피소드 착탄-연장(⑤), Phase-2 DR 힘 정합, vz 복원(⑥),
> per-env ctrl_mass(④). 검증 스크립트 `isaac_lab/_test_payload_drop.py`.
> → [[experiments/exp_019_physical_payload]] / [[research/physical_payload_attach]] / Rule 24

> **🔀 (이전 2026-07-13): Isaac Lab exp_015 이어학습(2차) — P2/P3 extended 각 +2000 iters 완주
> (P2_EXIT=0, P3_EXIT=0)** — `feat/isaac-env-migration`. §7 baseline 체크포인트에서 페이즈별
> 단독 연장(`isaac-verify`, 2048 envs, `release_terminal`·`w_aim` **미적용**).
> **P2 ext(iter 1098→3097):** drop tail **2.87 m**(1차 2.91 m 대비 ~0.04 m), best_min 0.008 m
> 스파이크, release_rate **0.33→0.01** 급락, success **0**. **P3 ext(iter 3097→5096):**
> drop tail **5.31 m**(1차 3.20 m **회귀**), reward 101.7→**74.5**, lead tail 평탄 0.35,
> success **0**. **0.8 m 돌파 ❌** — iter 예산 확대만으로 릴리스-종단 명중 미형성(Rule 20f).
> exp_018 `release_terminal` 종단 재구조 필요 재확인. 산출물(host `logs/exp015_cont/`):
> `exp015_phase{2,3}_ext_final.pt`, `summary_p{2,3}_ext.json`, `p{2,3}_stdout.log`,
> `pipeline_done.txt`. git commit/push 미수행.
> → [[experiments/exp_015_phased_curriculum]] §8 / [[research/curriculum_phase_convergence]] §2(e) / Rule 20f

> **(이전 2026-07-12): Isaac Lab exp_015 실학습 — Phase 1→2→3 커리큘럼 첫 end-to-end 완주 (baseline, ORCH_EXIT=0)** —
> `feat/isaac-env-migration`. 오케스트레이터를 L4(`isaac-verify`, headless)에서 완주:
> `--phases 1,2,3 --phase_iterations 600,500,500 --num_envs 2048 --logger tensorboard --seed 42
> --log_root /tmp/exp015_orch`. **plain 커리큘럼**(exp_018 `--release_terminal`·exp_017 `--w_aim`
> **미사용** = exp_015 원본 릴리스 메커니즘). throughput **~2.3 s/iter(≈28 K steps/s)** → 예산
> 조정 없이 ~65 min 완주(P1 ~23 · P2 ~21 · P3 ~20 min). **결과(tail-mean 20 iter)**:
> **Phase 1(접근/nominal): success 0.48→1.00, reward→107** — exp_014 eval 100% 재현(해석적 CCIP
> `drop_impact_error_m` ~0.13 m). **Phase 2(CCIP+Residual+DR, 정지): reward −0.8→94.7 회복
> (~150 iter), `drop_impact_error_m` 4.66→2.91 m ↓(best 0.37), release_rate peak 0.98/tail 0.33
> 변동, success ~0**(DR 착탄 2.9 m ≫ 성공반경 0.8 m). **Phase 3(이동타겟): reward→102,
> `lead_error_m` best 0.071 m(tail 평탄 0.34), release 0.10, success ~0.** **warm-start 무손실
> 실증**: 페이즈 경계 reward 딥→빠른 회복(Rule 20b/20e). **정직 평가: reward 우상향은
> proximity 스트림 지배 — P2/P3 릴리스-종단 명중(sub-0.8 m)은 베이스라인 500 iter로 미형성.
> exp_016(근접≠릴리스)·exp_017(보상-단독 실패)의 결론을 커리큘럼 스케일에서 재확인. 뚫는
> 해법은 exp_018 릴리스=종단 구조(미적용) — P2/P3 추가 학습 또는 종단 재구조 필요.** 산출물
> 영속화(host `/opt/drone-bombard/isaac-worktree/logs/exp015_orch/`): `exp015_phase{1,2,3}_final.pt`
> (각 1.7 MB), `train_stdout.log`(2.9 MB, 파싱 소스), `metrics_phase{1,2,3}.csv`, `summary.json`,
> 수렴그래프 `notes/experiments/exp015_convergence.png`. TensorBoard는 컨테이너
> `/tmp/exp015_orch/drone_bombard_ppo/…`. git commit/push 미수행(노트만 편집).
> → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]] / Rule 20

> **(이전 2026-07-06, Stage B): Isaac Lab exp_018 — 릴리스-종단 구조로 release_rate 5.5% → 100% (Rule 23)** —
> `feat/isaac-env-migration`. exp_017(판정 b) 직후 사용자 지시 실행: ①근접 종단(d_xy≤0.8) →
> **릴리스-발화 종단**(`DroneBombardEnvCfg.release_terminal`, 기본 False=레거시 bit-identical;
> 실패 게이트 7종·타임아웃 불변 — 적대 검증 byte-미접촉 확인) ②aim_err 보상 **nominal CCIP
> 전용** 명문화(트리거는 residual-포함 유지, 보상 양은 미포함 — reward-hacking 차단)
> ③Stage-A 보상 탐색 재실행(단일 노브·소폭). **결과(전 런 v1 warm-start, 400 iters, det
> 200-ep)**: **B0(xt0hrr1c, 보상=Stage A v1 그대로, 종단만 교체): 학습 내 release_rate
> 23→99.6% 단조 상승**(Stage A의 12→3.7% 단조 하락 정확히 반전 — 잘림이 지배 요인이었음을
> 인과 확정), **det 100.00%, drop err 0.125 m**(max 0.198), 호버-드롭 수렴(종단 속도 med
> 0.11 m/s). 스윕: B1(w=1.5) 100%, B3(w=0) 100% — **aim 항은 종단 구조에서 사실상 잉여**
> (자동 발화 referee가 탐험 노이즈를 +100 샘플러로 전환 — Stage A의 노이즈 증폭·γ-할인
> 문제 둘 다 역전), B2(knee 0.75)만 근소 열화(98.5%). farm 시그니처 0(ep_len 52→36 감소
> 수렴, stagnation/timeout/overshoot 전 구간 0). **4-lens 적대 검증이 done-flag alias 버그
> 사전 발견**(`success = _just_released` alias를 `_reset_idx`가 in-place clear → eval이
> success 0%로 보고할 뻔; `.clone()` 수정, 학습/wandb는 무영향) — Rule 23d. ckpt 4종:
> 컨테이너 `exp018_{B0,B1,B2,B3}_final.pt` + 호스트 `/opt/drone-bombard/checkpoints/exp018/`.
> **Stage C(DR drag/wind + residual, 별도 지시 대기) warm-start = `exp018_B0_final.pt`.**
> Phase-2 본선 이식 시 주의: release_tolerance 0.5(Phase 2 기본) vs 0.2(referee), 호버-드롭
> 프로파일의 바람 강건성은 미검(Phase 2 설계 의도 그 자체).
> → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] / Rule 23

> **(이전 2026-07-06, Stage A): Isaac Lab exp_017 — 밀집 CCIP 조준 보상, 판정 (b) 정체 (Rule 22)** —
> `feat/isaac-env-migration`. exp_016의 release_rate 6% 갭에 대한 **보상-변경-단독** 개입(사용자
> 제약: 종단/성공 게이트·entropy_coef·비전 캘리브레이션 불변, DR/residual은 Stage B로).
> **구현**: referee와 동일 aim_err의 dense 항 $w_{aim}(1-\tanh(e_{aim}/s))$ —
> `RewardCfg.w_aim`(기본 0.0=off, exp_014 parity)·`aim_reward_scale`, train.py `--w_aim` 주입,
> `math_utils.aim_error_reward`+테스트(40/40). **5-lens 사전 적대 검증**: parity(w=0
> bit-identical)/timing/plumbing(rsl_rl warm-start=N iters 추가 실행 실증)/physics(CCIP-hold
> v=d/T가 0.2 m 윈도 26–28 연속 스텝 유지 실현가능) 생존; pathology가 w=2 duty-cycle 펌프
> PV(~137–180 vs 완주 ~149, γ=0.995) 경고 → **w=1 헤지 개시** + farm 모니터.
> **결과(3 runs, warm-start 체인 399→1399, det 200-ep)**: ① P1 6-dim 기준선 신규
> 학습(750gpldr; exp_015는 스모크만·exp_014 ckpt는 4-dim): 학습 내 release_rate 12→3.7%
> **단조 하락**(근접 최적화가 릴리스 능력 능동 파괴), det 2.5%. ② v1(6z0gpnhy, w=1/knee
> 0.5 m): det **5.5%**, aim_err_min med 1.146→**0.889 m**, final_speed 3.35→**2.72 m/s** —
> 방향 실재(단 release_rate 차이는 n=200 p≈0.13), 학습 내 지표는 σ-지터에 가려 평탄(Rule
> 22c). ③ v2(fv5qqmtz, w=2/knee 1.0, 600 it): **회귀**(3.5%/1.096 m/3.45 m/s) — rew_aim
> 8×는 행동 불변의 수동 소득, σ 1.18→1.55. **원인 3종(구조적)**: γ-할인 +100이 모든 감속
> 벌함 · CCIP가 탐험 노이즈를 ×1.53 s 증폭(그래디언트 평탄화, entropy 불변 제약) · 성공
> 조기 종단이 조준 구간 제거. farm 시그니처 0. **결론: 릴리스는 dense 사이드 보상이 아니라
> Phase 2 릴리스-종단 구조로 학습(별도 지시 대기). w_aim은 Phase 2+ 그대로 이월 금지
> (residual 채굴 경로).** ckpt 3종 분리 보존: 컨테이너
> `/workspace/logs/isaac_lab/drone_bombard/exp017_{phase1,stageA,stageA2}_final.pt` + 호스트
> 백업 `/opt/drone-bombard/checkpoints/exp017/` — **차기 warm-start 소스 = stageA(v1) 권장**.
> → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] / Rule 22

> **(이전 2026-07-05 저녁): Isaac Lab exp_016 — "success 100% vs drop_impact_error 4.59 m" 디커플링 규명·수정 (Rule 21)** —
> `feat/isaac-env-migration`. **진단(적대 검증 5/5 확정)**: Phase 1엔 릴리스 트리거가 존재하지
> 않았고(`DropCfg.release_tolerance=0.2`는 정의만 되고 미사용 — v15 `drop_calculator_node`의
> CCIP ≤0.2 m 트리거 의도가 이식에서 소실), `drop_impact_error_m`은 **d_xy≤0.8 성공-종단
> 스냅샷**(잔여속도 포함)의 탄도 예측 = **속도 캐리** $v\cdot(\sqrt{2H/g}+0.1)$ = 3.0 m/s ×
> 1.53 s ≈ 4.6 m. `final_speed_xy` 계측(2.99 m/s)으로 산수 봉합. **수정**: 스크립티드 CCIP
> referee(매 policy step, |예측착탄−타겟|≤0.2 m ∧ alt>1 m 최초 충족 시 래치) — **지표 전용,
> 보상/종단/RNG 무접촉(학습 동역학 bit-identical)**. `drop_impact_error_m`=릴리스 시점 재정의,
> 구 지표 `drop_impact_error_terminal_m` 보존, `release_rate`/`aim_err_min_m`/`final_speed_xy`
> 신설. **A2 재평가(6407f8d 백포트, 200-ep deterministic)**: 구 지표 4.649 m 재현 ✓, **새 지표
> 발화 시 0.137 m(10 Hz)/0.172 m(100 Hz referee), 단 release_rate 6.0%/11.5%** — CCIP 스윕
> 최근접 med 0.755 m ≈ d_xy_min 0.665 m: **근접(d_xy) 보상으로 학습한 정책은 0.2 m 릴리스
> 윈도우를 거의 못 통과(cross-track 지배, 샘플링 아님)**. exp_013의 24 m도 동일 의미론(실패
> 지배 극단값) — 디커플링은 exp_012 지표 도입부터 구조적, plant 수정이 노출시킨 것. **진짜
> 투하 능력은 exp_015 Phase 2(릴리스 조건부 보상)가 학습해야 하며, 본 수정으로 Phase 1↔2
> `drop_impact_error_m` 의미론이 정렬됨.**
> → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] / Rule 21

> **(이전 2026-07-05): Isaac Lab exp_015 — Phase별 순차 커리큘럼 코드 완료 (미학습)** —
> `feat/isaac-env-migration`. 이미지의 3단계 커리큘럼(접근/nominal → CCIP+Residual/정지타겟 →
> 이동타겟)을 env + `train.py`로 **완전 구현**. ① **action 4→6** (`[0:4]` 속도 setpoint +
> `[4:6]` CCIP 착탄점 residual δx/δy) — 전 페이즈 6-dim 고정으로 `runner.load()` warm-start
> 무손실. ② **`phase`(1/2/3) 단일 노브** + 파생 플래그(residual/dr/moving_target/release)로
> 페이즈 동작 유도. ③ **릴리스 이벤트**: 온보드 nominal CCIP + 정책 residual 예측이 (lead)타겟
> tol 내면 투하 래치 → 실제 DR 물리(drag `U[0,0.15]`·wind `N(0,1.5)`) 낙하로 **진짜 착탄오차**
> 산출 → `w_impact·exp(-err/scale)` 터미널 보상. ④ **Gauss-Markov 이동타겟**(OU) + lead 보상
> (Phase 3). ⑤ **`train.py --phases 1,2,3`** 서브프로세스 오케스트레이터(각 페이즈 `model_final.pt`
> → 다음 `--resume` 체이닝; Isaac Sim 프로세스당 1 sim 제약). Phase 1은 미사용 dim zero-out으로
> **exp_014 접근 태스크와 동작 동일**(baseline 유효). **검증: 로컬 `py_compile` 12파일 통과;
> `pytest test_math.py`(+8 신규 = 38) 및 본 학습은 dev 박스 torch 부재로 L4/컨테이너 대기.**
> reward/DR/GM/lead 하이퍼파라미터는 초기 추정 → L4 dry-run 신호로 튜닝. 후속: 진짜 정책-학습
> lead 위해 obs에 타겟 속도 2채널(14→16) 추가 검토.
> → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20

> **(이전 2026-07-05): Isaac Lab exp_014 — plant 수정 + 비전 거리감쇠 → eval 100.0% (202/202)** —
> `feat/isaac-env-migration`. 확정 버그 2건 수정(속도킥→스폰타임 `UsdPhysics.MassAPI` authoring,
> 로터 ±200 rad/s 리셋 재주입 제거) + `--zero-actions` 게이트 PASS(11.9m→0.2m). 수정 검증 중
> **inertia 대반전 계측**: `set_inertias`는 solver에 전파되고 있었음 → **exp_013은 rate loop
> ~1300× 저토크 plant에서 학습된 것**(구 정책은 plant-overfit로 무효, **Rule 19** 신설,
> [[research/isaac_inertia_ctrl_mismatch]]). probe 3종(재평가/궤적판독/보상반사실) 수렴으로
> A1 생략 → A2(slant-range conf 감쇠) 400 iters: **R_alt=0.0000**(climb 창발 150-199 →
> 50 iter 내 기각), success 99.85%, **noise_std 0.80 안정(폭주 없음 — 18b 재해석: σ 폭주도
> plant 아티팩트)**. A0′(감쇠 OFF 대조): R_alt 0.0365, success 96.5% → 지배 요인=plant 일관성,
> 감쇠=꼬리 제거+YOLO parity(유지). **deterministic 200-ep eval = 100.0%, d_xy_min 0.665m,
> drop_impact_error 4.59m** (exp_013: 36%/1.4m/24m). 실 YOLO 캘리브레이션은 컨테이너
> annotator 버그로 차단(하네스 수리 완료, 커브는 분석값 calibration-pending).
> **reward_success·entropy_coef 불변(사용자 지시)** — 다음 페이즈에서 재평가(σ 안정이라
> entropy 0 근거 약화). 다음 후보: success_radius 0.8→0.5 커리큘럼, 임무 지표(drop error) 트랙.
> → [[experiments/exp_014_A2_visionrange]] / [[sessions/session_2026-07-05]]

> **(이전 2026-07-03) Isaac Lab exp_013 — 첫 프로덕션 PPO 학습 완주·진단 완료** —
> `feat/isaac-env-migration` 브랜치. 2048 envs×1000 iters(65.5M steps, 43분, wandb `wcjklw7a`)
> → **deterministic 200-ep eval = 36%**, d_xy_min 1.4m plateau(게이트 0.8 밖). 기동 직후
> **비전 사멸 버그**(`_update_vision` env-origin 프레임 혼용 → 벡터화 시 conf≡0; `yolo_eval.py`
> 동일) 발견·수정 후 재기동. 실패 3원인 규명: ①analytic conf 거리감쇠 누락→고도 상승
> farming(max_alt 33%, **Rule 17**) ②farmer(+225)>finisher(+121) 보상 불균형 — Gazebo v14
> final-approach stagnation과 동일 병인(**Rule 18a**) ③noise_std 0.8→3.92 폭주(**Rule 18b**).
> **사후 검증: --zero-actions FAIL(11.9m) — 리셋 속도킥 버그가 run 전체 오염(36%는 오염 plant 수치). 다음: exp_014 = 0순위 킥 수정 → conf 거리감쇠 + reward_success 300 + entropy_coef 0, fresh.** 온보딩 문서
> 3종(reward_tuning/wandb_guide/experiment_workflow) 신설. jekyun SAC 트랙 영향 없음.
> → [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]] /
> [[errors/err_20260703_vision_env_origin_frame]]
>
> (이전 상태 — Phase 2 코드 이식·parity·29/29 테스트: [[experiments/exp_012_isaac_migration_phase2]] /
> [[research/isaac_velocity_controller]] / Rule 16)

> **⚠️ `rl_yolo_v15_bc_stable` (Gazebo/SAC 트랙, isaac_jk 분기 전 마지막 상태, 2026-07-05) — X마커 미도달 회귀 의심 (미확정) + ⛔ 차단 이슈: 인프라 GPU 드라이버 불일치.** 반복 reset-recursion abort로 오토레쥼 서포바이저(`run_train_supervised.sh`) 재개 반복 후 0→310K. **정정: 310K 정지는 계획된 eval-stop이 아니라 07-03 GPU 드라이버 업그레이드(Isaac Lab용, 535→580)로 `drone-bombard-harmonic` 컨테이너가 깨지며 강제 중단된 것.** 사용자 관찰: 훈련된 에이전트가 X마커에 도달 못 함(이전 v14 대비 회귀). **원인 후보 2건(여전히 미확정):** ① 근접-게이팅 속도 댐핑(`w_vel=0.08`, `vel_damp_radius=3.0m`)이 `success_radius=0.8m`보다 훨씬 넓어 v14의 기존 실패 구간(final-approach stagnation 0.5–1.2m)을 재타격 ② crash-resume마다 replay buffer 초기화(가중치만 복원)로 310K 스텝 수만큼의 연속 학습이 실제로는 없었을 가능성. **이 트랙은 인프라 정합 및 재평가 없이 여기서 멈춤(isaac_jk는 이후 Isaac Lab 전용으로 진행).** → [[daily/daily_2026-07-05_gazebo_v15_regression]] / [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/rl_rules]] Rule 25 / Rule 26
>
> **원 배경 (2026-07-01):** **RL wobble 교정** 적용: eval `deterministic=True`라 wobble=학습된 bang-bang 정책(탐험 노이즈 아님). LPF A/B로 **PX4 수신 속도명령 jerk RMS 2.92→1.61(−45%), 평균 속력 불변** → smoothness-control 문제 확정. 교정 = (B) 근접-게이팅 속도 댐핑 `w_vel=0.15/vel_damp_radius=4`(→ v13 base 이식 시 `0.08/3.0`으로 완화) + (C) `w_ang_vel 0.05→0.15`·`w_action_smooth 0.05→0.20` + 로직 LPF `velocity_lpf_alpha=0.4`(학습==배포). dry-run PASS(크래시 0). **Fresh Start가 v14 5체크포인트 삭제 → `rl_checkpoints/v14_backup/`에 백업.** → [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/control_smoothness_wobble]] / Rule 15

> **⚠️ 미커밋 변경 (2026-06-22~23, 검증 완료 — commit 결정 대기):**
> ① **핸드오프 윈도우 확장** — `target_altitude` 5→10 m + `vision_callback` 탐지 게이트(`min_detection_conf=0.5`, `detection_pixel_radius` 200→300) + `start_drift_max` 5→10. 핸드오프 2.7→5.0 m(~2배). → [[experiments/exp_008_dryrun_alt10_handoff_window]] / Rule 13
> ② **Soft reset (리셋 처리량 ~3.9×)** — `drone_drop_env.py`에 teleport 회피 리셋(`_try_soft_reset`) + `soft_reset_enabled: true`. **장기 검증 ✅: byxyaf4d 3096 resets, soft 성공 ~91%, EKF bounded, no teleport/no PX4 restart** → Rule 14 검증완료. → [[experiments/exp_009_softreset_throughput]] / [[experiments/exp_010_byxyaf4d_v14_195k_eval]] / Rule 14
> **⚠️ v14 정책 자체는 회귀(65%<80%):** byxyaf4d 195K eval = **65% (13/20)** vs v13 80%(16/20). 실패 전부 final-approach stagnation. 원인=미성숙(39% budget). **soft reset 인프라는 채택 가치 있으나 "v14를 baseline으로" 결정은 미정.** → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]

### 활성 학습

| 항목 | 값 |
|------|-----|
| Run name | `rl_yolo_v13_terminal_reward` (iyhfy5ps) — **2026-06-20 157.7K/500K에서 SIGTERM stop (eval 위해)** |
| 상태 | ⏸️ stop (plateau, ep_rew_mean ~100, success ~82%, target_lost 0). `sac_drop_preempt.zip`+70MB replay 보존 → 재개 가능 |
| 평가 결과 (확정 06-21) | **clean 20-ep eval: success 16/20 (80%), gate 0, EKF-drift 0, mean reward 134, mean closest 0.81m.** 4개 실패 전부 종단 stagnation(0.81–1.09m). 80%≈학습 ~82%. → [[experiments/exp_007_iyhfy5ps_v13_eval]] |
| 평가 fixes | health gate + YOLO 누수 fix + evaluate.py 재작성 (06-21, push fb69bb9). 발산 근본원인=YOLO 누수, fundamental EKF 버그 아님 |
| 로그 | `/workspace/train_v13.log` (학습) · `/workspace/eval_v13.log` (평가) |
| Timesteps | 500,000 목표 (157.7K에서 중단) |
| 수정 config | `arm_bail_timeout: 10.0 → **20.0**` (06-17 armdiag) |
| ⚠️ 인시던트 | armdiag dry-run이 v13 30K 체크포인트 파괴(YAML 중복 키) → 30K 재개 불가, fresh 재시작. → [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] |
| 검증 | fresh run 초기 윈도우: **bail 0 / late-EKF 14–15.5s ×3 전부 회복** (구 10s면 bail) |
| ⚠️ OPEN 이슈 | YOLO `target_lost_rate` ~29% bimodal; teleport 후 EKF 13–16s 재수렴 자체 |

**처리량 진단(06-17):** v13(46y4xtiw)이 ~10h에 29.9K(6%)뿐 — fps≈0.83, ETA ~6.5일.
지배적 싱크 = `PX4 not armed after 10s` early-bail. armdiag dry-run(xgzum51v)으로
`pre_flight_checks_pass` 재수렴을 계측: **bimodal 0.0s(7/12) / 13–16s(5/12 ≈ 42%)**.
25s 창에서 bail 0 / SUCCESS 4 → v12의 10s 컷이 복구 직전 단두대질이었음 규명.
**Fix: `arm_bail_timeout` 10→20.** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / [[research/cruise_timeout_arming]] / Rule 11.



### 활성 보상 공식 (2026-03-22 패치, 미적용)

| Layer | 공식 | 파라미터 |
|-------|------|---------|
| R1 Safety | `−10` if alt < 2 m; `−8` if speed > 20 m/s | step > 20 이후 |
| R2 Stability | `−0.05 − 0.05‖ω‖² − 0.05‖Δa‖²` | w_time=0.05 (패치됨) |
| R3 Distance | `1.0 × (d_prev − d_xy)` | 선형, 포화 없음 |
| R3 Orient | `1.0 × cos θ × min(v_xy/2, 1)` | speed gate (안티-밀킹 패치) |
| R4 Drop | `50·exp(−5·d_err) [+100 jackpot if d≤0.1m]` | auto-drop at d_xy≤0.5m |
| Truncation | `−50` if step=500 and not dropped | 패치됨 |

### 학습 환경

| 파라미터 | 값 |
|---------|-----|
| Algorithm | SAC, `net_arch=[256,256]`, `device=cuda` |
| `num_envs` | 1 (Gazebo lockstep 병목으로 고정) |
| `total_timesteps` | 1,000,000 |
| RTF | **2** (RTF dry-run 결과: RTF=2 최적, avg 59.5 fps) |
| 예상 fps | **~60** |

### 체크포인트

- **보상 패치 전 마지막 정상:** `sac_drop_preempt.zip` (run `8otphxy8`, ~114K steps)
- **다음 학습:** Fresh start 필요 (보상 공식 변경으로 재개 금지)

---

# 2. Recent Progress

- **2026-08-01 (repo housekeeping) — `main`을 `isaac_jk`로 승격 + `Isaac-JS` 브랜치 정리.** 10개 브랜치 전수 계보 조사로 `main`이 07-03 시점(Isaac Lab skeleton)에 정체돼 있고 실제 작업은 전부 `isaac_jk`에 있음을 확인. `Isaac-JS`(제균 개인 브랜치, 07-02 이후 Isaac Lab 코드 없이 Gazebo/SAC 문서만 추가)의 고유 연구노트(v15 회귀 진단, 보상함수 리뷰, Rule 25/26)를 `isaac_jk`로 포팅 후 브랜치 삭제. `main`을 `isaac_jk`로 force-update(구 `main`은 태그 `archive/main-pre-isaac_jk-promotion`으로 보존 — junsang `_junsang` 노트·초기 `isaac_lab_tasks/` 스켈레톤 포함). push 용량초과 보고의 실제 원인도 규명: `Isaac-JS` 마지막 커밋의 `git add .`가 SAC replay buffer/영상 ~166MB를 실수로 포함시킨 것(브랜치 삭제로 해소, repo 전체 히스토리 bloat는 별도 과제). `Issac_JS`(junsang)는 미변경 — 세션 중 junsang의 신규 커밋(v20 task 등록)이 아직 `main` 미반영. → [[daily/daily_2026-08-01]]
- **2026-07-21 (Isaac Lab, exp_019) — 물리 페이로드 attach/detach: kinematic weld 구현 + hover-drop 검증 4/4 PASS.** 코드 전수 검토에서 릴리스 경로 결함 6종 발견(핵심: `_payload_attached`가 발화 시 False로 전환되지 않는 죽은 플래그 + 페이로드 자체가 물리적으로 부재). per-env RigidObject 페이로드 신설 — 부착=매 physics step pose+vel write(조인트 불가 제약 우회), 분리=CCIP 발화 후 release_delay(0.1 s) 카운트다운 만료 시 write 중단, 착탄=z≤0.10 m 래치로 측정 오차 기록. 검증: 추적 1.1 mm/분리 0 잔류/착탄 8/8/**측정 vs 해석적 |Δ| max 0.021 m**. 보상·종단 bit-identical, 기존 마커 env-0 버그도 수정. `play.py`에 payload_impact 통계 추가. → [[experiments/exp_019_physical_payload]] / [[research/physical_payload_attach]] / Rule 24
- **2026-07-03 (Isaac Lab migration) — README 컨테이너 진입 절차 수정 + `play.py` 4-tuple 버그 fix.** 사용자가 혼자 재현 시도 시 실패 원인 규명: README의 `docker pull drone-bombard-isaac:latest`가 가리키는 이미지는 로컬에도 GCP Artifact Registry(`isaac-lab` 저장소, 0 items)에도 **존재하지 않음** — pull 대상이 없었음. 실제로는 `isaac-lab-local:580` 이미지로 띄운 `isaac-verify` 컨테이너가 이미 dev VM에 떠 있었음. README §5에 "이미 떠 있는 컨테이너 재사용" 절 신설 + non-root exec 시 root 소유 캐시(`/isaac-sim/kit/cache` 등)로 인한 `PermissionError` 크래시 및 chown 해결법 문서화 + `PYTHONUNBUFFERED=1` 팁(미설정 시 `simulation_app.close()`의 하드 종료로 마지막 PASS/FAIL print 유실) + README 전체의 `./isaaclab.sh`(존재하지 않는 상대경로, `/workspace/drone-bombard`에서 cwd 불일치) → `/workspace/isaaclab/isaaclab.sh` 절대경로로 일괄 수정(14곳). 검증 중 `isaac_lab/play.py`의 `run_zero_actions`/`run_scripted`/`run_policy`가 `RslRlVecEnvWrapper`(rsl_rl 4-tuple `obs,rew,dones,extras` API) 사용 중임에도 5-tuple(`obs,rew,terminated,truncated,info`) unpacking을 시도해 **항상 `ValueError`로 즉시 크래시**하던 버그 발견·수정. 수정 후 `--zero-actions`는 실행은 되나 altitude drift 12m/100 step로 FAIL(`verify_one_episode.py`는 동일 env로 148-step 안정 호버 PASS) — wrapper 경로 자체의 미해결 회귀 가능성, 후속 조사 필요. → [[isaac_container_access]] (Claude memory)
- **2026-07-03 (Isaac Lab migration) — 실제 실행 검증 통과 (VERIFY: PASS).** 사용자 요청으로 이 dev 박스에 `isaac-sim:5.1.0` pull → Isaac Lab v2.3.2+rsl_rl 설치 → `verify_one_episode.py`(신규 무학습 하네스)로 `Isaac-DroneBombard-Direct-v0` **1 에피소드 실제 실행**. env 구성·USD 씬(드론)·질량 오버라이드(2.173kg)·reset(obs (1,14))·**148스텝 안정 호버**(고도 유지, 중력보상)·obs/reward/termination 유한(NaN 0)·stagnation guard 정상 발동. **실행으로만 잡히는 env/컨트롤러 버그 5종 수정**(핵심: rate loop 토크에 관성항 누락 → ~46× 과토크 → 즉시 스핀아웃; `τ=I·(k_rate·rate_err)`로 수정). + 이미지 자체 버그 2종(dangling `_structures.py` 심링크, core isaaclab 미설치) Dockerfile 반영. **렌더링/GUI는 driver 535<580으로 이 박스에서 불가** — 물리/CUDA 정상, 시각화는 L4 VM 필요(사용자: headless 검증 수용). 커밋 `f2f1b1a`. → [[experiments/exp_012_isaac_migration_phase2]] §6b / [[research/isaac_velocity_controller]]
- **2026-07-03 (Isaac Lab migration, `feat/isaac-env-migration` 브랜치):** **Phase 2 — env+PPO 코드 이식 완료.** 별도 워크트리(`git worktree add /opt/drone-bombard/isaac-worktree feat/isaac-env-migration` + `git merge jekyun`, merge `940c88b`)에서 진행 — jekyun의 라이브 v15 학습(tmux `rl_train`) 방해 없음. `isaac_lab/` 신설: `math_utils.py`(action rate-limit/LPF, pinhole vision projection, hold-buffer, ballistic/CCIP, 3-layer reward, overshoot/stagnation guard — 순수 torch, isaaclab 무의존) + `drone_bombard_env.py`(DirectRLEnv, 위 math_utils를 isaaclab lifecycle에 연결) + `agents/rsl_rl_ppo_cfg.py` + `train.py`/`play.py`/`yolo_eval.py`. v13/v15 obs(14)·action(4)·reward·termination 상수 전부 parity 이식(표: [[experiments/exp_012_isaac_migration_phase2]]). SAC→PPO(rsl_rl), target/spawn 랜덤화 신규(Gazebo는 고정 타겟), vision=학습 시 analytic pinhole(YOLO 캘리브레이션 노이즈)+평가 시 실제 YOLOv8 이원화, drop=액션 아닌 스크립트 CCIP 메트릭(태스크 스코프 v15와 동일). Phase-2 훅 4종(CCIP residual, release 상수 cfg화, obs superset 고정, domain-rand 스텁) 비활성 wiring — Phase 1 출력 불변. **검증:** `pytest isaac_lab/tests/test_math.py` **29/29 통과**(drone-bombard-harmonic 컨테이너, torch 2.4.1, isaaclab 미설치 — 파일 경로 직접 로드로 패키지 `__init__` 우회). `py_compile` 전체 통과. L4 Spot VM 미기동 → env 스모크·실제 학습 미실행(README에 정확한 커맨드 문서화). 부수 발견: Gazebo `hyperparams_v13.yaml`의 `limit_tilt:0.26`는 코드에서 미사용인 죽은 설정(실제 게이트는 `limit_inverted_tilt=1.047` 기본값) — 이식 안 함. Overshoot guard가 success_radius=0.8에서 도달 불가능한 것은 Rule 10의 의도된 설계임을 Gazebo 소스로 재확인(버그 아님) — 비종단 진단 카운터만 신설. 신규 **Rule 16**(시뮬레이터 이식 시 plant/reward parity는 상수뿐 아니라 타이밍+메커니즘까지 검증). → [[experiments/exp_012_isaac_migration_phase2]] / [[research/isaac_velocity_controller]] / [[research/rl_rules]] Rule 16
- **2026-07-05 (Gazebo/SAC 트랙, isaac_jk 분기 전 마지막 기록) — v15(310K) "X마커 미도달" 진단 (코드 변경 없음).** 사용자 관찰: wobble 교정판 모델이 이전 모델(v14)과 달리 X마커에 도달 못 함. 학습 이력 재구성 결과 v15는 잘못된 base config로 첫 크래시 후 수정, 이어서 **반복 reset-recursion abort**로 오토레쥼 서포바이저 추가, 0→310K에서 preempt — **그 이후 정식 eval이 한 번도 기록되지 않았음**을 확인. 원인 후보 2건(미확정): ① `vel_damp_radius=3.0m`가 `success_radius=0.8m`보다 넓어 근접-속도 댐핑(B)이 v14의 기존 final-approach stagnation 구간(0.5–1.2m)을 재타격했을 가능성 ② `run_train_supervised.sh`의 resume이 가중치만 복원하고 replay buffer는 매번 초기화 — 반복 abort가 있었다면 310K 스텝만큼의 연속 학습이 실제로는 없었을 가능성. **진단 중 GPU 드라이버 불일치(호스트 580 vs `drone-bombard-harmonic` 컨테이너 NVML 535, Isaac Lab 드라이버 업그레이드 여파) 발견 — evaluate.py 실행 자체가 reset-recursion으로 실패해 원인 확정이 차단됨.** 이 트랙은 인프라 정합 없이 여기서 멈춤(이후 이 워크트리는 Isaac Lab 전용). → [[daily/daily_2026-07-05_gazebo_v15_regression]] / [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/rl_rules]] Rule 25 / Rule 26
- **2026-06-23:** **v14(byxyaf4d) plateau stop + 195K eval = 65% + soft reset 장기검증 통과.** ep_rew_mean이 70K부터 ~120–135 평탄(125K step 정체) 확인 → **196.5K/500K(~39%)에서 SIGINT graceful stop**(`sac_drop_195000_steps.zip` 보존, sim 점유 해제). **clean 20-ep deterministic eval = 성공 65%(13/20)** — v13 80%(16/20) 대비 **회귀**. 실패 7 전부 **final-approach stagnation**(~0.5–0.8m 접근 후 0.50m gate 직전 정체; min 0.52/0.63/0.70/0.79/0.79/1.09/1.19, ep18은 2cm차). 항법·탐지 아닌 **종말 거리 좁히기 약점.** EKF 귀책 실패 0(eval중 health-gate EKF divergence 2회 전부 full-restart self-heal 후 성공). **Soft reset 장기검증 ✅:** stop 직전 env `attempts=3096 success=2826 skipped=118`(soft ~91%, no teleport/no PX4 restart) → exp_009의 미해결 질문(학습 정책 fallback율·EKF drift bounded) **해소, Rule 14 검증완료.** **회귀 원인=정책 미성숙**(39% budget, terminal-tightening은 막판 sharpen 스킬; reward plateau는 거친 정책의 평탄). 추가 가설: 10m 고도(v13 5m)로 최종 하강↑. **비디오:** `record_flight.py`+3-ep evaluate 동시 → 3/3 success(0.48–0.50m) 캡처 → `rl_eval_results/v14_195k_flight_annotated.mp4`(2.1MB, YOLO 박스) + `_raw.mp4`(3.7MB). headless `gz sim -s`라 onboard annotated가 산출물. **v14 commit 결정 대기**(soft reset 인프라 채택 vs 65%<80% baseline 채택). → [[experiments/exp_010_byxyaf4d_v14_195k_eval]] / Rule 14
- **2026-06-22 (오후):** **리셋 처리량 ~3.9× — soft reset로 teleport-EKF 병목 회피.** v14 fps≈2, ETA ~2.5일의 원인을 계측으로 확정: 에피소드마다 CRUISE timeout(~42s)+full restart(~22s)이고, 근본은 teleport+disarm 후 PX4 **EKF 추정기 재수렴** 대기(`ctrl_0.log: Delaying arm — pre_flight_checks_pass=False`). **fresh restart도 동일 timeout**(restart≈handoff). **EKF2_GPS_CHECK 0 A/B = 음성**(airframe가 이미 `COM_ARM_WO_GPS 1` → GPS는 게이트 아님; 실제 게이트=EKF validity/innovation 수렴, 바이패스 param 설계상 없음) → param/timeout 레버 고갈 규명. **해결 = soft reset**(`drone_drop_env.py _try_soft_reset`): 종료 시 flyable이면 disarm/teleport 없이 armed+airborne 유지하고 position setpoint로 출발점 복귀(controller 살려둬 offboard heartbeat 유지) → mission_manager FSM만 재시작 → 재핸드오프. EKF 미교란 → 재수렴 0. **프로토(9.2min/32회): throughput 0.93→3.61 handoffs/min(~3.9×), fps 2→9, reset 65s→11s, soft 성공 32/32(100%), EKF d_xy 안정 4.5–5.8m(발산 없음).** flyable 아니면 기존 teleport+restart fallback(downside bounded). full run `rl_yolo_v14_softreset`(byxyaf4d, fresh 500K, online) 기동 — 장기 EKF drift/fallback율 검증 중. **미커밋.** → [[experiments/exp_009_softreset_throughput]] / [[research/reset_throughput_bottleneck]] / Rule 14
- **2026-06-22:** **핸드오프 윈도우 확장 — 고도는 레버 아님, 탐지 게이트가 진짜 레버.** 사용자 요청("X마커가 늦게=거의 머리 위 탐지돼 RL 핸드오프 후 학습 윈도우가 짧다"). **시도 1(고도만 10 m):** `target_altitude` 5→10 + `start_drift_max` 5→10 정합. **실패** — clean 핸드오프 여전히 d_xy 2.7 m(베이스라인 동급), 3 ep 중 2가 순항-시작 spurious(conf=0.00, d_xy≈11 m, 넓어진 FoV의 X-like 지면 FP) → health gate 발동/abort. 원인: 마커 apparent size ∝ 1/고도(YOLO 늦게 lock) + `vision_callback` 200 px 필터가 핸드오프를 머리 위로 클립. **시도 2(10 m + 탐지 게이트):** `vision_callback`에 confidence 게이트(`min_detection_conf=0.5`; real 마커 conf 0.73–0.95 vs 지면 FP ≤0.45) + 공간 필터 200→300 px(`detection_pixel_radius`; real off-center 264–293 px 조기 accept). **dry-run PASS: 핸드오프 d_xy 2.7→5.0–5.2 m(윈도우 ~2배), spurious 0, EKF-drift 0, conf 0.93.** dry-run 격리(`--checkpoint-dir rl_dryrun_alt10` + offline)로 메인 체크포인트 보호. **코드 변경 미커밋**(사용자 직접 커밋 예정). 기하+탐지 변경이라 fresh 필수 아니나 핸드오프 3.5→5 m 변화로 정책 초반 재적응 예상. → [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13
- **2026-06-21:** **eval 발산 흡수 루프 근본 원인 규명 & 수정 3종 (health gate + YOLO 누수 + evaluate.py).** 06-20 발견한 EKF↔camera 발산 루프의 진짜 원인 = **YOLO `xmarker_detector` 누수**: `_start_infra`가 fresh-start마다 YOLO 노드를 죽이지 않고 새로 spawn → 누적(검증 시 3개) → 충돌하는 pixel_coords 발행 → spurious CRUISE→TRACKING(conf=0.00) + EKF↔camera 불일치. **fundamental EKF 버그 아님** (clean slate에선 handoff 0.9m 정상). **수정:** ① `drone_drop_env.py` reset() step 8b **health gate**(`d_xy_prev>start_drift_max(5.0)`면 full restart+progressive settle 후 retry, max_retries(6) 초과 시 loop 대신 abort), ② fresh-start kill 리스트에 `xmarker_detector` 추가(누수 차단), ③ `evaluate.py` 재작성(success_rate/step-to-reach/closest d_xy from obs[12,13]/outcome breakdown; 죽은 `info['drop_error_actual_m']` NaN 의존 제거). config `hyperparams_v13.yaml`에 `start_drift_*` 추가. **Dry-run PASS(clean slate, 3/3 SUCCESS, gate 0회, mean reward 162, report NaN 없음).** `--symlink-install`이라 src 편집 live, rebuild 불필요. → [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12
- **2026-06-20:** **v13 정책 평가 + 학습 stop.** iyhfy5ps가 157.7K(~32%)에서 **plateau**(ep_rew_mean ~100 80K부터 평탄, success ~82%, target_lost 0, fps≈0). eval을 위해 학습 **SIGTERM graceful stop**(preempt+70MB replay 보존, 재개 가능). deterministic eval(`sac_drop_preempt.zip`, 20-ep 요청/13 실행): **ep 1–3 전부 0.8m 성공(reward 126/114/132, step 41–72; 평균 124 > 학습 ~100) — 정책 양호.** **ep 4–13 전부 step1 EKF divergence(`d_xy≈11.9m`=home→target) → −15 truncation 흡수 루프** (연속 full-restart가 EKF 수렴 못 시킴; 카메라는 마커 봄=TRACKING OK이나 EKF position만 발산; 06-17 EKF 재수렴과 동일 뿌리, eval에서 누적 악화). **harness 결함 2종:** `evaluate.py`가 env 미emit 키 `info['drop_error_actual_m']` 의존 → miss-distance/CEP/drop-speed 전부 NaN; v13 env는 0.8m 성공원 종료(탄도 투하 미모델링) → CEP 비실재 → success-rate/step-to-reach로 평가해야. **다음:** 에피소드 시작 EKF↔카메라 health gate(drift면 retry) + evaluate.py 지표 교체 후 재평가. → [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12
- **2026-06-17 (오후):** **v13 fresh 재시작(iyhfy5ps) + 인시던트.** 30K 재개를 시도했으나 armdiag dry-run이 **YAML 중복 `checkpoint_dir` 키**(격리 경로가 main에 덮임)로 메인 dir에서 실행되어 v13 30K 체크포인트 5개 삭제 + preempt를 599-step으로 덮음 → **30K 디스크 복구 불가**. 사용자 결정으로 **fresh 재시작**(arm_bail=20, 0→500K). 프로덕션 검증: 초기 윈도우 **bail 0, late-EKF 14.1/14.8/15.5s ×3 전부 회복**(구 10s면 30% bail). 재발방지: 파괴적 fresh-start 전 startup `Checkpoints:` 로그로 격리 검증. → [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]]
- **2026-06-17:** **arm_bail 처리량 병목 진단 & 수정.** v13(46y4xtiw)이 ~10h에 6%(29.9K)뿐, ETA ~6.5일 — 지배적 싱크가 `PX4 not armed after 10s` bail임을 확인. 컨트롤러에 `PREFLIGHT-PASS` dt 계측 추가 후 격리 dry-run(`hyperparams_v13_armdiag.yaml`, `arm_bail_timeout=25s`, offline). **결과: EKF 재수렴 bimodal — 0.0s(7/12) / 13–16s(5/12 ≈ 42%), 25s에서 bail 0 / SUCCESS 4.** v12의 10s 컷이 recoverable-with-time을 full-restart-only로 오판하고 복구 직전(3–6s 전) 단두대질했음. **Fix: `hyperparams_v13.yaml` arm_bail_timeout 10→20.** v13은 SIGTERM emergency save로 30K+리플레이 보존, 재개 가능. → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11
- **2026-06-15:** **Arming-rejection throughput fix.** `rl_yolo_v11_cam_fix`(k1uqgs8i) 분석 → 443 CRUISE 타임아웃의 근본 원인이 teleport 후 stale EKF arm 거부(28.2% NEVER ARMED, 전부 attempt 1/3)임을 규명. 수정 3종 적용: (#3) `/fmu/out/vehicle_command_ack` arm 거부 사유 로깅, (#2) `pre_flight_checks_pass` 게이팅, (#4) `arm_bail_timeout=10s` early-bail → 즉시 full infra restart. colcon build clean + dry-run(400 step, 0 타임아웃) 검증 후 fresh run `rl_yolo_v12_arm_fix`(500K) 기동. ⚠️ 정정: `cruise_poll_timeout`은 이미 20.0s(이전 "60s"는 fallback 기본값 오독). ⚠️ OPEN: YOLO target_lost_rate ~29% bimodal 미해결. → [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]]
- **2026-06-13:** v7 패치 적용 후 fresh run `rl_yolo_v7_drift_guard` (WandB: `7lhjy40o`) 시작. EKF drift guard (step1 d_xy>5m→truncate), proximity 4m→2.5m, penalty_target_lost -0.5→-0.1, stagnation_start_step 400→50.
- **2026-06-12:** Vision 기반 RL 학습 인프라 완성. EKF East 반전 버그 2종 수정. fresh run `rl_yolo` (WandB: `45l8vkw5`) 121K steps. **분석: target_lost_rate=1.0 원인 = EKF drift (dominant) + 카메라 FOV gap 3차진 이후 2.89m vs 시작 d_xy 3.5m).** run 폐기.
- **2026-04-16:** RTF dry-run 3종 완료 (RTF 1/2/4). **RTF=2 최적** (avg 59.5 fps, 61s/4Kstep). RTF=4는 Python 병목으로 역전. Exp 002 RTF=2로 결정.
- **2026-04-16:** WandB API key 영구 연결 (`/opt/drone-bombard/.wandb.env`, `--env-file` 방식). Docker image `drone-bombard-px4built:latest` — PX4 빌드 + 커스텀 airframes 4016-4019 포함.
- **2026-04-14:** Obsidian 연구 비서 시스템 초기화. `notes/` 구조 구축, CLAUDE.md + RL_Project_Log.md 간소화.
- **2026-03-22:** 보상 해킹 분석 → 4개 anti-milking 패치 적용 (학습 대기 중).
- **2026-03-20:** 선형 거리 보상 도입 (지수 포텐셜 교체), CRUISE retry, 3중 물리 폭발 방어.
- **2026-03-20:** Method A (1-World-4-Payload) 아키텍처 완성 및 dry-run 통과 (31 fps).
- **2026-03-19:** 자기관리 인프라 안정화 (z=0 스폰, COM_OF_LOSS_T=10s, fps=30-31).

---

# 3. Remaining Tasks (Next Steps)

## 2026-08-23 이후 착수 순서 (research_architecture v3 §11)

| 순위 | 작업 | 상태 |
|---|---|---|
| 0 | CCIP `vz` 수정 | ✅ 완료 2026-08-23 |
| 0 | **T3 오라클 재정의** (실 PhysX 드리프트 기준) — B4·Oracle gap·Abstract 의존 | ❌ |
| 1 | 페이로드 항력 `is_global=True` | ❌ |
| 1 | `_drag_coef` 팬텀 채널 정리 | ❌ |
| 1 | **T0~T3 Table 1 재측정** (vz 수정이 T2 horizon·T3 오라클에 영향) | ❌ |
| 2 | DR 축 정리(질량/Cd 병합, CoM 삭제) + **DR_SCALE 노브 분리** | ❌ |
| 2.5 | **actor 관측에서 wind/drag 제거 단독 실험** — 주 주장 #3의 전제 | ❌ |
| 3 | 관측 재설계: heading-invariant 프레임 + 비대칭 critic (함정 3종 주의) | ❌ |
| 4 | 주파수 변경 시 **per-second 보상 재스케일 표** (10→50 Hz면 per-step 5배, loiter는 ~25배) | ❌ |
| 4 | 고도 범위 vs 종료 조건 정합 (`min_altitude=3.0` / `release_alt_min=3.0` → 4 m 시작이면 창 1 m) | ❌ |
| 0 | CCIP `vz` 수정 / T3 오라클 재정의 / 항력 `is_global` / 팬텀 채널 | ✅ 완료 2026-08-27 (`59755e7`, `f0ef4b8`) |
| 2 | DR 축 정리 + DR_SCALE 노브 분리 | ✅ 완료 2026-08-27 (`f0ef4b8`) |
| 2.2 | DR_SCALE 스윕 유효성 게이트 | ✅ **통과** 2026-08-27 (exp_025) |
| 5 | `release_delay` 실 latency 플랜트 구현 | ✅ 완료 2026-08-27 |
| 5 | success_radius 1.0 → 0.5 m | ✅ 완료 2026-08-27 (exp_026) |
| 5 | 릴리즈 판정 10 → 100 Hz | ✅ 완료 2026-08-27 (exp_026) |
| **A** | **헤드라인 지표 교체 CEP50 → CEP90 / 성공률(0.5 m)** — 오라클 갭이 꼬리로 이동 | ❌ |
| **A** | **T1/T2 동일화를 Table 1에서 어떻게 제시할지 결정** (행 병합 vs 10 Hz 열 병기) | ❌ |
| **B** | **급기동 도입 여부 결정** — [[research/agility_ceiling]] §4 단계별 변경. `reveal_radius`와 속도는 반드시 세트 | ❌ |
| 3 | 관측 재설계: heading-invariant 프레임 + 비대칭 critic | ❌ (급기동 도입 시 선행 필요) |
| 2.5 | actor 관측에서 wind 제거 단독 실험 | ❌ |
| — | T0~T3 최종 Table 1 n≥500 | ❌ |


- [ ] **(repo housekeeping)** `Issac_JS`의 신규 커밋(v20 task/flag 등록, junsang이 세션 중 push)을 `main`/`isaac_jk`에 merge — 아직 미반영.
- [ ] **(repo housekeeping, 선택)** git 히스토리 다이어트 — SAC replay buffer·YOLO epoch 체크포인트·mp4가 여러 브랜치 히스토리에 누적(pack 320MB). `git filter-repo` 필요, 파괴적 작업이라 사용자 확인 후 진행.
- [ ] **(repo housekeeping, 선택)** `donghyeok`/`junsang` 브랜치(둘 다 어디에도 머지 안 된 고아 브랜치) 처리 방침을 팀에 확인.
- [ ] **(Gazebo/SAC 트랙, 보류 — isaac_jk는 이 트랙을 더 이상 진행하지 않음)** v15 회귀 원인 미확정 상태로 인프라 차단(GPU 드라이버 535/580 불일치)됨. 재개하려면: 드라이버/컨테이너 정합 → `evaluate.py` 재실행 → `vel_damp_radius` 축소 또는 `w_vel→0` 적용 여부 결정. → [[daily/daily_2026-07-05_gazebo_v15_regression]] / Rule 25 / Rule 26
- [x] **(Isaac Lab, exp_019)** 물리 페이로드 attach/detach — kinematic weld 구현·검증 완료(4/4 PASS). → [[experiments/exp_019_physical_payload]]
- [ ] **(Isaac Lab, exp_019 후속 — 별도 지시 대기)** ⑤에피소드를 페이로드 착탄까지 연장(발화-종단 → 착탄-종단 + 측정 착탄오차 터미널 보상; 종단 의미론 변경 = fresh 학습) ⑥CCIP vz 항 복원(`t=(vz+√(vz²+2gH))/g`) ②Phase-2 DR(drag/wind) 힘을 물리 페이로드 낙하에도 적용(해석↔물리 정합) ④per-env `_ctrl_mass` 텐서화(분리 후 팬텀 0.1 kg). → [[research/physical_payload_attach]] §트레이드오프
- [x] **(Isaac Lab, exp_015)** Phase별 순차 커리큘럼 **코드 구현 완료** — action 4→6 residual, phase 노브+파생 플래그, 릴리스 이벤트+DR 착탄오차 터미널 보상, Gauss-Markov 이동타겟+lead, `train.py --phases` 오케스트레이터. `py_compile` 12파일 통과. → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20
- [x] **(Isaac Lab, exp_015)** 본 학습 완주 — `--phases 1,2,3 --phase_iterations 600,500,500 --num_envs 2048`(baseline, ~65 min, ORCH_EXIT=0). Phase 1 완전 수렴(success 1.00), P2/P3는 방향성 신호(drop↓/lead best 0.071m)만·명중 미형성. → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]]
- [ ] **(Isaac Lab, exp_015 후속)** P2/P3 명중 능력 확보: (a) exp_018 `release_terminal` 구조를 Phase 2·3에 적용(warm-start=exp018 B0 검토) 또는 (b) P2/P3 iteration 대폭 증량. PhaseCfg(residual_scale/release_tolerance/w_impact/w_lead) 튜닝. → [[research/curriculum_phase_convergence]]
- [ ] **(Isaac Lab migration, 병행 트랙)** L4 Spot VM 기동(`infra/deploy.sh` 빌드+push, `infra/startup.sh` 실행) → Cartpole 스모크 → `Isaac-DroneBombard-Direct-v0` env 스모크(2-iter) → `play.py --zero-actions/--scripted` 물리 검증. → [[experiments/exp_012_isaac_migration_phase2]]
- [ ] **(Isaac Lab migration)** PX4 속도-스텝응답 Gazebo 캡처 세션(`vel_logger_v2.py` 신규, 7-포인트) → Isaac 컨트롤러 게인 검정. 현재 미검정(구조 일치, 게인 초기값). → [[research/isaac_velocity_controller]]
- [ ] **(Isaac Lab migration)** `yolo_eval.py --calibrate` 첫 실행 → vision 캘리브레이션 v1(현재 v0=스펙 추정).
- [ ] **(Isaac Lab migration, 게이트 조건부)** `feat/isaac-env-migration`(이 워크트리)에서 `ros2_ws/`/`gazebo_models`/PX4 파일 정리 — **jekyun(라이브 SAC)는 대상 아님, 절대 미삭제.** 위 2개 항목(env 스모크 통과 + PX4 스텝응답 캡처) 완료 전까지 보류(사용자 확인, 2026-07-03). → [[experiments/exp_012_isaac_migration_phase2]] §8
- [x] **Vision 기반 RL 인프라 완성** — YOLO + SAC 시각 서보잉 파이프라인 구축
- [x] **EKF East 반전 버그 수정** — proximity target + RL env reward target 좌표 수정
- [x] **WandB `45l8vkw5` 100+ 에피소드 분석** → target_lost=1.0, EKF drift 확인 → 폐기
- [x] **EKF drift 방어 로직** — step 1에서 d_xy>5m이면 즉시 truncate (ekf_drift)
- [x] **fps 개선 — CRUISE 타임아웃 근본 원인 수정** — arm 게이팅(#2) + early-bail(#4). v12에서 효과 검증 중 → [[research/cruise_timeout_arming]]
- [x] **v12/v13 arm 처리량 재진단** — `ARM REJECTED` 0회(게이팅 작동), 실제 병목은 EKF 재수렴이 10s bail 초과(13–16s). **arm_bail 10→20s 적용** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11
- [x] **v13 재시작** — 30K 체크포인트 인시던트로 소실 → **fresh 재시작**(iyhfy5ps, arm_bail=20). 검증: bail 0, late-EKF 회복 확인.
- [x] **핸드오프 윈도우 확장** — 고도↑(레버 아님) 기각, **탐지 게이트 수정**(conf 0.5 + 공간필터 200→300 px)으로 핸드오프 2.7→5.0 m(~2배), spurious 0. dry-run PASS, **미커밋** → [[experiments/exp_008_dryrun_alt10_handoff_window]] / Rule 13
- [x] **리셋 처리량 병목 규명 + 수정** — 원인=teleport 후 EKF 재수렴(param 불가). **soft reset로 ~3.9×** 프로토 검증 → [[experiments/exp_009_softreset_throughput]] / Rule 14
- [x] **soft reset 장기 검증** — byxyaf4d 3096 resets에서 soft 성공 ~91%, EKF bounded, no teleport/no PX4 restart. **Rule 14 검증완료.** → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- [ ] **⚠️ v14 정책 회귀(65%<80%) 해소** — 195K eval 실패 전부 final-approach stagnation(미성숙 가설). 옵션: ① soft reset 켠 채 500K까지 재개/연장(미성숙 검증) ② 10m→5m 고도 A/B(종말 거리 가설) ③ 종말 보상 shaping 강화. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- [ ] **⚠️ v14 commit 결정 (사용자)** — soft reset/탐지게이트 인프라는 검증완료지만 정책 65%<80%. "v14를 validated baseline으로" 채택 여부 결정 필요. 윈도우 확장 + 탐지게이트 + soft reset 코드 일괄 미커밋 상태.
- [ ] **v13(iyhfy5ps) 추세 점검** — 첫 롤아웃 후 success_rate 발생 + ep_len/env/ep_reward 추세 + 전체 bail율(구 ~21/h 대비)
- [ ] **(장기) teleport EKF 재수렴 단축** — 13–16s 재수렴 자체 줄이기(명시적 EKF reset 등). 타임아웃은 증상 완화일 뿐
- [ ] **⚠️ YOLO target_lost_rate ~29% bimodal 해결** — per-step 트리거가 전부/전무로 분리(악화 0.24→0.35). obs[9-11] zeroed + `-10` 페널티. 미해결 (이번 세션 범위 밖)
- [ ] **PX4 로그 /dev/null 리다이렉트** — `/tmp/px4_{i}.log` 100+ MB 증가 방지

---

# 4. Training History

> **전체 히스토리:** `notes/experiments/training_history.md`
> **개별 실험 노트:** `notes/experiments/exp_NNN_*.md`

최근 주요 runs:

| 날짜 | Run ID | Steps | 요약 |
|------|--------|-------|------|
| 2026-07-21 | **exp019 물리 페이로드 attach/detach (검증 전용, 병행 트랙)** | 0 (8 envs hover-drop 검증, isaac-verify) | **kinematic weld 구현 + 4/4 PASS.** 부착 추적 1.1 mm, 분리 0/8 잔류, 착탄 8/8, 측정 vs 해석적 CCIP \|Δ\| mean 0.012/max 0.021 m. 보상·종단 bit-identical. Rule 24. → [[experiments/exp_019_physical_payload]] |
| 2026-07-13 | **exp015 이어학습(2차) P2/P3 ext (Isaac PPO, 병행 트랙)** | P2: +2000 iters (1098→3097) · P3: +2000 iters (3097→5096), 2048 envs, ~3h | **iter 예산 확대 검증 — 0.8m 돌파 ❌.** P2 ext: drop 2.87m(정체), release 0.33→0.01, success 0. P3 ext: drop 5.31m(회귀), reward 74.5, success 0. Rule 20f. → [[experiments/exp_015_phased_curriculum]] §8 |
| 2026-07-12 | **exp015 실학습 (baseline, Isaac PPO, tensorboard, 병행 트랙)** | 2048 envs × 600/500/500 iters (~104M steps, ~65 min) | **Phase 1→2→3 커리큘럼 첫 완주(ORCH_EXIT=0). Phase 1만 완전 수렴.** plain `--phases 1,2,3`. P1 success 0.48→1.00·reward→107(exp_014 재현); P2 reward −0.8→94.7·drop 4.66→2.91m↓·release peak 0.98/tail 0.33·success~0; P3 reward→102·lead best 0.071m·success~0. warm-start 무손실(Rule 20e). reward 우상향=proximity 지배 — P2/P3 명중 미형성(추가 학습 또는 exp_018 종단구조 필요). ckpt/로그/그래프 host 영속화. → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]] |
| 2026-03-20 | 8otphxy8 | 114K | 선형 거리 보상 + CRUISE retry. 마지막 정상 베이스라인. |
| 2026-03-22 | — | — | 보상 패치 적용 (학습 없음). Fresh start 대기 중. |
| 2026-04-16 | mtx7ud6o/x8jq9fsy/u8w3xn0w | 5500×3 | RTF 1/2/4 dry-run. RTF=2 최적 (59.5 fps). |
| 2026-06-12 | esmtny0a | 33K | Vision SAC. proximity 버그로 128ep stagnation. 폐기. |
| 2026-06-12 | 45l8vkw5 | 121K | rl_yolo. target_lost=1.0 전구간. 원인: EKF drift + FOV gap. 폐기. |
| 2026-06-13 | 7lhjy40o | 진행 중 | rl_yolo_v7_drift_guard. EKF drift guard + proximity 2.5m + penalty_lost=-0.1. |
| 2026-06-14 | k1uqgs8i | ~42K | rl_yolo_v11_cam_fix. 학습 개선(env/ep_reward 20→54, 404 successes)이나 443 CRUISE 타임아웃으로 중단. |
| 2026-06-15 | rl_yolo_v12_arm_fix | 진행 중 | Arming-rejection throughput fix (arm 게이팅 + early-bail). dry-run 0 타임아웃 검증 후 fresh 기동. |
| 2026-06-17 | 46y4xtiw | ~30K (중단) | rl_yolo_v13_terminal_reward. ~10h에 6%뿐(fps≈0.83) — arm_bail 병목 진단 위해 graceful stop. 재개 대기. |
| 2026-06-17 | xgzum51v (offline) | 1000 (dry-run) | v13_armdiag. EKF 재수렴 bimodal(0s/13–16s) 계측 → arm_bail 10→20s 수정. Rule 11. ⚠️ 이 dry-run이 YAML 중복 키로 v13 30K 체크포인트 파괴. |
| 2026-06-17 | iyhfy5ps | 진행 중 (fresh 0→500K) | rl_yolo_v13_terminal_reward **fresh 재시작** (arm_bail=20). 30K 인시던트 후. 검증: bail 0, late-EKF 14–15.5s ×3 회복. → [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] |
| 2026-06-22 | dryrun_alt10 (uqy7lmny/_gated, offline) | 1500×2 (dry-run) | **핸드오프 윈도우↑.** 고도만 10 m=실패(레버 아님; 마커 작아짐 + 200 px 필터 캡). 10 m+탐지 게이트(conf 0.5 + 200→300 px)=성공: 핸드오프 2.7→5.0 m(~2배), spurious 0. **미커밋.** → [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13 |
| 2026-06-22 | EKF A/B + softreset proto (offline) | 1500+5000 | **리셋 처리량 ~3.9×.** 원인=teleport 후 EKF 재수렴(`pre_flight_checks_pass=False`). EKF2_GPS_CHECK 0 A/B=음성(COM_ARM_WO_GPS). **soft reset(teleport 회피)=성공**: 0.93→3.61 handoffs/min, fps 2→9, reset 65s→11s, soft 100%, EKF 안정. → [[experiments/exp_009_softreset_throughput]] / Rule 14 |
| 2026-06-22 | rl_yolo_v14_softreset (byxyaf4d) | 진행 중 (fresh 0→500K) | **full run — soft reset ON 장기 검증.** 10m + 탐지게이트 + soft reset 일괄, online. EKF drift bounded?/fallback율?/실제 ETA(~15h 예상) 확인 후 커밋. → [[experiments/exp_009_softreset_throughput]] |
| 2026-06-23 | rl_yolo_v14_softreset (byxyaf4d, stop @196.5K) | 196.5K/500K(~39%), reward plateau | **plateau stop + 195K eval = 65%(13/20).** v13 80% 대비 회귀(실패 전부 final-approach stagnation). EKF 귀책 0. **Soft reset 장기검증 ✅**(3096 resets, soft ~91%, EKF bounded) → Rule 14 검증완료. 비디오 3/3 success 캡처. 회귀=미성숙(39% budget). commit 결정 대기. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]] |
| 2026-07-01 | rl_yolo_v15_bc_stable | 진행 중 (fresh 0→300K) | **wobble 교정(LPF+B+C) 적용 fresh run.** jerk RMS 2.92→1.61(−45%) A/B 확정 후 기동. → [[experiments/exp_011_wobble_lpf_reward_damping]] / Rule 15 |
| 2026-07-03 | isaac_migration_phase2 (`feat/isaac-env-migration`, 병행 트랙) | 0 (코드만) | **Isaac Lab env+PPO 이식.** v13/v15 parity, `pytest test_math.py` 29/29 통과, L4 VM 미기동. jekyun SAC 학습과 별개 브랜치/워크트리. → [[experiments/exp_012_isaac_migration_phase2]] |
| 2026-07-03 | exp013_v1_baseline (Isaac PPO, 병행 트랙, 중단 @iter 106) | ~7M steps | **비전 사멸 버그 발견·중단.** `rew_vision`≡0.0000 → `_update_vision` env-origin 프레임 혼용(2048-env grid에서 타겟 항상 프레임 밖). 수정+수치검증(visible 0%→63%). → [[errors/err_20260703_vision_env_origin_frame]] |
| 2026-07-03 | **exp013_v2_visionfix (wcjklw7a, Isaac PPO, 병행 트랙)** | 65.5M steps (1000 iters 완주) | **첫 완주 + deterministic 200-ep eval = 36%.** plateau @iter 700, d_xy_min 1.4m 정체. 실패: max_alt 33%(상승 farming, Rule 17)+crash 27%. farmer(+225)>finisher(+121) 불균형(Rule 18a), noise_std 0.8→3.92 폭주(Rule 18b). **사후 --zero-actions FAIL(11.9m): 리셋 속도킥 활성 — run 오염, max_alt 1차 용의자.** 다음=exp_014(0순위 킥 수정 → conf 거리감쇠+success 300+entropy 0, fresh). → [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]] |
| 2026-07-05 | **exp014 A2 (v3qk07pg) + A0′ (azoc1xp0), Isaac PPO, 병행 트랙** | 각 26.2M steps (400 iters) | **plant 수정 + 비전 거리감쇠 → deterministic 200-ep eval = 100.0% (202/202), d_xy_min 0.665m.** 수정: ①킥→스폰타임 MassAPI authoring(게이트 11.9m FAIL→0.2m PASS) ②로터 스핀 리셋 재주입 제거 ③**inertia 대반전**: `set_inertias`는 전파되고 있었음 — exp_013은 rate loop ~1300× 저토크 plant(Rule 19, 구 정책 plant-overfit로 무효). A2: R_alt=0.0000(climb 창발→기각 시그니처), noise_std 0.80 안정(폭주 없음). A0′ 대조: R_alt 0.0365 → 지배 요인=plant 일관성, 감쇠=꼬리 제거+YOLO parity(유지). 실 YOLO 캘리브레이션은 이미지 annotator 버그로 차단(하네스 수리 완료). reward_success·entropy 불변(다음 페이즈). → [[experiments/exp_014_A2_visionrange]] / [[research/isaac_inertia_ctrl_mismatch]] |
| 2026-07-05 | isaac_phased_curriculum (`feat/isaac-env-migration`, 병행 트랙) | 0 (코드만, 미학습) | **Phase별 순차 커리큘럼 구현.** 이미지 3단계(접근/nominal → CCIP+Residual/정지 → 이동타겟) 완전 구현: action 4→6(δ residual), phase 노브+파생 플래그, 릴리스 이벤트(nominal CCIP+δ 트리거 → 실제 DR 낙하 착탄오차 터미널 보상), drag/wind DR, Gauss-Markov 이동타겟+lead, `train.py --phases 1,2,3` 서브프로세스 오케스트레이터(6-dim 고정 → warm-start 무손실). Phase 1=exp_014 baseline 동작 동일. **`py_compile` 12파일 통과; `pytest`(+8 신규)·본 학습은 L4/컨테이너 대기.** → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20 |
| 2026-07-05 | **exp016 CCIP 릴리스 referee 재평가 (eval-only, A2 ckpt, 병행 트랙)** | 0 (200-ep deterministic eval) | **디커플링 규명: 4.59m = 지표 의미론 버그(릴리스 트리거 부재, 성공-종단 속도 캐리).** CCIP referee(≤0.2m) 수정 후: 발화 시 0.137m, release_rate 6%/11.5%(10/100Hz), aim_err_min med 0.755m ≈ d_xy_min(cross-track 지배). 구 지표 4.649m 재현. 보상/종단 bit-identical. Rule 21. → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] |
| 2026-07-06 | **exp017 Stage A — 밀집 CCIP 조준 보상 (750gpldr/6z0gpnhy/fv5qqmtz, Isaac PPO, 병행 트랙)** | 3 runs (P1 400 + v1 400 + v2 600 iters, 2048 envs) | **보상-변경-단독은 release_rate 못 올림 — 판정 (b), Rule 22.** det 200-ep: 기준선 2.5% → v1(w=1) 5.5%(aim 0.889m·speed 2.72, 방향 실재·p≈0.13) → v2(w=2/knee 1.0) 3.5% 회귀. P1 기준선 학습 내 12→3.7% 단조 하락(근접 최적화가 릴리스 능력 파괴). 원인=γ-할인 완주 보너스·CCIP 노이즈 증폭(×1.53s)·성공 조기 종단. 5-lens 사전 검증, farm 0. ckpt 3종 분리 보존(+호스트 백업) — 차기 warm-start=stageA(v1). → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] |
| 2026-07-06 | **exp018 Stage B — 릴리스-종단 이벤트 (xt0hrr1c/0ns10yso/4vaodj0o/kk06wsbx, Isaac PPO, 병행 트랙)** | 4 runs × 400 iters (전부 v1 warm-start) | **릴리스-종단 구조 → det release_rate 100.00%, drop err 0.125 m (Rule 23).** 종단 교체 단독(B0)으로 5.5%→100%(학습 내 23→99.6% 단조 상승 — Stage A 하락 반전, Rule 22a 인과 확정). aim 보상 노브 불감(w 0/1.5 100%, knee 0.75만 98.5%) — 자동 발화 referee가 노이즈를 +100 샘플러로 전환. done-flag alias 버그 사전 수정(eval success 0% 위험). 호버-드롭 수렴(종단 속도 0.11 m/s). **Stage C warm-start = exp018_B0_final.pt.** → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] |
| 2026-07-23 | **exp020 물리 페이로드 부착 첫 학습 (o5jn9xzk train / vryuc6mu eval, Isaac PPO, 병행 트랙)** | 400 iters (2048 envs, warm-start=exp018_B0, 보상 bit-match) | **물리 페이로드(kinematic weld) 학습 비용 = 0 — det 200-ep success/release 100.00%, drop err 0.169 m.** `physical_payload=True`가 유일한 델타, release_rate 첫 롤아웃부터 100%(재학습 과도기 없음). σ 드리프트 1.41→1.71 모니터 대상. `play.py --wandb` eval-figure 파이프라인 신설. 컨테이너 빈 WANDB_API_KEY 함정(`--env-file` 필수). ckpt 호스트 `/opt/drone-bombard/checkpoints/exp020/`. → [[experiments/exp_020_o5jn9xzk_payload_training]] |
| 2026-07-30 | **exp021 v19 + 이동 타겟 CV/CT/CA warm-start 학습 (a6saa42b/29jqq1lu/ntumqwoz, Isaac PPO, 병행 트랙)** | 3 runs × 1000 iters (2048 envs, warm-start=준상 v19 precise 사본) | **이동타겟 모션(CV/CT/CA)을 v19에 obs-보존 포팅 → 3종 학습 완주.** 종반 창: cv release 0.67–0.90/drop 0.39–0.90 m, ct 0.43–0.80/0.24–1.24 m, ca 0.40–0.78/0.32–1.96 m(reward 음수 잔존 — 가속 타겟 최난). 난이도 cv<ct<ca. det eval 후속. ckpt `/opt/drone-bombard/checkpoints/exp021/`. → [[experiments/exp_021_v19_moving_target]] |
| 2026-07-30 | **exp021 det 200-ep eval 3종 (1nvvuogg/prdqujah/gdow3vfg, eval)** | 0 (deterministic eval only) | **cv success 44.5%/release 81.5%/drop med 0.775 m · ct 33.8%/82.6%/0.783 m · ca 16.5%/63.5%/1.063 m.** 릴리스 이월·명중은 리드 부재로 경계 몰림(released-miss>success). 개선 1순위=리드 개입(KF obs 포팅/target-vel obs/w_lead). play.py --wandb NameError(impacted) 수정, eval seed 미고정 표본 변동(32↔44.5%) 확인. → [[experiments/exp_021_v19_moving_target]] §3b |
