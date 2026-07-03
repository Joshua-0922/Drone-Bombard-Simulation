---
date: 2026-06-22
tags: [research, vision, detection, handoff, altitude, yolo, mission-manager]
status: active
type: research
---

# 핸드오프 윈도우의 진짜 레버 = 탐지 게이트 (고도 아님)

> **한 줄:** "마커가 늦게 탐지돼 RL 윈도우가 짧다"의 원인은 **순항 고도가 아니라** `vision_callback`의
> 200 px 공간 필터 + confidence 게이트 부재. 고도↑는 마커를 더 작게 만들어 오히려 역효과.
> 검증: [[experiments/exp_008_dryrun_alt10_handoff_window]] · 규칙: [[research/rl_rules]] Rule 13

---

## 문제

`mission_manager` FSM: TAKEOFF → CRUISE → **TRACKING(RL 인계)**. CRUISE→TRACKING는 YOLO가 마커에
stable lock(5 tick) 할 때 발동. v13 정하방 카메라 + 200 px 공간 필터에서 핸드오프 d_xy ≈ 3.2–3.8 m
(거의 머리 위) → RL이 제어하는 수평 거리가 짧음 = **짧은 학습 윈도우**.

## 시도한 가설: 고도↑ (5→10 m)

기하적 직관: 고도↑ → 카메라 footprint cone 넓어짐 → 마커가 더 먼 수평거리에서 FoV 진입 → 일찍 탐지.
**dry-run에서 반증됨** ([[experiments/exp_008_dryrun_alt10_handoff_window]] v1):

1. **마커 apparent size ∝ 1/고도.** 10 m에서 마커는 5 m의 절반 크기 → YOLO가 더 *가까워야* lock →
   clean 핸드오프 여전히 d_xy 2.7 m(베이스라인과 동일). **탐지가 일찍이 아니라 동등/늦음.**
2. **200 px 공간 필터가 진짜 캡.** 고도와 무관하게 "중심 근처" 탐지만 accept → 핸드오프를 항상
   "거의 머리 위"로 클립.
3. **부작용:** 넓어진 FoV가 순항 *시작*에서 X-like 지면 FP를 잡음 → spurious CRUISE→TRACKING
   (conf=0.00, d_xy≈11 m) → health gate 발동 → restart 루프.

→ **고도는 레버가 아니다.** (오히려 탐지 신뢰성을 떨어뜨림.)

## 진짜 원인 & 수정

`vision_callback`이 (1) `z>0`면 confidence 무관하게 accept, (2) 200 px 안이면 accept 였음.
- **confidence 게이트 추가** (`min_detection_conf=0.5`): real 마커는 conf 0.73–0.95, 지면 FP는 ≤0.45.
  0.5 게이트가 둘 사이 간격에 안착 → 순항-시작 FP 차단(spurious 0).
- **공간 필터 완화** (200→300 px, `detection_pixel_radius`): real off-center 탐지(264–293 px)를
  더 일찍 accept → 핸드오프 d_xy 2.7→**5.0 m**(윈도우 ~2배). conf 게이트가 넓힌 영역을 지켜주므로 안전.

**결과(v2):** 핸드오프 5.0–5.2 m, spurious 0, EKF-drift 0, real conf 0.93.

## 적용 규칙

- **"늦게 탐지" 문제는 탐지 파이프라인(필터 반경·conf·마커 가시성)에서 풀어라. 고도/비행 기하는 레버가 아니다.**
- **공간 필터를 완화하려면 반드시 confidence 게이트와 함께.** 반경만 넓히면 off-center FP 표면이 커진다.
- **conf 게이트 임계는 데이터로.** real vs FP conf 분포를 로깅(accept/reject)해 그 사이 간격에 임계를 놓아라.
- **고도는 마커 가시성을 깎는다.** apparent size ∝ 1/고도 → YOLO lock 거리 단축. 윈도우 확장엔 역효과.

## 향후 조건
- `detection_pixel_radius` 더 키우면(>~360) 프레임 가장자리(코너 ~400 px) 근접 → edge 불안정. 300이 sweet spot.
- 마커를 물리적으로 키우거나 YOLO를 원거리/소형에 robust하게 하면 *그때* 고도↑가 윈도우 확장에 기여 가능.

## 역링크
- [[experiments/exp_008_dryrun_alt10_handoff_window]] — 검증 dry-run (v1/v2)
- [[research/terminal_overshoot_trap]] — 핸드오프 거리 ↔ 종단 보상 트랩 (Rule 10, 반대 방향 트레이드오프)
- [[research/rl_rules]] Rule 13
