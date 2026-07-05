# Issue #029 — Curriculum stage 조건이 정책 시작 위치를 반영 안 함

작성: 2026-07-05
상태: 🔴 진행 중 (근본 원인 확정. Stage 재설계 안 대기)
관련:
- [../design/design_review_2026-07-05.md](../design/design_review_2026-07-05.md) — 이번 주 종합 + 재설계 안
- [issue_028_rad_ekf_stale_after_reset.md](issue_028_rad_ekf_stale_after_reset.md) — 이전 진단 (정정 필요)
- [../meeting_notes/meeting_notes_2026-07-05.txt](../meeting_notes/meeting_notes_2026-07-05.txt)

---

## 1. 증상

RAD v1 Phase 1 학습 v4/v5/v6 반복 시행에서 stage3 (design radius 4.53m) 을 정책이 절대 통과하지 못함.

### v6 관찰 (결정적)
- Stage3 두 번 도전, 두 번 다 window success 0%
- Stage1/2 는 매우 쉽게 통과 (95%+)
- Stage1↔Stage3 사이 성공률 급락 (95% → 0%)

---

## 2. Fact 확인 — Initial position 실측

### 도구
v6 에서 신규 도입한 `info['initial_target_dist_3d/xy']`, `initial_pos_{x,y,z}`, `initial_speed_xy` log.

### 실측값 (100 ep 통계, stage1_intro 시)
- `initial_target_dist_xy_mean` = **5.10m** (min 5.00 ~ max 5.23)
- `initial_target_dist_3d_mean` = 6.39m
- `initial_pos_x_mean` = -0.157m
- `initial_pos_y_mean` = 0.049m
- `initial_pos_z_mean` = 0.15m
- `initial_speed_xy_mean` = 0.031 m/s

### 해석
- Drone spawn (0, 0, 0), Target (4, 3, 0). 직선 거리 √25 = 5.0m
- Cruise 로 이동 0.15m (매우 짧음)
- **정책 시작 시점 target 거리는 항상 5m 근처**

---

## 3. 진짜 원인 — Curriculum stage 설계 결함

### Stage 조건 vs 시작 거리 매칭 (v6 curriculum)

| Stage | Radius | switch_d² | Spawn d²≈25 | 학습 유도? |
|---|---|---|---|---|
| stage1_intro | 6.0m | 36.0 | 안 (여유 11) | ✗ trivial |
| stage2_close | 5.5m | 30.25 | 안 (여유 5.25) | ✗ trivial |
| **stage3_target** | **4.53m** | **20.5** | **밖 (부족 4.5)** | **✓ 유일 학습 stage** |

### 결과

- Stage1/2 는 정책이 아무것도 안 해도 성공 (spawn 이 이미 sphere 안)
- 정책은 stage1/2 에서 이동 학습을 배우지 않음
- Stage3 에서 처음 이동 학습 필요 → 능력 부재 → 완전 실패
- **실제 학습해야 할 이동거리 = 5.0m - 4.53m = 0.47m** (매우 짧음)

---

## 4. 왜 이걸 몰랐나 (이전 진단 궤적 반성)

### 여러 오진단
1. "spawn 위치가 sphere 안" (이론 계산, fact 확인 X)
2. "spawn_yaw random 으로 시작 방향 다양, 거리 2.5~5.6m" (이론 오추정)
3. "cruise 안 됨. drone 이 spawn 그대로" (v6 log 관찰, 표현 오류)
4. "cruise 는 정상, 이동 거리만 짧음" (log 재확인)
5. "target_speed 상승" 제안 → 사용자 지적: v1~v5 도 동일. 다른 원인.
6. **"curriculum stage 설계 결함"** (최종 확정)

### 교훈
- Fact 확인 없이 이론 계산만으로 진단 → 오추정 반복
- Initial position log 도입 (v6) 후 즉시 확정
- **모든 debug 시 실측 도구 우선 원칙** 확립 필요

---

## 5. 해결안 — Stage 재설계 (v7)

### 새 5-stage 안 (spawn d²≈25 반영)

| Stage | switch_d² | Radius | active_conditions | z_min | max_dist | limit_tilt |
|---|---|---|---|---|---|---|
| stage1_intro | 23.04 | 4.80m | [] | 0.05 | 40 | 1.2 |
| stage2_close | 21.62 | 4.65m | [] | 0.15 | 30 | 0.9 |
| stage3_target | 20.5 | 4.53m | [] | 0.25 | 25 | 0.5 |
| stage4_partial | 20.5 | 4.53m | [C1, C4, C5] | 0.35 | 20 | 0.30 |
| stage5_full | 20.5 | 4.53m | [C1~C7] | 0.5 | 15 | 0.26 |

### 원리
- Stage1 부터 spawn 밖 (d²=23 < spawn d²=25) → 정책이 이동 학습 필요
- 매 stage radius 갭 0.15m (매우 완만)
- Stage3 까지는 sphere entry 자체만 (조건 없음)
- Stage4, 5 에서 조건 추가

### 예상 학습 궤적
1. Stage1 (4.80m): target 방향 0.2m 이동 학습
2. Stage2 (4.65m): 0.35m 이동
3. Stage3 (4.53m): 0.47m 이동 (design 목표)
4. Stage4: 부가 조건 3개
5. Stage5: 부가 조건 7개

---

## 6. 관련 issue

- Issue #028 (진단 방향 정정 필요): 원래 "PX4 EKF stale" 라 판단. 실제로는 cruise 정상 작동, 이동 짧을 뿐. Root cause 는 curriculum 설계.

---

## 7. 체크리스트

- [ ] v6 학습 종료 결정
- [ ] Curriculum yaml stage 재설계 반영
- [ ] v7 학습 시작
- [ ] Stage 1 진입 후 정책이 이동 학습하는지 확인 (initial log)
- [ ] Stage 2 → 3 → 4 → 5 순차 진행 여부 관찰
- [ ] 성공 시 Phase 2 (Drop) 학습 준비
