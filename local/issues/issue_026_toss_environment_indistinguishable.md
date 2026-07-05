================================================================================
 Issue #026 — Toss 가 환경 의 자연 해법 (hover-drop vs toss 구분 못 함)
================================================================================
상태   : 방향 결정 필요 (사용자 두번째 처방 후보 검토 중)
발견일 : 2026-06-22 (v8 toss 관찰)
파일   : issue_026_toss_environment_indistinguishable.md
연관   : v8 (#036 결과), v9a (#038 결과), #027 (payload tracking 후보)


================================================================================
 1. 문제점
================================================================================

  사용자가 v8 (`phase1_redux_v8_no_invalid_penalty`, run 96bokgae) 의 정책을
  GUI 로 시각 검증.

  관찰된 정책 행동 (모든 ep 일관):
    1. drone 이 (0, 0, 0.24) spawn → ARM → TAKEOFF → cruise alt 5m
    2. 정책이 cruise 명령 (NW) 무시 + target (4, 3) NE 방향 비행
    3. drone 이 marker (4, 3) 를 **지나친** 위치 (예: (5, 4)) 까지 감
    4. 그 위치에서 멈춤 + 몸을 marker 방향으로 기울임 (pitch back)
    5. detach → payload 가 forward momentum 으로 marker 향해 toss
    6. drop_calculator 가 payload ground impact 측정 → err 1-2m

  → "toss" 전략. 사용자 의도 (hover-drop = drone 이 marker 위 정확히 도달 후 drop)
     와 다른 방향이지만 학습이 완전 수렴 + 80.6% success — 고무적 결과.


================================================================================
 2. 사용자의 두 가설
================================================================================

  사용자 정확히 분석:

  가설 1 — 탐험 부족 (local optimum):
    - v8 = 8,736 ep, 3,644 drops. 충분히 탐색
    - 그러나 toss 가 "초기 우연 발견 + 정확도 좋음" → 그 경험 위주 학습
    - SAC entropy reg 가 mode collapse 방지 하지만 강한 local optimum 형성
    - **부분 정확** — local optimum 의 강도

  가설 2 — 환경이 두 행동 구분 못 함:
    - hover-drop 과 toss 의 학습 입장 reward 거의 같음
    - 환경 자체가 두 행동 선호 안 함
    - 학습이 발견하기 쉬운 + 정확도 좋은 toss 선택
    - **가장 본질적 원인** (root cause)


================================================================================
 3. 환경 약점 분석
================================================================================

  | # | 환경 요소                          | toss 유리하게 만든 부분                          |
  |---|-----------------------------------|----------------------------------------------|
  | 1 | auto_drop_threshold = 2.0         | 정책이 explicit drop 결정 안 함. ballistic CCIP   |
  |   |                                   | < 2m 이면 자동 drop. dynamics 가 drop 시점 결정    |
  | 2 | sparse reward (terminal drop_err) | 어디서 drop 할지 명시적 안내 없음.                 |
  |   |                                   | 정책이 dynamics 안에서 최적 발견 → toss            |
  | 3 | target NE vs cruise NW            | 정책이 cruise 명령 override → forward vel 활용     |
  | 4 | detach 후 ep 즉시 종료              | payload trajectory 직접 평가 안 됨.               |
  |   |                                   | ballistic 의 randomness 가 dynamics 의 자연 해법    |

  잘못된 가설 (1차 분석 시 잘못 알린 것):
    "hover_drop_block_threshold = 0.0 이 hover-drop 차단 → toss 강제"
    → 틀림. yaml comment: "0 = 비활성 (모든 drop 허용)". hover-drop 허용됨.

  진짜 원인:
    - 위 4 가지 환경 요소의 결합
    - 학습 입장에서 "지나친 후 drop" 과 "정확히 도달 후 drop" 의 reward 차이가
      너무 작아서 더 효율적 (시간/dynamics) 인 toss 선택


================================================================================
 4. 사용자 가 제안한 두번째 처방 후보
================================================================================

  v9a (첫번째 처방, payload_dist + drop_angaccel) 이 본질 변화 안 만든 후,
  사용자가 두번째 처방 3 가지 후보 제시 (각각 위험성 자각).

  4-1. 옵션 1 — target 거리 제한:
    drone 이 target 과 어느 거리 이상으로 가까워지면 penalty
    → drone 이 target 위 까지 안 가게 강제
    → early shot (조준 후 toss) 학습 유도

    안전: ★★★ (단순 distance penalty, 명확)
    구현:
      ```python
      if d_xy < target_distance_min:
          reward -= scale * (target_distance_min - d_xy)
      ```

  4-2. 옵션 2 — 시간 패널티 강화:
    w_time 증가 → 짧은 ep 강제 → early shot 유리

    안전: ★ (사용자 자각: "기존 학습 잘 된 경우에만 사용가능. 그 외에는 발산 가능")
    구현:
      ```yaml
      reward.w_time: 0.01 → 0.05
      ```
    위험: per-step density 변경 → SAC entropy 발산 (Round 4 교훈)

  4-3. 옵션 3 — drop 좌표 강제:
    drop 시점이 사용자 정의 임의 좌표 에서 멀어질수록 penalty
    → 정책이 그 좌표에서 drop 학습 + 그 좌표에 맞춘 속도 조절

    안전: ★★ (강력 효과, 좌표 결정이 미묘)
    구현:
      ```python
      if drop_triggered:
          reward -= scale * ||drone_xy - desired_drop_xy||
      ```
    위험: 좌표 결정이 학습 prior 강제, 정해진 좌표 외 학습 불가


================================================================================
 5. 다른 가능 처방 (v9b, v9c, ...)
================================================================================

  | # | 처방                                            | 효과                                  |
  |---|------------------------------------------------|---------------------------------------|
  | D | spawn / target randomization                   | 정책 generalize                       |
  | E | cruise 명령 비활성                              | 정책 free flight (NW prior 제거)       |
  | F | w_prediction 활성 (현재 0)                       | drop 시점 CCIP 정확도 학습 강화         |
  | G | auto_drop_threshold 좁힘 (2.0 → 0.5)            | 더 정확한 drop 위치 학습                |
  | H | auto_drop 비활성 + 정책 explicit drop action     | 정책 self-deciding (어려움)            |
  | I | Gazebo payload pose 받기 + ep 연장 (Issue #027) | 실제 payload trajectory 학습 100%      |
  | J | Fresh start with strong shaping                | v8 prior 제거, 새로 학습               |


================================================================================
 6. 결정 이력
================================================================================

  2026-06-22:
    - 사용자 GUI 관찰 시 toss 전략 발견
    - 사용자 두 가설 제기 (정확)
    - v9 처방 결정 (drop_angaccel + w_dist) — 첫번째 시도

  2026-06-27:
    - v9a 결과 분석 — 본질 변화 안 됨
    - 사용자 두번째 처방 후보 3 가지 (1, 2, 3) 제기
    - 안전 순위: 1 > 3 > 2
    - 사용자: "별개로 새 모델 생각" — 두번째 처방 진행 결정 대기


================================================================================
 7. 추천 진행
================================================================================

  추천 안건 순서:
    1. **v9a 100k 까지 학습** (의도 step 충족) — 본질 변화 확인 (#025 참고)
    2. **두번째 처방 옵션 1** (target 거리 제한, 가장 안전) — 효과 명확
    3. **Issue #027** (payload tracking 100%) — 큰 변경 but 사용자 의도 정확
    4. 환경 변경 (cruise 비활성, target randomize) — 정책 generalize
    5. **fresh start** — v8 prior 제거 (마지막 옵션)


================================================================================
 8. 관련
================================================================================

  - parameter_log.md §4 #36 (v8 결과 + 정책 행동 분석), #38 (v9a 결과)
  - issue_025 — fine-tune step 부족
  - issue_027 — payload tracking 후 detach (사용자 의도 100% 구현)
  - meeting_notes_2026-06-22.txt — 토스 관찰 + v9 처방 결정
