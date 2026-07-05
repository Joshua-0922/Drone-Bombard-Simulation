================================================================================
 Issue #027 — Payload tracking 후 detach (사용자 의도 100% 구현 후보)
================================================================================
상태   : 검토 / 미시작
발견일 : 2026-06-27
파일   : issue_027_payload_tracking_after_detach.md
연관   : #026 (toss 환경 자연 해법), v9a (#028) 의 처방 2 분석


================================================================================
 1. 문제점
================================================================================

  사용자가 v9 처방 2 의 정확한 의도 표현:

  > "payload 가 처음부터 끝까지 (0,0) 에서 (4,3) 으로 가까워 지면 가산점,
  > 중간에 멀어지는 부분이 있으면 패널티"

  핵심: **payload 의 trajectory** 를 매 step 추적. drone 의 움직임 무관.

  현재 v9a 처방 2 (w_dist 1.5):
    - attached 동안 drone == payload (offset 0.14m 작음) → drone 의 d_xy reward 와 동일
    - detach 후 ep 즉시 종료 → payload 의 ballistic trajectory 추적 못 함
    - 사용자 의도 의 detach 후 부분 미반영


================================================================================
 2. 현재 시스템 분석
================================================================================

  2-1. 매 step (drop 전):
    - 정책 input: drone 의 pos, vel, ang_vel + CCIP (ballistic 예측 거리)
    - reward: r3_dist (drone 거리 감소) + r3_orient + r3_impact (CCIP)
    - **예측만** (실제 payload trajectory 측정 X)

  2-2. Drop 시점:
    - drone 이 publish_drop() → DetachableJoint detach 명령
    - drop_calculator (별도 ROS 노드) 가 Gazebo 의 payload model 의 ground contact 감지
    - 그 위치 ROS publish → env 가 받음 → drop_err 계산
    - terminal reward (precision + jackpot)
    - **ep 즉시 종료** (terminated = True)

  2-3. Drop 직후:
    - 추적 안 함
    - 이미 ep 종료
    - 다음 reset (D1 처방: _kill_infra + _start_infra 38s)


================================================================================
 3. 사용자 의도 의 검토 (재방문)
================================================================================

  사용자가 직접 짚은 분석 (정확):
  > "그런데 너의 말도 어느정도 맞는게, 이건 결국 payload 경로 분석이니까
  > drone이 x 위에서 떨구는 거나, x에 도달하기 전에 조준해서 떨구는 거나
  > reward는 동일하겠네?"

  → payload distance reward 의 본질 = drone 행동 무관, payload 의 monotonic
    distance 감소 보상. 다양한 drop strategy (hover-drop, early shot, toss)
    모두 가능.

  단, **detach 후 payload 의 ballistic trajectory 가 marker 향해 가야**
  +reward 큼. 그게 정책의 학습 핵심 신호 가능.


================================================================================
 4. 구현 옵션
================================================================================

  4-1. 옵션 A — Gazebo payload pose 직접 받기:

    Gazebo 가 모든 model 의 dynamic pose publish:
      /world/x_marker_world/dynamic_pose/info (또는 별도 topic)

    확인 필요:
      ```bash
      docker exec drone-bombard-harmonic bash -c "
      gz topic -l | grep -i payload
      "
      ```

    가능 구현:
      1. gz_ros2_bridge 에 payload_0 의 pose topic 추가 (yaml)
      2. drone_drop_env 에 ROS subscriber 추가 (drone 의 pos_enu 와 유사)
      3. step() 매 step 에 payload position 직접 측정 (attached + detached 모두)
      4. payload distance reward 매 step 적용

    코드 추가량: ~50 줄

  4-2. 옵션 C — detach 후 ep 연장:

    현재:
      ```python
      self._node.publish_drop()
      self.dropped = True
      got_result = self._drop_error_event.wait(timeout=3.0)
      ...
      terminated = True       # ep 즉시 종료
      ```

    변경:
      ```python
      self._node.publish_drop()
      self.dropped = True
      # ep 종료 안 함 — payload landing 까지 step 계속
      # 매 step 동안 payload 위치 추적 + reward
      # drop_calculator 의 actual landing 받으면 terminated = True
      ```

    drop_wait_timeout 길게 (5-10s) 또는 별도 카운터.

    코드 추가량: ~30 줄 (step/reset 로직 변경)

  4-3. 옵션 A + C 결합 = 사용자 의도 100%:
    - 매 step (attached + detached) payload 위치 측정
    - reward: r3_dist 의 payload 버전 + (detach 후 ballistic trajectory 시 매 step distance reward)
    - terminated 는 payload landing 시


================================================================================
 5. 영향 평가
================================================================================

  5-1. 학습 시간:
    - 각 ep 의 step 수 증가 (drop 후 ~50 step 더 추적)
    - 총 학습 시간 ~20% 증가

  5-2. 학습 신호:
    - dense 강화 (drop 전 + drop 후 매 step)
    - 정책이 detach 시점의 drone state (pos, vel, attitude) 의 ballistic 결과 학습
    - → CCIP 정확도 학습 강화 (w_prediction 활성과 시너지)

  5-3. 정책 행동 가능 변화:
    - early shot (조준) 가능성 증가 (drop 시점 의 drone state 가 trajectory 결정 → 학습 강화)
    - hover-drop 도 가능 (drone 이 target 위 hover → drop → free-fall → marker)
    - toss 도 유지 (payload 가 forward momentum 으로 marker 향해 → +reward)

  → 사용자 의도 (지나치는 현상 안 함) 가 자연 emerge 가능. 단, 학습이 잘 되었을 때.


================================================================================
 6. Trade-off / 위험
================================================================================

  | 항목                  | 평가                                    |
  |---------------------|----------------------------------------|
  | 코드 변경 크기         | 옵션 A + C = ~80 줄 + bridge yaml       |
  | 학습 시간 증가         | ~20%                                   |
  | 정책 행동 변화 보장     | 학습 잘 되면 yes                        |
  | 환경 안정성           | drop_calculator 와 잘 조율 필요          |
  | 백업 복잡도           | env / bridge / yaml 모두 변경 필요       |


================================================================================
 7. 추천 진행 (사용자 결정 시)
================================================================================

  Phase A — 진단 (1-2 시간):
    1. gz topic list | grep payload → 가능 topic 확인
    2. gz_ros2_bridge 추가 가능성 검토
    3. drop_calculator 의 payload position publish 가능성 검토

  Phase B — 옵션 A 구현 (1 일):
    1. bridge yaml 에 payload pose 추가
    2. drone_drop_env 에 _on_payload_pose callback
    3. payload_distance reward 매 step

  Phase C — 옵션 C 구현 (1 일):
    1. step() 의 termination 로직 변경
    2. drop 후 N step 추가 (또는 landing detection)
    3. drop_wait_timeout 조정

  Phase D — 학습 (100-200k step):
    1. fresh start (v8 prior 가 toss 강함, fine-tune 시 의미 작음)
    2. 또는 v8 warm start + 새 reward 적응

  Phase E — 평가 (1 시간):
    1. dgui 로 정책 행동 시각 확인
    2. payload trajectory plot
    3. 사용자 의도 (지나치는 현상 해결) 검증


================================================================================
 8. 결정 이력
================================================================================

  2026-06-27:
    - v9a 의 처방 2 분석 시 detach 후 payload tracking 의 필요성 발견
    - 사용자 의도 "처음부터 끝까지" 가 detach 후 trajectory 포함
    - 별도 안건으로 등록 (큰 변경이라 단계적 검토)
    - 사용자 결정 대기 (v9a 더 학습 vs 두번째 처방 vs 이 안건)


================================================================================
 9. 관련
================================================================================

  - parameter_log.md §4 #38 — v9a 처방 2 의 r3_dist 와 동일 분석
  - issue_026 — toss 환경 자연 해법 (이 안건 의 motivation)
  - issue_007 — CCIP gap (CCIP 예측 정확도와 연결)
  - design/design_review.md — payload 추적 메커니즘 후보
