---
date: 2026-10-11
tags: [experiment, sim2real, sim2sim, gazebo, px4, ros2, deployment]
status: done
type: experiment
wandb_run: (없음 — Gazebo Harmonic + PX4 v1.15.4 SITL 12 에피소드)
---

# exp_043 — Isaac 정책을 ROS 2 노드로 이식해 Gazebo + PX4 SITL에서 투하: L0 6/6 성공(중앙값 0.20 m), 장착 정상

> **한 줄.** Isaac Lab에서 학습한 L0(그리고 L0 + GRU 잔차)를 numpy ROS 2 노드(`ros2_ws/src/drop_policy`)로 옮겨, Isaac 기체·페이로드에
> 맞춘 Gazebo 모델과 **진짜 PX4 v1.15.4 SITL** 위에서 6개 시험 환경 × 2팔 = 12 에피소드를 날렸다. **전부 X 근처에 착지했고**, L0는 6/6이
> 0.5 m 이내(중앙값 0.203 m)였다. 페이로드는 정지 비행에서 정확히 기체 원점 0.140 m 아래에 붙어 있었고(비행 중 흔들림 std < 3 cm), 분리도 매번 됐다.
> GRU 잔차는 P 제어 비행으로 학습한 것이라 PX4 위에서 오히려 나빴다(중앙값 0.282 m, 맞바람 1.23 m) — exp_042·Rule 47의 재확인.

관련: [[experiments/exp_042_px4_controller_noretrain]] (Rule 47) · [[research/sim2real_controller_alignment]] · [[research/sim2real_retrain_audit]] · [[errors/err_20261011_offboard_yaw_nan]] · [[experiments/training_history]]

## 1. 무엇을 만들었나
| 층 | 내용 |
|---|---|
| 정책 이식 | `isaac_lab/_export_deploy.py`로 L0 actor(26→256→256→7, ELU)와 GRU(26→64→2)를 npz로. numpy 코어 `policy_core.py`가 관측 26채널, 행동 처리, 공칭 착탄 예측, 릴리즈 게이트를 Isaac 코드와 줄 단위로 재현. 골든 테스트 5/5(행동·GRU 1e-4, 착탄 예측 1e-3) |
| 관측 동등성 | `isaac_lab/_trace_policy_obs.py`로 같은 handoff의 Isaac 상태·관측·행동을 기록해, numpy 빌더가 같은 관측을 만들고 같은 행동을 내는 것을 확인 |
| 노드 | `drop_mission_node`: CLIMB(접근 방위로 yaw 고정) → CRUISE(handoff 상태 재현) → POLICY(10 Hz, 상태 100 Hz마다 릴리즈 판정, 판정 0.22 s 뒤 분리) → HOLD. 기존 `drone_controller`(offboard, ENU→NED, 20 Hz LPF α 0.4 = Isaac)를 그대로 사용 |
| Gazebo 모델 | `x500_drop`: Isaac 기체와 같은 질량·관성, 최대 로터 1000 → 1116 rad/s(T/W 2.0), 페이로드 장착 0.10 → 0.14 m, 카메라 제거, 기체 항력 0.06(Hydrodynamics). `payload_drop`: Ø10 × 6 cm, 0.1 kg, 항력 0.005. 월드 템플릿 `drop_world.sdf.in` |
| PX4 | airframe `4020_gz_x500_drop`: ESC 최대 1116, 호버 0.5, 기울기 35°, 하강 3 m/s(기본 1.5는 정책 명령을 자름) |
| 시험 | `run_gazebo_eval.py`: 에피소드마다 새로 띄움. DR 없음, 고정 조건 |

## 2. 결과 (12/12 착지)
| 시험 환경 | L0 착지 오차 [m] | L0+GRU 착지 오차 [m] | 투하 속도 L0 / GRU [m/s] | 투하 고도 L0 / GRU [m] | 배달 시간 L0 / GRU [s] | 장착 오프셋 z (정지 비행) | 비행 중 장착 흔들림 std 최대 |
|---|---|---|---|---|---|---|---|
| S1_straight | 0.145 | 0.203 | 3.98 / 4.03 | 3.45 / 3.37 | 5.1 / 5.0 | -0.1400 / -0.1400 | 0.027 |
| S2_left20_slow | 0.316 | 0.411 | 3.19 / 2.88 | 3.37 / 3.35 | 5.6 / 5.8 | -0.1400 / -0.1400 | 0.024 |
| S3_right25_fast | 0.202 | 0.152 | 4.00 / 4.01 | 3.48 / 3.52 | 4.9 / 4.9 | -0.1400 / -0.1400 | 0.029 |
| W1_cross3 | 0.077 | 0.159 | 3.95 / 4.02 | 3.41 / 3.42 | 5.1 / 5.0 | -0.1399 / -0.1399 | 0.027 |
| W2_head4 | 0.433 | 1.232 | 3.84 / 3.88 | 3.22 / 3.73 | 5.3 / 5.1 | -0.1398 / -0.1398 | 0.024 |
| W3_tail3_left20 | 0.204 | 0.361 | 3.27 / 3.15 | 3.51 / 3.43 | 5.5 / 5.5 | -0.1399 / -0.1399 | 0.024 |

L0: n=6, 착지 6/6, 오차 중앙값 0.203 m, 평균 0.230 m, 최대 0.433 m, 0.5 m 이내 6/6

GRU: n=6, 착지 6/6, 오차 중앙값 0.282 m, 평균 0.419 m, 최대 1.232 m, 0.5 m 이내 5/6

장착 오프셋 z는 정지 비행 중 Gazebo 참값(페이로드 − 기체 원점). 흔들림은 비행 중(CRUISE·POLICY) 같은 값의 표준편차 최대 성분.

## 3. 해석
- **배포 경로가 동작한다.** Isaac에서 학습한 그대로의 가중치가, 진짜 PX4 속도 제어기 위에서 handoff → 하강 → 시점 판정 → 투하를 해냈다. 투하 속도 2.9~4.0 m/s, 고도 3.2~3.7 m로 Isaac의 "던지기" 행동(평균 3.45 m/s)과 같다.
- **L0 정확도는 Isaac PX4 모드 평가와 같은 수준.** Gazebo 중앙값 0.203 m(n=6) 대 Isaac PX4 모드 CEP50 0.279 m(n=600, exp_042). 표본이 작아 비교는 정성적이다. 바람 4 m/s 맞바람에서도 0.43 m.
- **GRU는 PX4 위에서 해롭다(Rule 47).** 6개 중 5개에서 L0보다 나빴고 맞바람에서 1.23 m. P 제어 비행 데이터로 학습한 GRU가 PX4 PID 비행의 입력 패턴을 잘못 읽는다. 일괄 재학습(PX4 모드 덤프 → GRU 적합) 전까지 **현재 배포 최선 모델은 L0 단독**이다.
- **장착·분리 정상.** 정지 비행 −0.1400 m(설계값 −0.14), 비행 중 흔들림 std ≤ 0.029 m, 분리 후 페이로드가 매번 낙하·착지.

## 4. 이식 중 발견해 고친 것
1. **기존 `drone_controller`의 숨은 버그:** 속도 모드 `TrajectorySetpoint`가 yaw를 비워 두어(기본 0) PX4가 기수를 북쪽에 고정하고 yaw 속도 명령을 무시했다. Gazebo 시절 RL의 yaw 명령이 전부 버려지고 있었다. → [[errors/err_20261011_offboard_yaw_nan]]
2. **handoff 기수 불일치:** 위치 모드도 yaw가 NaN이라 이륙 뒤 기수가 정해지지 않았다. Isaac은 접근 방위를 바라본 채 handoff하므로 `/drone/cmd/yaw`를 추가해 고정했다. 고치기 전에는 정책이 기울기 축을 잘못 읽어 좌측으로 이탈했다.
3. **PX4 로컬 원점:** SITL에서 PX4 로컬 원점은 Gazebo 월드 원점이라 이륙 지점과 (2, 10) m 차이가 났다. 위치 명령을 측정한 PX4 − 상태 오프셋으로 변환하도록 했다(실기에서도 같은 규약).
4. **상태 출처:** PX4 SITL GPS 잡음(수평 0.2 m, 수직 0.5 m)으로 고도 추정이 0.8 m 틀렸다. 실기 RTK 대용으로 정책 상태는 Gazebo 참값을 쓰고(`state_source=gz_truth`), PX4 제어는 자기 EKF로 돈다.
5. **Gazebo 항력 모델 차이:** Hydrodynamics는 축별 u|u|, Isaac은 노름 |v|v. 40 m 낙하 시험에서 Gazebo는 축별 ODE와 1.5 cm 이내로 일치했다. 3–8 m 투하에서는 두 모형 차이가 수 cm로 추정된다(한계로 기록).

## 5. 다음
- 일괄 재학습(→ [[research/sim2real_retrain_audit]]) 뒤 같은 12 에피소드를 다시 돌려 L0·GRU를 재평가한다. 이번 결과가 그때의 기준선이다.
- 반복 수를 늘려(시험 환경당 5회 이상) Gazebo와 Isaac의 수치를 정량 비교한다.
- `state_source=px4`(EKF 상태)로도 한 번 돌려 추정 잡음의 영향을 본다.

## 6. 영상 (10-11 추가)
최고 사례(측풍 3 m/s, L0)를 카메라 두 대(측면 원경·투하점 근경)로 다시 돌려 녹화했다: 착지 오차 0.043 m(원 실행 0.077 m). `assets/exp043_best_drop_W1_cross3_L0.mp4` — 실속도 handoff→착지, 이어서 투하 순간 1/4 배속.
- 녹화 방법: `run_gazebo_eval.py --video` (gz-sensors 카메라는 이미지 토픽 구독자가 있을 때만 렌더·저장하므로 러너가 구독자를 띄운다; `__EGL_VENDOR_LIBRARY_FILENAMES=.../10_nvidia.json`).
- 렌더링 중 시뮬이 실시간의 약 1/6로 느려지자 벽시계 10 Hz 정책이 시뮬 시간으로 60 Hz가 되어 투하에 실패했다 → 미션 노드와 `drone_controller`를 `use_sim_time`(Gazebo `/clock` 브리지)으로 전환. PX4도 같은 시계에 묶여 있으므로 시뮬 속도와 무관하게 10 Hz가 유지된다. 실기에서는 벽시계 그대로.

## 7. Isaac과 같은 환경인가
물리 상수는 같다: 기체 항력 0.06 N/(m/s)², 페이로드 0.1 kg·항력 0.005, 바람은 수평 일정풍이 기체·페이로드에 상대 기류 2차 항력으로 작용, 질량·관성·T/W 2.0·장착 0.14 m. 다른 점: (1) Isaac 학습은 에피소드마다 바람·페이로드 항력(±20%)·투하 지연(σ 0.05 s)을 무작위로 뽑고 여기서는 고정값; (2) 항력이 Isaac은 노름형 |v|v, Gazebo는 축별 u|u|; (3) 제어기가 Isaac 학습은 P, 여기서는 진짜 PX4 PID + EKF; (4) 정책 상태는 Gazebo 참값(RTK 대용).
