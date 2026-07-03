---
date: 2026-06-12
tags: [experiment, SAC, YOLO, vision, proximity, EKF]
status: in_progress
type: experiment
wandb_run: esmtny0a
---

# Exp 004: rl_yolo_debug — Vision-Based TRACKING + EKF 좌표 버그 수정

## 실험 목표

YOLO 기반 CRUISE→TRACKING 전환 구현. X마커 위에서 탐지 시 RL이 시각 서보잉 학습.

## 설정

| 항목 | 값 |
|------|-----|
| WandB Run | `esmtny0a` (run name: `rl_yolo_debug`) |
| 시작 | 2026-06-12 04:28 |
| 알고리즘 | SAC, SB3 |
| 마커 위치 | Gazebo ENU (East=11, North=10) |
| `_target_ned` (수정 전) | `[10, 11]` ❌ |
| `_target_ned` (수정 후) | `[10, -11]` ✅ |
| `_stable_detect_ticks` (수정 전) | 15 |
| `_stable_detect_ticks` (수정 후) | 5 |
| YOLO conf 임계값 | 0.25 |
| 공간 필터 | pixel_dist ≤ 200px from image center |

## 주요 발견

### 1. EKF East 축 반전 (치명적 버그)

PX4 SITL Gazebo Harmonic에서 EKF의 East 축이 **-Gazebo_East**를 가리킨다. `drone_controller_node.py`가 포지션 명령의 y를 의도적으로 반전(`target_pos = [msg.x, -msg.y, -msg.z]`)하여 이를 보정하고 있다.

결과:
- 마커 Gazebo ENU (East=11, North=10) = EKF NED (North=10, East=**-11**) = NED **(10, -11)**
- 기존 `_target_ned = [10, 11]`은 Gazebo 상의 NW 방향 (마커에서 ~22m) — 절대 도달 불가
- 수정 후 `_target_ned = [10, -11]`로 프록시미티 트리거 작동 확인: **d_xy=3.9m ≤ 4m**

### 2. 실제 YOLO 탐지 conf 범위

```
실제 마커 탐지: 0.10–0.35 (드론이 마커 직상방 통과 시)
False positive: 0.10–0.63 (Gazebo 회색 지형)
```

conf만으로는 실제/FP 구분 불가. 공간 필터(pixel_dist ≤ 200px)가 핵심.

### 3. 이중 YOLO 노드 문제

- RL env가 YOLO를 인프라 프로세스로 관리 (`_infra_procs`)
- 이전 세션에서 수동으로 YOLO를 추가 기동 → 2개 실행
- 2개 퍼블리셔 → stable_detect_count가 2× 빠르게 증가 → 오탐지 TRACKING
- 수정: 수동 기동한 YOLO(PID 539114) 종료

### 4. ultralytics Boxes 필터 버그

Boolean 텐서 인덱싱 `detections[bool_tensor]`가 이 버전에서 silent fail. 정수 슬라이스 `detections[:0]`로 대체.

## 결과 (수정 전: 04:28–13:xx)

| 에피소드 | ep_rew_mean | 비고 |
|---------|-------------|------|
| 128 | -174 | 전체 stagnation-terminated (TRACKING 미도달) |

## 결과 (수정 후: 13:xx~)

- 첫 2에피소드: `PROXIMITY TRIGGER: d_xy=3.9 m ≤ 4.0 m` 확인 ✅
- RL env `TARGET_ENU_X` 버그도 발견 (11.0 → -11.0 수정) — fresh run 필요

## Exp 005: rl_yolo fresh — 2M 타임스텝 (WandB: 45l8vkw5)

| 항목 | 값 |
|------|-----|
| WandB Run | `45l8vkw5` (run name: `rl_yolo`) |
| 시작 | 2026-06-12 13:24 |
| Timesteps | 2,000,000 |
| 실행 방식 | nohup (PID 1370970, 터미널 종료해도 지속) |

### 8 에피소드 검증 (2383 timesteps)

| 항목 | 수치 | 판정 |
|------|------|------|
| TRACKING 진입 d_xy (정상) | 3.2–3.7m (7/9 에피소드) | ✅ 근접 트리거 정상 |
| TRACKING 진입 d_xy (EKF 드리프트) | 16.8m, 19.0m (2/9) | ⚠️ 기존 문제 |
| YOLO 마커 탐지 | step 186, conf=0.25, d_xy=2.1m | ✅ |
| ep_rew_mean | -151 (4ep) → -181 (8ep) | 아직 random exploration 단계 |
| fps | 2 | ⚠️ CRUISE timeout infra 재시작 지연 |

**EKF East 수정 효과 확인**: d_xy 3.2–3.7m (이전 19–22m).

## 관련 노트

- [[research/ekf_east_reversal]] — EKF East 반전 패턴 (이 세션에서 새로 발견)
- [[experiments/training_history]]

## 코드 변경

- `ros2_ws/src/mission_manager/mission_manager/mission_manager_node.py`
  - `_target_ned = [10, -11]`
  - `_stable_detect_ticks = 5`
  - 공간 필터 추가 (pixel_dist > 200px 거부)
- `ros2_ws/src/vision_detection/vision_detection/monocamera_xmarker_detector.py`
  - conf 임계값 하드 게이트 구현
  - `detections[:0]` 슬라이스 기반 필터
  - 첫 탐지 프레임 디버그 저장
