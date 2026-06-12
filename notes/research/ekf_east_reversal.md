---
date: 2026-06-12
tags: [PX4, EKF, coordinate, bug, navigation]
status: confirmed
type: research
---

# PX4 SITL Gazebo Harmonic: EKF East 축 반전

## 발견

2026-06-12, 8시간 training 실패 분석 과정에서 확인.

## 패턴

PX4 SITL + Gazebo Harmonic 조합에서:

```
EKF NED East (y축) = -Gazebo_East
EKF NED North (x축) = Gazebo_North  (정상)
```

즉 East 방향이 **180° 반전**되어 있다.

## 증거

드론이 NED (+1, +1) 방향으로 순항할 때 YOLO 로그의 Drone 위치:

```
"ENU: (-5.6, 6.3)"  → EKF: NED_East=-5.6, NED_North=6.3
"ENU: (-11.4, 11.3)" → 마커 (Gazebo ENU 11, 10) 직상방 통과 확인
"ENU: (-41.8, 41.9)" → 41.9m North, 41.8m Gazebo_East 이동
```

NED (+1,+1) 명령 → Gazebo NE 방향 이동: EKF East가 반전되어 drone_controller y-negation이 이를 보정.

## drone_controller_node.py와의 관계

```python
# line 72 — 의도적 y 반전 (EKF East 반전 보정)
self.target_pos = [msg.x, -msg.y, -msg.z]
```

- `msg.y = cruise_target_y` (EKF NED East 방향으로 증가)
- `-msg.y` → PX4에 -EKF_East = +Gazebo_East로 전달 ✓

이 반전이 있기 때문에 드론은 **올바르게 Gazebo NE 방향으로 비행**한다.

## 프록시미티 트리거 보정

마커 Gazebo ENU (East=11, North=10)의 **EKF NED 좌표**:

```
EKF_North = Gazebo_North = 10
EKF_East  = -Gazebo_East = -11
→ EKF NED (10, -11)
```

따라서 `mission_manager_node.py`의 `_target_ned = [10, -11]` (기존 `[10, 11]` ❌).

## 확인

`_target_ned = [10, -11]` 적용 후:
```
PROXIMITY TRIGGER: d_xy=3.9 m <= 4.0 m  ✅ (두 에피소드 연속)
```

## 적용 규칙

- 모든 Gazebo world 좌표 → EKF NED 변환 시 East 축에 부호 반전 적용
- Gazebo ENU (East=X, North=Y) → EKF NED (North=Y, East=-X)
- 이 보정은 drone_controller의 y-negation이 있는 한 필요

## 관련 노트

- [[experiments/exp_004_rl_yolo_debug_vision]]
- [[research/rl_rules]]
