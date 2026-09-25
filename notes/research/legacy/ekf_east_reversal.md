---
date: 2026-06-12
updated: 2026-06-14
tags: [PX4, EKF, coordinate, bug, navigation, RETRACTED]
status: retracted
type: research
---

# ⚠️ RETRACTED — "EKF East 축 반전"은 오진이었음

> **2026-06-14 정정 (GUI 육안 + 직접 측정으로 검증):**
> 이 노트의 핵심 주장 **"EKF NED East = −Gazebo_East"는 틀렸다.**
> 실제로는 **PX4 EKF East = +Gazebo_East (반전 없음, 1:1 대응)**.
>
> 2026-06-12의 진단은 검증 없이(당시 GUI 미사용) 내려졌고, 잘못된 좌표 가정 위에
> 타겟 부호를 뒤집어 **드론이 마커의 East-거울상 위치로 비행**하게 만들었다.
> 06-12 이후 모든 "성공" 로그(d_xy≈0.45m)는 거울상 위치 도달이었을 뿐 실제 마커 도달이 아니었다.
>
> 정확한 좌표 관계와 수정 내역은 **[[daily/daily_2026-06-14]]** 및 메모리 `coordinate-frames` 참조.

---

## 실제로 측정된 좌표 관계 (2026-06-14)

동시 샘플링으로 직접 측정:

| | Gazebo (ground truth) | PX4 `vehicle_local_position` (NED) |
|---|---|---|
| East | −8.0 m | `y` = −8.98 m |
| North | +7.5 m | `x` = +8.90 m |

```
PX4 NED North (x) = +Gazebo_North     (정상)
PX4 NED East  (y) = +Gazebo_East      ← 반전 없음! (06-12 가정과 반대)
env pos_enu[0] = msg.y = PX4 East = Gazebo East
env pos_enu[1] = msg.x = PX4 North = Gazebo North
```

마커 Gazebo ENU (East=11, North=10) → **PX4 NED 실측 (North=10, East=+11)**.

## 진짜 버그였던 것

East 축 부호가 시스템 전체에서 거꾸로 설정되어 있었음 (마커는 +11인데 모든 타겟이 −11):

| 위치 | 잘못된 값 (06-12) | 올바른 값 (06-14) |
|------|------------------|------------------|
| `mission_manager` `cruise_speed_y` | `+1.0` (→ −East로 순항) | `−1.0` (→ +East) |
| `mission_manager` `target_ned_y` (proximity) | `−11` | `+11` |
| `drone_drop_env` `TARGET_ENU_X` / `hyperparams target_enu_x` | `−11` | `+11` |

North(10)은 처음부터 옳았음. **East(y)만 거울 반전**.

## drone_controller y-negation의 올바른 해석

```python
# drone_controller_node.py line 72 / 77
self.target_pos = [msg.x, -msg.y, -msg.z]            # 위치
self.target_vel = [..., -msg.linear.y, ...]          # 속도
```

이것은 "EKF East 반전 보정"이 **아니다**. 단지 `/drone/cmd/*`의 y축이 "+West"
규약이라는 의미일 뿐 (PX4 East = −cmd.y). 따라서:
- Gazebo East **+11**에 도달하려면 cmd.y = **−11** 이어야 함 → `cruise_speed_y` 음수.
- RL 속도도 cmd.y가 negate되지만, 정책이 보상(타겟 +11)으로 부호를 학습하므로 코드 수정 불필요.

## 검증 (2026-06-14)

수정 후 fresh run `rl_yolo_v10_east_fix`:
- 드론 Gazebo 위치 (East=+7.9, North=+7.85) → 마커 (11,10) 방향 ✅ (GUI 육안 확인)
- env `CRUISE→TRACKING | ENU:(9.5, 9.4) | d_xy:1.6m` — 실제 마커 근처에서 d_xy 작음 ✅
- **`[YOLO] Marker visible in RL episode | d_xy:1.3m | conf:0.39`** — 드론이 실제로 마커 위에 있으니 YOLO가 탐지! ✅

> **교훈:** 좌표 진단은 반드시 Gazebo ground-truth(`gz model -m <model> -p`)와 동시
> 측정으로 검증할 것. 내부 d_xy 로그만으로는 "거울상 자기일치"에 속을 수 있다.

## 관련 노트

- [[daily/daily_2026-06-14]] — 수정 전체 기록 + 검증
- [[daily/daily_2026-06-12]] — 오진이 발생한 날 (정정 배너 추가됨)
- [[research/rl_rules]]
