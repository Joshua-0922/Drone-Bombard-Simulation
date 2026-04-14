---
date: 2026-03-19
tags: [error, gazebo, ODE, spawn, PX4]
status: resolved
type: error
---

# Err — ODE AABB 크래시 (드론 스폰 고도)

---

## 증상

- 에피소드 시작 직후 Gazebo 즉시 크래시
- Run `a9f6lk57` (자기 관리 인프라 첫 시도)
- ODE AABB (Axis-Aligned Bounding Box) integer overflow 에러

---

## 원인

`PX4_GZ_MODEL_POSE=0,0,5` 설정으로 드론이 z=5 m에서 스폰.

```
z=5 m 스폰
  ↓ 5m 자유낙하 (5초 EKF 워밍업 동안)
  ↓ ~10 m/s 속도로 지면 충돌
  ↓ 모터 스핀업 시 극단적 접촉력 + 모터 토크 동시 발생
  ↓ ODE AABB integer overflow → Gazebo 크래시
```

---

## 해결책

```python
# 이전 (잘못됨)
PX4_GZ_MODEL_POSE = f"0,{y},5"

# 수정 1차: 지면에 스폰 (z=0)
PX4_GZ_MODEL_POSE = f"0,{y},0"

# 수정 2차: 착지 기어 겹침 방지 (z=0.5)
PX4_GZ_MODEL_POSE = f"0,{y},0.5"
```

지면에서 스폰 → 자유낙하 없음 → ODE 크래시 없음.

---

## 추가 발견사항

`PX4_GZ_MODEL_NAME` (pre-spawn 모드) 사용 시에도 크래시 발생:
- 4개 모터 플러그인이 물리 스텝 1에서 동시 활성화
- `PX4_SIM_MODEL` (dynamic spawn) 방식으로 전환하여 해결

---

## 관련

- [[../research/architecture]] — Method A 스폰 설정
