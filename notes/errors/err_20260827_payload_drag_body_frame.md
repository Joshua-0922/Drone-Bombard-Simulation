---
date: 2026-08-27
tags: [error, physx, isaac-lab, payload, drag, frame, resolved]
status: resolved
type: error
---

# err_20260827 — 페이로드 항력을 월드 프레임으로 계산해놓고 링크 프레임으로 전달

관련: [[research/t3_oracle_entrainment]] · [[research/research_architecture]] §2 B2 ·
[[research/rl_rules]] (Rule 31)

---

## 증상

**런타임 에러가 없었다.** 바람이 페이로드를 미는 방향이 드론 자세만큼 회전되어
있었을 뿐이라 시뮬레이션은 정상적으로 돌았고, 학습도 수렴했다. 발견 경로는 실패가
아니라 **T3 오라클을 실제 낙하와 대조하는 과정**이었다.

## 원인

`isaac_lab/drone_bombard/drone_bombard_env.py`

```python
v_air = torch.stack([self._wind_xy[:, 0] - v[:, 0],       # 월드 프레임 바람
                     self._wind_xy[:, 1] - v[:, 1],       # 월드 프레임 페이로드 속도
                     -v[:, 2]], dim=-1)
f = (...) * speed * v_air                                 # -> 월드 프레임 힘
...
self._payload.set_external_force_and_torque(forces, torch.zeros_like(forces))
#                                                    ^ is_global 기본값 False
```

Isaac Lab 2.3.2의 `RigidObject.set_external_force_and_torque`는
`is_global: bool = False`가 기본값이고, docstring에 *"External forces in bodies'
local frame"*이라고 명시되어 있다. 즉 **월드 프레임 벡터를 링크 프레임으로 해석**했다.

페이로드는 분리 직전까지 드론 자세를 물려받으므로(kinematic weld), 항력이 드론의
roll/pitch만큼(최대 35°) 회전된 방향으로 걸렸다.

**같은 파일 안에 올바른 선례가 있었다.** 기체 쪽 바람 항력은 월드 프레임으로 계산한 뒤
`quat_apply_inverse_pure`로 body 프레임에 명시 변환한다(`:1112`). 규약을 알고
있었는데 페이로드에서만 누락했다.

## 해결

```python
self._payload.set_external_force_and_torque(forces, torch.zeros_like(forces), is_global=True)
```

주석으로 "이 힘은 월드 프레임에서 조립되었고 API 기본값이 링크 프레임"임을 남겼다.

## 파급

이 버그는 **바람이 실제로 페이로드를 어느 방향으로 미는지**를 바꾸므로,
2026-08-26 이전의 모든 착탄 데이터가 영향을 받는다. 다만 학습 산출물은
[[research/research_architecture]] §0에서 이미 폐기가 결정되어 있었다.

수정 후 무학습 베이스라인 재측정(n=64)에서 T0 hover가 91.5% → 68.8%로 내려갔다.
회전된 힘이 부분적으로 상쇄되던 것이 정상화되면서 바람이 일관되게 페이로드를 밀게
되었기 때문으로 보인다(단 n=64 예비 측정이므로 n=200 정식 재측정 필요).

## 함정 메모

1. **`is_global`은 `RigidObject`와 `permanent_wrench_composer` 양쪽에 있고 둘 다
   기본값이 False다.** 힘을 월드 프레임에서 조립했다면 반드시 명시해야 한다.
2. `set_external_force_and_torque`는 deprecated 경고를 띄우지만(5초마다 throttle)
   `permanent_wrench_composer.set_forces_and_torques`도 같은 인자를 받는다.
   이번 수정에서는 최소 변경으로 기존 API를 유지했다.
3. **기체 쪽 코드가 올바른 선례였다.** 같은 파일 안에서 두 경로가 다른 규약을 쓰고
   있었으므로, 규약을 아는 것만으로는 부족하고 **한 곳에 묶어야** 한다.

## 후속

- [ ] T0~T3 n=200 정식 재측정 (이번 수정 + 오라클 수정 반영)
- [ ] 액추에이터/페이로드에 외력을 넣는 모든 지점에 프레임 규약 점검
