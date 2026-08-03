---
date: 2026-08-03
tags: [error, isaac-lab, v16, v19, v20, payload, termination, metrics]
status: resolved
type: error
---

# 물리 페이로드 착지 래치가 "정지한 페이로드"를 영원히 놓친다

> 발견 경로: P0-2 페어드 평가 하네스가 `released=True · landed=False`로 30 s 타임아웃까지 간
> 에피소드를 세면서 드러남. 관련: [[experiments/exp_022_p0_handoff_dyn_dr]] · [[research/physical_payload_attach]]

## 증상

- v19 정책을 v19에서 평가: `release_rate=100%`인데 **timeout 17/200 (8.5%)**, 그 17개는
  투하했지만 `landed=False`.
- 새 베이스라인(argmin) 32-ep 스모크: release 75% / delivery 31% — **released-but-never-landed 14개**,
  전부 정확히 29.9 s(=타임아웃)까지 진행.

## 원인

`_step_payload_physics`의 착지 판정이

```python
newly = active & (z_local <= self.cfg.payload_ground_z)   # payload_ground_z = 0.0
```

인데, 페이로드는 **높이 0.06 m 실린더**다. 지면에 정지하면 중심 z = **half-height = 0.03 m**이고
솔버가 관통을 막으므로 `z ≤ 0.0`은 **영원히 성립하지 않는다.** 래치는 오직 한 서브스텝 안에서
평면을 뚫고 지나가는(터널링) 우연한 경우에만 발동했다 — 즉 **착지 판정이 물리 아티팩트에 의존**하고 있었다.

**계측 (`_probe_payload_landing.py`, 32 env 강제 릴리스):**

| | 수정 전 | 수정 후 |
|---|---|---|
| 착지 래치 | **0/32** | **32/32** |
| 도달 최소 z | min 0.0300 / med 0.0500 | min 0.0300 / med 0.0337 |

`min z = 0.0300`이 정확히 half-height라는 점이 결정적 증거.

> ⚠️ 진단 함정: 착지가 래치되면 그 스텝 안에서 `_reset_idx`가 `_payload_landed`를 지운다.
> 스텝 직후 라이브 버퍼를 읽으면 **항상 놓친다** — 반드시 pre-reset 스냅샷(`_last_final_snapshot`)에서
> 읽을 것 (Rule 23d와 같은 alias 함정).

## 수정

`payload_land_eps: float = 0.10` 신설, 판정을 `z_local <= payload_ground_z + payload_land_eps`로.
0.10 m는 **exp_019가 해석-측정 착탄 parity를 검증한 바로 그 값**(\|Δ\| ≤ 0.021 m) —
v16/v19 land-terminal 경로를 작성하면서 0.0으로 회귀했던 것을 되돌린 셈이다.

## 영향 (기존 수치 정정)

| | 수정 전 | 수정 후 |
|---|---|---|
| v19 정책 @ v19 (200-ep, 시드 고정) | success **91.0%**, timeout 17 | success **100.00%**, timeout 0 |

- **[[experiments/exp_022_p0_handoff_dyn_dr]] §6의 "동일 ckpt가 91% vs 준상 100%" 불일치는
  선택 편향이 아니라 이 버그였다.** 페어드 하네스 + 수정 후 정확히 100.00%로 재현된다.
- 영향 범위: **v16/v19/v20의 land-terminal 성공률은 일관되게 과소 집계**되어 있었고, 그만큼
  `timeout`이 과대 집계되었다. 학습 보상에도 영향(착지 종단 보상을 못 받은 에피소드가 존재).
- 방향은 항상 한쪽(타임아웃 → 착지)이므로 **버전 간 비교의 순서는 뒤집히지 않지만**, 절대 수치는
  재측정이 필요하다. exp_022 §3 표는 수정 후 재측정본으로 교체함.

## 교훈 (→ [[research/rl_rules]] Rule 28)

- **종단 이벤트 판정 임계값은 "정지 상태의 기하"를 포함해야 한다.** 부피가 있는 물체에 평면
  통과 테스트(`z ≤ ground`)를 쓰면 정지한 물체는 영원히 통과하지 못한다.
- **래치 실패는 실패로 보이지 않는다 — 타임아웃으로 위장한다.** `release_rate`만 보면 정상이고
  `success`만 떨어지므로 정책 문제로 오진하기 쉽다. **이벤트율과 결과율을 분리해 보고할 것**
  (`release_rate` vs `delivery_rate`, 그리고 `released_not_delivered` 카운터).
