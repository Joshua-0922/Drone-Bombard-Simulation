---
date: 2026-08-27
tags: [research, control, agility, envelope, ctbr, perception, analysis]
status: open
type: research
---

# 급기동(dive-bomb)은 지금 코드에서 가능한가 — 천장 4중 구조

> **답: 불가능하다.** 그리고 막고 있는 것이 하나가 아니라 **독립된 네 층**이라,
> 하나만 풀면 바로 다음 층에 걸린다.
>
> **가장 중요한 발견은 코드 밖에 있다.** 지금 `reveal_radius = 7.0 m`이
> **사용 가능한 순항 속도를 약 5.8 m/s로 못박고 있다.** 이것은 설정값이 아니라
> 기하학이다.

관련: [[experiments/exp_026_release_rate_100hz]] · [[research/research_architecture]] §10 ·
[[research/isaac_velocity_controller]] · [[research/control_smoothness_wobble]]

---

## 1. 네 층의 천장

### 층 1 — 작동 권한 (얼마나 세게 꺾을 수 있나)

| 항목 | 값 | 함의 |
|---|---|---|
| `action.vx_scale` / `vy_scale` / `vz_scale` | 4.0 / 3.0 / 3.0 m/s | **명령 가능한 최대 속도가 4 m/s** |
| `action.rate_limit` | 0.2 / 정책 스텝 | 정규화 명령 전폭 반전에 **10스텝 = 1.0 s** |
| `controller.accel_xy_clamp` | 8.0 m/s² | 0.82 g (틸트 39.2°에 해당) |
| `asset.tilt_clamp_deg` | **35°** | $g\tan 35° = 6.87$ m/s² = **0.70 g** ← 먼저 걸린다 |
| `controller.accel_z_clamp` | 4.0 m/s² | 0.41 g |
| `asset.thrust_to_weight_unloaded` | 2.0 | 기체는 $\sqrt{4-1}\,g = 17$ m/s² = **1.73 g**까지 가능 |

**기체가 낼 수 있는 횡가속도의 40%만 컨트롤러가 허용하고 있다.**
T/W 2.0에서 고도를 유지할 수 있는 최대 틸트는 $\arccos(1/2) = 60°$이므로,
`tilt_clamp_deg`를 60°까지는 물리적으로 올릴 수 있다. 그 이상은 T/W를 3.0으로
올려야 한다(레이싱 쿼드 수준).

### 층 2 — 종료 조건 (급기동하면 에피소드가 죽는다)

| 항목 | 값 | 함의 |
|---|---|---|
| `termination.limit_ang_vel` | **2.0 rad/s (115 °/s)** | ⭐ 스냅 롤 한 번이면 `bad_attitude` 종료 |
| `termination.limit_inverted_tilt` | 1.047 rad (60°) | |
| `termination.min_altitude` | 3.0 m (스텝 1부터) | 3 m 아래로 내려가는 급강하 = crash |
| `termination.v_max_safety` | 20 m/s | 안 걸림 |

`limit_ang_vel = 2.0`이 단일 최대 장애물이다. 층 1을 다 풀어도 여기서 죽는다.

### 층 3 — 릴리즈 엔벨로프 (급기동 중에는 못 던진다)

| 항목 | 값 |
|---|---|
| `release.max_speed` | 5.0 m/s |
| `release.max_vz` | 3.0 m/s |
| `release.max_tilt` | **0.35 rad (20°)** — 컨트롤러의 35°보다 더 좁다 |
| `release.max_ang_vel` | 4.0 rad/s |
| `release.alt_min` / `alt_max` | 3 / 8 m |

### 층 4 — 보상이 민첩성에 세금을 매긴다

`w_ang_vel = 0.05·|ω|²`, `w_tilt = 0.05·(roll²+pitch²)`,
`w_action_smooth = 0.05·|Δa|²`. 층 1~3을 다 풀어도 **정책이 스스로 얌전해진다.**

---

## 2. ⭐ 진짜 천장은 인지 기하다 — 속도와 `reveal_radius`의 결합

표적은 순항축에서 최대 `marker_disk_radius = 5.0 m` 벗어나 있고, 드론은
`reveal_radius = 7.0 m`에서야 표적을 본다. 발견부터 머리 위까지 $t = d/v$이고,
그 안에 횡방향 $L$을 옮기려면 $a \ge 2L/t^2$이 필요하다. 정리하면

$$d \;\ge\; v\sqrt{\frac{2L}{a_{\max}}}$$

$L = 5$ m 기준, 필요한 최소 발견 거리:

| $v$ (m/s) | 35° (6.87 m/s²) | 55° (14.0) | 60° (17.0) |
|---|---|---|---|
| 4 | 4.8 m | 3.4 m | 3.1 m |
| 6 | 7.2 m | 5.1 m | 4.6 m |
| 8 | 9.7 m | 6.8 m | 6.1 m |
| 12 | 14.5 m | 10.1 m | 9.2 m |
| 16 | 19.3 m | 13.5 m | 12.3 m |

**현재 `reveal_radius = 7.0 m` + 틸트 35° → 사용 가능한 최대 속도 5.8 m/s.**
그런데 `handoff.speed_range = (2.0, 6.0)`이다. **시나리오가 이미 기하학적 한계에
정확히 붙어 있다.** 우연이 아니라, 그 위로는 안 풀리니까 자연히 그 아래로 튜닝된 것이다.

> **결론: 속도를 올리려면 발견 거리를 같이 올려야 한다.**
> 12 m/s로 날려면 틸트 55°에서도 `reveal_radius ≈ 10 m`가 필요하다.
> 이것은 코드 제한이 아니라 운동학이므로 우회로가 없다.

---

## 3. 급기동이 논문에 좋은가 — 그렇다, 기대와 다른 이유로

직관은 "빠르면 부정확하다"이고 실제로 맞다. exp_025에서 T0 호버가 정확도 1위인 것이
그 증거다(수평 속도 0 → 탄도 산포 0). 하지만 **커지는 오차가 어느 성분인가**가 중요하다.

릴리즈 지연 불확실성 $\sigma_\tau = 0.05$ s가 만드는 축방향 오차는 $\sigma_\tau v$다.

| $v$ | 오차 |
|---|---|
| 4 m/s | 0.20 m |
| 5 m/s | 0.25 m |
| 12 m/s | **0.60 m** |
| 16 m/s | **0.80 m** |

이것은 **A그룹(모델 오차) 성분이고, 잔차가 고칠 수 있는 바로 그 성분이다.**
반면 타이밍 성분은 exp_026에서 이미 100 Hz 판정으로 제거됐다.

> **즉 민첩해질수록 오차 예산이 "잔차가 고칠 수 있는 쪽"으로 이동한다.**
> 논문의 기여를 키우는 방향이다. 이것이 급기동을 도입할 가장 강한 논거다.

### 릴리즈 기하는 다행히 깨끗하다

`_step_payload_physics`가 운반 중 페이로드를 **월드 프레임 z 오프셋**으로 매단다
(`pose[:, 2] += payload_mount_z`). 즉 드론이 아무리 기울어도 페이로드는 수직 아래에
있고, 각속도에서 오는 속도도 물려받지 않는다. 따라서 **`release.max_tilt`를 넓혀도
`_nominal_impact`의 기하가 깨지지 않는다.** (실제 하드포인트는 기체와 함께 돌지만,
플랜트와 예측기가 **일관**되다는 점이 여기서는 더 중요하다.)

---

## 4. 무엇을 바꿔야 하나 (의존 순서)

| 순서 | 항목 | 현재 → 제안 | 이유 |
|---|---|---|---|
| 1 | `termination.limit_ang_vel` | 2.0 → 6.0 rad/s | 이걸 안 풀면 나머지가 전부 무의미 |
| 1 | `termination.min_altitude` | 3.0 → 1.5 m | 급강하 인출 여유 |
| 2 | `asset.tilt_clamp_deg` | 35 → 55° | 0.70 g → **1.43 g** |
| 2 | `controller.accel_xy_clamp` | 8.0 → 14.0 m/s² | 틸트 클램프와 정합 |
| 2 | `controller.accel_z_clamp` | 4.0 → 9.8 m/s² | 자유낙하 강하 허용 |
| 3 | `action.vx_scale` / `vz_scale` | 4/3 → 12/8 m/s | 명령 자체가 못 내면 권한이 무의미 |
| 3 | `action.rate_limit` | 0.2 → 0.5 | 전폭 반전 1.0 s → 0.4 s |
| 4 | `perception.reveal_radius` | 7.0 → 12.0 m | ⭐ §2. **속도와 반드시 세트** |
| 4 | `handoff.speed_range` | (2, 6) → (6, 14) m/s | |
| 5 | `release.max_speed` / `max_vz` / `max_tilt` | 5/3/0.35 → 12/8/0.6 | §3의 기하는 안전 |
| 6 | `task_reward.w_ang_vel` / `w_tilt` / `w_action_smooth` | 릴리즈 근처에서만 부과하도록 게이팅 | 층 4 |

**보상 재스케일 주의**: 위 per-step 항들은 정책 주파수에 비례하고, `w_loiter`는
체류 스텝 수의 **2차식**이다. 주파수를 건드릴 경우
[[research/research_architecture]] §11 "순위 4의 함정 (1)"의 재스케일 표가 필요하다.

---

## 5. 진짜 dive-bomb에는 CTBR이 필요하다 — 판단 번복

2026-08-27 오전에 CTBR을 **기각**했다([[daily/daily_2026-08-27]] §6). 그 판단의
전제는 "논문의 축은 정확도이지 민첩성이 아니다"였다. 축이 민첩성으로 바뀌면
전제가 무너진다.

속도 명령 인터페이스의 대역폭은 $k_{p,vel,xy} = 1.8$ → $\tau = 0.56$ s,
−3 dB 약 **0.29 Hz**다. 여기에 `rate_limit`의 0.5 s와 LPF 75 ms가 더해진다.
**1초 안에 강하에서 인출하는 기동을 0.29 Hz 루프로는 실행할 수 없다.**

| 목표 | 필요한 것 |
|---|---|
| "빠르고 공격적" (12 m/s, 55° 뱅크, 1.4 g) | §4의 1~6번이면 충분. **CTBR 불필요** |
| 진짜 dive-bomb (하중을 걸고 인출, 준수직 진입) | **CTBR 필수** |

CTBR로 가면 `rate_limit` + LPF 평활 체인이 사라져 가우시안 탐험 노이즈의
plant-level 필터링이 없어진다 — Rule 18b의 $\sigma$ 분석이 달라지고 학습 안정성을
재튜닝해야 한다. 비용이 실재하므로, **§4를 먼저 하고 남는 제어 오차를 계측한 뒤**
CTBR을 판단하는 순서를 권한다.

---

## 6. 미결

- [ ] §4의 1~3번만 적용한 무학습 베이스라인을 12 m/s에서 측정 — 속도-정확도 곡선의 실측
- [ ] `reveal_radius` 확대가 "블라인드 순항 → 발견 → 선회" 서사를 해치지 않는지 확인
      (표적 거리도 같이 늘려야 블라인드 구간이 유지된다)
- [ ] 급기동 도입 시 heading-invariant 관측이 먼저 필요한지 — 큰 뱅크각에서
      월드 프레임 관측의 부담이 커진다([[research/research_architecture]] §5.1)
