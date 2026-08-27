---
date: 2026-08-27
tags: [error, reward, rl, termination, dry-run, resolved]
status: resolved
type: error
---

# err_20260827 — 무료 탈출구: 벌하지 않는 종료 조건 + 탐지 뒤에 갇힌 접근 보상

> **dry-run이 잡았다.** 학습을 며칠 돌린 뒤였다면 "왜 안 배우지"로 몇 날을 태웠을 결함
> 두 개다. 프로젝트 규칙의 dry-run 의무가 정확히 이걸 위한 것.

관련: [[research/reward_operating_point]] · [[research/research_architecture]] §11.1 ·
[[research/rl_rules]] Rule 33

---

## 증상

DR_SCALE 1.5, 512 envs, 40 iteration dry-run:

| it | reward | eplen | bad_attitude | rel% | **rew_progress** | rew_time | **rew_failure** |
|---|---|---|---|---|---|---|---|
| 0 | −29.6 | 13.0 | 0.33 | 0.0 | **0.0** | −31.0 | −20.0 |
| 4 | −179.8 | 117.3 | **1.00** | 0.0 | **0.0** | −160.0 | **0.0** |
| 8 | −240.4 | 182.4 | **1.00** | 0.0 | 9.0 | −275.0 | **0.0** |
| 13 | −256.2 | 201.3 | 0.50 | 0.0 | 0.0 | — | — |

**평균 보상이 단조로 악화한다** (−29.6 → −300). 에피소드는 길어지는데(13 → 229 스텝)
보상은 나빠진다.

## 원인 1 — `bad_attitude`가 무료 탈출구였다

`_get_dones`는 다섯 가지로 에피소드를 실패 종료시킨다:

```python
failure = (crash | overspeed | bad_attitude | out_of_range | max_alt) & alive
```

그런데 보상은 **세 가지만** 벌하고 있었다:

```python
failure_t = (f["crash"] * crash_penalty
             + f["out_of_range"] * out_of_range_penalty
             + (f["timeout"] & ~released) * no_drop_penalty)
#  bad_attitude / overspeed / max_altitude -> 벌점 없음
```

`w_time`을 0.01 → 1.0으로 올린 순간 이 구멍이 치명적이 됐다.
**자세를 무너뜨리면 남은 스텝마다 1.0씩 아끼면서 벌점은 0**이다.
정책은 정확히 그걸 배웠다 — `bad_attitude`가 종료의 100%, `rew_failure`는 정확히 0.0.

> **이것은 `w_time` 인상이 만든 결함이 아니라, `w_time` 인상이 드러낸 결함이다.**
> 구멍은 처음부터 있었고 시간이 공짜일 때는 악용할 가치가 없었을 뿐이다.

## 원인 2 — 접근 보상이 탐지 뒤에 갇혀 있었다

```python
r = r + det * flying * (progress + aim)
```

`det`는 `d_xy <= reveal_radius(7.0)`이다. 그런데 **탐지하려면 접근해야 하고,
접근 보상은 탐지해야 켜진다.** 닭-달걀이다.

실측: `rew_progress`가 전 구간 **정확히 0.0**, `d_xy_min`은 13~18 m를 오간다.
정책이 손에 쥔 신호는 per-step 비용뿐이었고, 그중 방향 정보를 담은 항은 하나도 없었다.

## 해결

**① 실패 종단을 합집합 하나로 묶는다.** 원인별 합이 아니라 마스크 하나.

```python
any_failure = (f["crash"] | f["overspeed"] | f["bad_attitude"]
               | f["out_of_range"] | f["max_altitude"])
failure_t = (any_failure.float() * rw.failure_penalty          # -200, 단일 노브
             + (f["timeout"] & ~self._released).float() * rw.no_drop_penalty)
```

`crash_penalty`/`out_of_range_penalty` 두 노브를 **`failure_penalty` 하나로 대체**.
노브가 하나면 같은 구멍이 다시 생길 수 없다.

**② 접근 보상의 탐지 게이트를 제거한다.**

```python
prog_t = flying * progress          # det 곱하지 않음
aim_t  = det * flying * aim         # 조준 오차는 탐지 전에 무의미하므로 유지
```

미관측 표적까지의 참 거리를 **보상**에 쓰는 것은 정당하다 — 셰이핑은 특권 정보를
써도 되고, 정직해야 하는 것은 **관측**이며 관측의 탐지 마스크는 그대로다.
정책이 여기서 배우는 것은 "앞으로 순항하라"이고, 그것이 의도한 블라인드 순항 행동이다.

## 함정 메모

1. **종료 조건 목록과 벌점 목록은 반드시 같은 곳에서 파생시켜라.** 두 곳에 따로 적으면
   반드시 어긋난다. 합집합 마스크 하나가 정답.
2. **per-step 비용을 올릴 때는 모든 종료 경로의 순가치를 다시 계산하라.**
   불변식: *어떤 실패 종단도 최악의 성공 완수보다 나빠야 한다.*
3. **탐지·가시성 게이트를 셰이핑에 걸 때는 부트스트랩 순환을 확인하라.**
   "A를 얻으려면 B가 필요한데 B의 보상이 A 뒤에 있다"면 학습이 시작되지 않는다.
4. **per-component 보상 로깅이 없으면 이 둘 다 안 보인다.** 재구축한 task env는
   base env의 `_episode_sums`를 채우지 않아 `Episode_Reward/*`가 전부 0이었다.
   11개 항목을 등록한 뒤에야 `rew_progress = 0.0`, `rew_failure = 0.0`이 눈에 보였다.
