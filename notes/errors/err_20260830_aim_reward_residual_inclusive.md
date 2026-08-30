---
date: 2026-08-30
tags: [error, reward-hacking, residual, l1, open]
status: open
type: error
---

# task_env의 dense aim 보상이 잔차 포함 오차를 먹고 있다

> **L0에는 영향 없다** (`residual.enabled=False`라 잔차가 0). **L1/L2를 돌리기 전에 고쳐야 한다.**

관련: [[research/residual_ceiling]] · [[experiments/exp_018_release_terminal]] ·
[[research/rl_rules]]

---

## 증상 (아직 발생 전, 코드 감사로 발견)

`task_env._get_rewards`의 `rew_aim_pot`이 **잔차가 더해진** 탄착 오차를 먹는다.

```
_get_dones:1919-920   _, d_impact, _ = self._ccip(pos, vel)   # 잔차 포함
                      self._d_impact = d_impact
_get_rewards:1013     d_impact = self._d_impact
        :1052-1054    aim = w_ccip * (exp(-k*d_impact) - exp(-k*d_impact_prev))
```

`_ccip`은 `rescfg.enabled`면 `apply_ccip_residual(impact, self._residual_action, scale)`을
적용한다. 즉 **정책이 잔차를 표적 쪽으로 내밀기만 하면 비행이 하나도 나아지지 않아도
조준 보상을 받는다.** 잔차는 정책의 자유 출력이므로 이것은 순수한 보상 해킹 통로다.

## 이미 같은 결함을 한 번 막았다 — base env에는 가드가 있다

`drone_bombard_env._evaluate_release`에 exp_018의 명시적 가드가 있다:

> *"DELIBERATE (reward-hacking guard, exp_018): the dense aim reward
> (`_aim_err_last` -> `rew_aim`) is fed the NOMINAL-only error — never the
> residual-corrected one. ... The release TRIGGER (fire) stays residual-inclusive
> by design — biasing the release decision is the residual's job."*

**`task_env`는 이 가드 없이 다시 쓰였다.** 2026-08-27 환경 재구축 때 보상 경로가
새로 작성되면서 빠졌다.

## 크기

`potential_shaping = True`라 항이 telescoping되어 에피소드 합은
$w_{ccip}\,(e^{-k d_{final}} - e^{-k d_0})$ 이고 **상한이 +30**(자연 단위, `reward_scale` 0.1 → 3.0).
`reward_success` 300 / `success_bonus` 100과 비교하면 ~10% 수준이다.

**파국은 아니지만 정확히 틀린 것을 지불한다** — 그리고 `w_ccip = 30.0`은 현재 가장 큰
shaping 가중치다. 08-29 L1 파일럿이 0.9%로 무너진 원인 후보 목록에 이것을 추가한다
(당시엔 `residual.scale = 2.0`만 의심했다).

## 처방

base env와 같게 만든다 — **dense 조준 보상은 nominal-only 오차로, 게이트와 교차
솔버는 잔차 포함 그대로.**

```python
# _get_dones 에서 잔차 없는 예측을 따로 캐시
self._d_impact_nominal = |nominal_impact - perceived_target|
# _get_rewards 는 그것을 쓴다
```

`_resolve_release`의 `crossed` 판정과 `release_gate`는 **건드리지 않는다** —
릴리즈 결정을 편향시키는 것이 잔차의 일이고, 그 결과는 실제 착지 오차로 채점된다.

## 왜 이 결함이 설계 논거를 하나 더 만드는가

이 통로가 존재하는 이유는 **잔차를 RL 보상으로 학습시키기 때문**이다.
잔차를 **지도학습**(목표 = 실제 착탄점 − 공칭 예측)으로 두면 이 해킹 통로 자체가
사라진다 — 잔차는 보상을 전혀 보지 않는다.
→ [[research/residual_ceiling]] §8
