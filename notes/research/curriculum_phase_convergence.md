---
date: 2026-07-13
tags: [research, isaac-lab, ppo, curriculum, warm-start, release, convergence, extended-training]
status: active
type: research
---

# Phase 커리큘럼 실학습 수렴 특성 — warm-start 효과와 릴리스 갭

> exp_015 원본 커리큘럼(baseline, `--release_terminal` 미적용)을 L4에서 1→2→3 완주시켰을 때
> 관측된 **페이즈별 수렴 특성**과, 이것이 exp_016/017/018 릴리스 서사와 어떻게 맞물리는지 기록.
> 근거 실험: [[experiments/exp_015_phased_curriculum]] §7 (2026-07-12, ORCH_EXIT=0, ~65 min) +
> §8 이어학습 (2026-07-13, P2/P3 각 +2000 iters).

관련: [[research/phased_curriculum]] · [[research/ccip_release_decoupling]] · [[research/ccip_aim_reward_stageA]] · [[research/release_terminal_stageB]] · [[research/rl_rules]] (Rule 20/21/22/23)

---

## 1. 관측 (2048 envs, 600/500/500 iters, seed 42)

| | Phase 1 | Phase 2 | Phase 3 |
|---|---|---|---|
| reward (init→final) | 36.8 → **106.9** | −0.8 → **94.7** | 57.0 → **101.7** |
| success (term.) | 0.48 → **1.00** | ~0 (peak 0.14) | ~0 (peak 0.12) |
| release_rate | — | 0.33 (peak 0.98) | 0.10 (peak 0.71) |
| drop_impact_error_m | 0.13 (해석적) | **4.66 → 2.91** ↓ | 3.5 → 3.2 (노이즈) |
| lead_error_m | — | — | 0.34 (best 0.071) |

## 2. 발견

### (a) warm-start는 접근 능력을 무손실 보존하고, 페이즈 전환은 태스크 재정의로 나타난다 (Rule 20 실증)
6-dim 고정 아키텍처의 `runner.load()` 체이닝(P1→P2→P3)이 실제로 작동. **Phase 2 시작에서
reward가 −0.8로 급락**(릴리스+DR 보상 지형이 완전히 새로움)했다가 **~150 iter 내 90+로 회복** —
접근 정책이 보존됐기에 재적응이 빠르다. **Phase 3는 시작부터 reward ~57**로 출발(P2의 접근+릴리스
스킬 인계). 즉 페이즈 경계의 reward 딥→빠른 회복이 곧 "warm-start 유효 + 보상 공식 변경" 시그니처
(Rule 20b가 예측한 그대로).

### (b) reward 우상향 ≠ 임무 능력 — proximity 스트림이 리턴을 지배한다
Phase 2·3 모두 reward가 90~102로 강하게 회복하지만 **success는 ~0**. reward는 접근/proximity/dist
성분이 지배하고, 릴리스-종단 명중(real_err ≤ 0.8 m)은 그 위의 희소 이벤트다. **총 리턴만 보면
"수렴"으로 착시**되므로, 커리큘럼 2·3단계는 반드시 `release_rate` + 릴리스 실제 `drop_impact_error_m`
(+P3 `lead_error_m`)로 판정해야 한다.

### (c) 베이스라인 릴리스 메커니즘은 500 iter로 sub-0.8 m 명중을 못 만든다 (exp_016/017 재확인)
- Phase 2: release_rate가 mid-training에 0.98까지 올라 **던지는 법은 배우나**, DR 하 착탄오차가
  4.66→2.91 m로 **성공 반경 0.8 m 밖에 정체** → 명중 전환 실패. release_rate tail은 ~0.33으로
  변동(던지기와 근접-최적화가 경합).
- Phase 3: 이동타겟이 더해져 release_rate가 0.10으로 하락, lead_error tail 평탄(~0.34).
- 이는 **근접(d_xy) 보상으로 학습한 정책은 릴리스 윈도우를 거의 못 통과**(exp_016, 6–11.5%)하고
  **dense/mechanism 개입만으로는 안 뚫린다**(exp_017 판정 b)는 결론을, 이제 **커리큘럼 전체
  스케일에서** 재확인한 것.

### (d) 알려진 해법과의 접속
exp_018은 근접 종단을 **릴리스-발화 종단**으로 바꿔 release_rate 5.5%→100%를 달성했다(Rule 23a:
조기 성공 종단이 조준 구간을 잘라먹던 것이 지배 요인). **이번 exp_015 베이스라인은 그 구조를
쓰지 않았으므로** Phase 2·3의 낮은 성공은 예견된 결과다. 후속 커리큘럼 실학습은 Phase 2·3에
`release_terminal`(또는 등가의 종단 재구조)을 적용하고 warm-start 소스로 exp_018 B0를 검토할 것.

### (e) 이어학습(2차): iter 예산 확대만으로는 명중 능력 미형성 — P3는 회귀 (2026-07-13, §8)
§7 baseline 체크포인트에서 P2·P3를 각 +2000 iter 연장(P2: iter 1098→3097, P3: 3097→5096,
2048 envs, `release_terminal` 미적용). **결과(tail-mean 20):**

| | 1차 (500 it) | 2차 (+2000 it) | 판정 |
|---|---|---|---|
| P2 drop [m] | 2.91 | 2.87 (best_min 0.008) | **정체** (~0.04 m 개선) |
| P2 release_rate | 0.33 | 0.01 | **급락** (던지기 빈도 감소) |
| P2 success | ~0 | 0 | 미달 |
| P3 drop [m] | 3.20 | 5.31 | **회귀** |
| P3 reward | 101.7 | 74.5 | 하락 |
| P3 lead [m] | 0.336 (best 0.071) | 0.347 | 평탄 |
| P3 success | ~0 | 0 | 미달 |

**발견:**
- **0.8 m 돌파 없음** — 전 구간 success ≈ 0. P2 best_min 0.008 m는 스파이크이지 tail 수렴이 아님.
- P2 +2000 iter는 drop을 ~2.9 m에서 **더 못 낮춤**; 대신 release_rate가 0.33→0.01로 **근접 최적화가
  릴리스를 더 억제**(exp_017 Stage A의 단조 하락 패턴과 동형).
- P3 +2000 iter는 **악화** — 이동타겟 난이도+베이스라인 구조 한계로 추가 iter가 역효과.
- **규칙화:** 베이스라인 커리큘럼에서 P2/P3는 iter를 늘려도 sub-0.8 m 명중을 기대하지 말 것.
  구조 개입(exp_018 `release_terminal`) 없이는 P2 drop ~3 m plateau, P3는 불안정/회귀.

## 3. 규칙화 (Rule 20 실학습 corroboration)
- 커리큘럼 baseline은 **Phase 1만 완전 수렴**하고 Phase 2·3은 "던지기 학습 + 명중 미달"에 머문다 —
  reward 곡선이 아니라 이벤트 지표로 판정하라(Rule 20b).
- reward가 우상향하는데 success가 0이면 "**보상 스트림이 임무 이벤트를 지배하는가**"를 먼저 의심
  (Rule 18a farmer-vs-finisher의 커리큘럼판).

## 관련 노트
- [[experiments/exp_015_phased_curriculum]] — 본 실학습(§7) + 커리큘럼 설계
- [[research/phased_curriculum]] / [[research/rl_rules]] Rule 20
- [[research/ccip_release_decoupling]] (Rule 21) / [[research/ccip_aim_reward_stageA]] (Rule 22) / [[research/release_terminal_stageB]] (Rule 23)
- [[00_index]]
