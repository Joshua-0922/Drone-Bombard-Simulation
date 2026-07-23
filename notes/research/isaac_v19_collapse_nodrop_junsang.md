---
date: 2026-07-23
tags: [research, isaac, v19, reward, collapse, no-drop, ppo, potential-shaping]
type: research
status: active
owner: junsang
---

# Isaac v19 붕괴 진단: no-drop reward local-optimum + A/B/D 처방

> **한 줄:** v19(실제 물리 투하) 체크포인트가 수렴 후 **투하를 아예 안 하도록 붕괴**(release 0%).
> 원인 = **조준 유지에 주는 상주(standing) shaping 보상**이 노이즈 큰 물리 투하보다 이득이라
> "완벽 조준 + 영원히 호버"가 국소최적이 됨. 처방 A(포텐셜형 shaping)+B(loiter 페널티)+D(best 저장).
> 관련: [[experiments/exp_014_v19_full_integration_junsang]] · [[research/rl_rules]] · [[research/isaac_model_spec_junsang]]

---

## 증상 (eval로 확증)
라이브 시청 중 "드론이 표적서 멈추고 끝나는데 payload가 안 떨어진다" 관찰 → 헤드리스 eval:

| 체크포인트 | release | success | 종료 | aim_err_min |
|---|---|---|---|---|
| `v19_model.pt` (iter499) | **0%** | 0% | 전부 timeout | **med 0.35m** |
| `v18_phase2_model300.pt` on V19 | **98%** | 8% | success/timeout | med 0.27m |

**핵심 시그니처:** iter499는 **조준은 완벽(0.35m)한데 투하만 소실.** 비행·조준 능력은 멀쩡 →
무작위 붕괴가 아니라 "투하 회피가 이득"인 보상구조로의 **의도된(정책 최적화) 붕괴**.

## 원인: 상주(standing) CCIP 보상
`V19Env._get_rewards` (v11_env.py) 의 shaping 항 분석:

$$ r_{\text{shape}} = \underbrace{w_p (d_{xy}^{prev} - d_{xy})}_{\text{차분형: 호버 시 }0} \;+\; \underbrace{w_c\, e^{-k\, d_{impact}}}_{\text{상주형: 완벽조준 시 매 스텝 }+w_c} $$

- `progress`는 **차분형**이라 가만있으면 0 (설계 정상).
- `ccip`는 **절대값** $e^{-k d}$ 이라 $d\!\approx\!0$이면 **매 스텝 $+w_c(=0.5)$ 확정** → 200스텝 호버 시 +100 누적.
- `w_time`(-0.01/step)이 이걸 전혀 못 막음.

**비대칭:** 던지면 에피소드 종료(물리 노이즈로 착탄 1.4m, success 경계서 자주 miss, **고분산**).
안 던지면 **저분산으로 상주 보상 계속 수확** → PPO가 후자로 드리프트. 375→499 과도학습이 붕괴 완성.

## 처방 (구현 완료, 재학습 검증 대기)
**A — ccip를 포텐셜(차분)형으로** (근본, Ng et al. 1999 policy-invariant):
$$ w_c\big(e^{-k d_{impact}} - e^{-k d_{impact}^{prev}}\big) $$
조준을 **개선할 때만** 보상, 유지엔 0 → 호버 수확 인센티브 소멸. `cfg.ccip_potential_shaping=True`.

**B — 인엔벨로프 미투하 누진 페널티:** `-w_loiter * gate_steps` (gate 열린 채 안 던진 연속 스텝수).
서성일수록 비용↑ → 탈출구는 투하뿐. `cfg.v19_w_loiter=0.02`, `_gate_steps`(비엔벨로프서 0으로 self-heal).

**D — best-checkpoint 보존:** `save_interval 50→25`; `select_best_checkpoint.py`가 model_*.pt를
훑어 **실제착탄 success 기준 best**를 `model_best.pt`로 복사(Isaac 1회 부팅, 가중치만 교체).
**교훈: log_dir 절대 비우지 말 것** — iter375(착탄 0.56m) 유실이 이번 사달의 화근.

## 검증 결과 (재학습, 2026-07-23) — ✅ 성공
v18_phase2 warm-start → v19 재학습(A+B+D), iter300→600, 1024 envs. `select_best_checkpoint.py`
로 전 체크포인트 평가:

| iter | success | release | med_err(m) |
|---|---|---|---|
| 300 (시작점) | 8.7% | 98.7% | 1.04 |
| 350~450 | **0%** | 0~6% | — (재적응 딥) |
| 500 | 34% | 96% | 0.94 |
| 550 | 61% | 99% | 0.72 |
| **599 (best)** | **76.7%** | **96%** | **0.563** |

- **붕괴 방지 확정:** iter499(옛 붕괴 지점)를 넘어 **iter600까지 release 96~100% 유지**. 예전처럼
  0으로 무너지지 않음. → A(포텐셜 shaping)가 "호버 수확" 인센티브를 제거한 효과.
- **정밀도 향상:** success 0%(붕괴)/8%(v18) → **76.7%**, 착탄 0.563m(유실된 옛 best 0.56m 복원 + success 대폭↑).
- **⚠️ 재적응 딥(iter350~450):** 새 보상으로 바꾸면 옛 정책이 의존하던 상주보상이 사라져 release가
  **일시적으로 0까지 떨어졌다가 회복**. → **재적응 구간에서 조기종료 금지**, D(best저장)로 peak를 떠야 함.
  (딥에서 멈췄으면 "실패"로 오판했을 것 = D의 필요성 실증.)

## 다음
- 검증됐으니 [[research/rl_rules]] **Rule 13** 등록(상주 vs 포텐셜 shaping + 재적응 딥).
- best 모델 백업: 노트북 `~/v19_backup/v19_abd_run/model_best.pt`(=iter599), VM `~/v19_abd_backup/run/`.
- 코드: branch `Issac_JS` `2f2bf9b`. 실험: [[experiments/exp_015_v19_abd_retrain_junsang]].
- **후속(정밀도)**: 착탄 정체(0.56m)의 원인=성공존 내부 평평보상 → 연속 착지보상으로 **0.356m·success 100%** 달성. → [[experiments/exp_016_v19_precision_landing_junsang]]
- 관련: [[experiments/exp_014_v19_full_integration_junsang]] · [[research/isaac_viz_tools_junsang]]
