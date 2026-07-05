---
date: 2026-06-05
tags: [research, phase1, summary, roadmap, sac, drone-drop]
status: active
type: research
owner: junsang
---

# Phase 1 종합 정리 — Round 1 ~ 7 v3, 그리고 redux

> Phase 1 전체의 흐름·핵심 발견·전환 이유를 한 장으로. 라운드별 상세는 `local/design/model_history.md`.
> **출처:** `local/design/model_history.md`, `local/issues/master.txt`, `local/meeting_notes/`(05-26~06-05)

---

## Phase 1이란

X마커(target)에 드론이 자율 접근해 폭탄을 **정밀 투하**하는 정책을 SAC로 학습.
Round 1(첫 베이스라인)부터 Round 7 v3(685k 자연 종료)까지가 Phase 1, 그 후 **redux**(task 재설계)로 이어짐.

- **목표:** auto_drop으로 target에 정밀 명중 (success < 5m → 후에 < 1m)
- **스택:** SAC(SB3 기반 DampedEntropySAC), PER, CCIP 기반 d_impact, Gazebo+PX4 SITL

---

## 라운드 한눈에 (시간순)

| Round | Run | Steps | best/success | 핵심 |
|-------|-----|-------|--------------|------|
| 1 | ruozrv5x | 150K | 4.64m / 1건 | 첫 베이스라인. signal 너무 sparse |
| 2 | z05fx7g9 | 150K | 2.53m / 16건 | gradient 완만화. post-success regression 발견 |
| 3 | lidq3ydu | 157K | 4.32m / 8건 | PER+안전망 도입. #021 첫 발생(당시 미진단) |
| 4 | 4j46qwpk | 146K | — | hover 차단(per-step) → **ent_coef 6.03 발산** |
| 5 | mnlr1zpe | 156K | — | terminal 처방도 발산 → **#019 본질적 발산 확정** |
| 6 v1/v2 | 6b8bslmz | 294K | 4.36m / 4건 | DampedEntropySAC. cap 갇혀 발산 |
| 7 1차~v2 | iobwvcrm 외 | ~385K | 1.85m / 12건 | **target_entropy −15** 근본 처방. critic 폭주 발견 |
| **7 v3** | 436xl0bb | **685K** | **1.32m / 16건** | critic 안정화. **Phase 1 endpoint** |
| redux v1 | ayi27a56 | 89K | **0.809m / 96.3%** | target(4,3)+random_drop=0 |
| redux v2 | za9zxdh6 | ~390K 🔄 | TBD | curriculum 정밀화 |

---

## 핵심 발견의 시간순 (무엇을 배웠나)

1. **Round 2 — post-success regression:** success 직후 정책이 발산 → PER+LR+tau로 완화 (Round 3).
2. **Round 4·5 — SAC 발산은 처방 형태 무관:** per-step이든 terminal이든 ent_coef 6.0+ 발산 → **#019(본질적 발산)** 확정.
3. **Round 6→7 — 근본 원인 규명:** bounded action(tanh) + default target_entropy(−5)가 원인 → **−15로 처방**. → [[research/sac_bounded_action_target_entropy_junsang]]
4. **Round 7 v2 — critic 폭주가 2차 트리거:** target_entropy만으론 부족, critic_loss 폭주가 ent 발산 유발 → Huber + target_q_clip=500 + per-sample damping (Round 7 v3).
5. **#021 — 인프라 오진단 정정:** Round 3·6의 "PX4 20GB"·"OOM"이 실은 동일 gz timeout이었음 → [[errors/err_20260528_gz_timeout_recurrence_junsang]]
6. **Phase 1 eval — 정책의 random_drop 의존:** deterministic eval에서 drop 거의 안 됨, best 1.32m은 isolated lucky drop(Representative Best `not_measurable`) → **redux 전환의 정당성**.

---

## Phase 1 → redux 전환 이유

Round 7 v3로 **SAC·인프라 처방은 모두 완성**(ent_coef 1.0→0.055, critic_loss 200k→35, #021 crash 0회). 그러나:

- 정책의 **정밀도 한계**: median drop_error 15m, bimodal 분포(11% 정밀 / 89% 빗나감)
- **random_drop 의존성**: deterministic 모드에서 reliable auto_drop 못함
- 14.87m(spawn→target) 거리가 **학습 난이도 과다**

→ 단일 파라미터 튜닝(Phase 2)이 아니라 **task 자체를 바꾸는 redux**:
- `target_enu (11,10) → (4,3)` (14.87m → 5m)
- `random_drop_prob 0.005 → 0` (정책이 직접 drop 강제)

**결과(redux v1):** random_drop 없이 830 drops 전부 auto, **96.3% success, best 0.809m**. → 정책 자체가 학습 가능함을 입증.

---

## Phase 1에서 확립된 안전장치 (영구 통합)

| 범주 | 처방 |
|------|------|
| SAC 발산 | target_entropy −15, per-sample damping, hard cap 1.0 |
| critic 안정 | Huber loss, target_q_clip=500 |
| reward 폭주 | hard cap [−200,+300], sigmoid altitude penalty |
| 인프라(#021) | TimeoutExpired catch, max_consecutive_fast_resets=100 |
| 보존 시스템 | SuccessReplay(auto+success만), DropEpisodeRecorder |

---

## 다음 (redux v2 진행 중)

curriculum learning으로 정밀화 (1m/1m/0.3m thresholds). 목표: 1m success_rate > 50%, 첫 jackpot(<0.3m), best < 0.3m, fps 5~10 회복. → [[experiments/exp_005_phase1_redux_junsang]]

---

## 관련 노트

- [[research/sac_bounded_action_target_entropy_junsang]] — 발산 근본 원인
- [[research/sac_reward_density_junsang]] — per-step density 관점
- [[errors/err_20260528_gz_timeout_recurrence_junsang]] — #021 인프라 크래시
- [[experiments/exp_005_phase1_redux_junsang]] — redux 실험 노트
- [[experiments/training_history]] · [[00_index_junsang]]
- local: `design/model_history`, `issues/master`, `backups/phase1_final_round7_v3/README.md`
