---
date: 2026-06-06
tags: [index, dashboard, junsang]
status: active
type: index
owner: junsang
---

# 🧭 junsang 개인 대시보드

> 공용 [[00_index]] 와 별도로, **junsang 작업분만** 모은 개인 대시보드.
> 전역/공용 상태는 [[00_index]] 참조. 여기는 내 노트·실험·현재상태만 관리.

---

## 현재 상태 (2026-06-05)

- **알고리즘:** SAC (DampedEntropySAC), L4 GPU, PER(α0.6), LR 1e-4, tau 0.002, **target_entropy −15**, Huber + target_q_clip 500, per-sample damping
- **현재 학습:** **Phase 1 redux v2 (`za9zxdh6`, ~390k 진행 중)** — curriculum 정밀화 (auto_drop/success 1m, jackpot 0.3m) → [[daily/daily_2026-06-05_junsang]]
- **최고 성적:** **best drop 0.809m** (redux v1, 96.3% success at 5m). Phase 1 endpoint(Round 7 v3) best 1.32m
- **Phase 1 마감:** Round 7 v3(`436xl0bb`, 685k) 자연 종료, 609MB 백업. SAC·인프라 처방 모두 입증
- **핵심 교훈:** ① SAC 발산 근본=bounded action+target_entropy(−15로 처방) ② 정책의 random_drop 의존 → target (4,3)+random_drop=0으로 정면 돌파
- **인프라:** `_kill_infra`/`_kill_episode` timeout 5s→2s (drop 잦을 때 fps 폭락 처방), #021 gz timeout 1·2차 처방
- **다음 행동:** redux v2 모니터링 (1m success_rate>50%? 첫 jackpot? best<0.3m? fps 5~10?)

---

## 내 실험 현황 (Round 1 ~ Phase 1 redux)

| Round | Run ID | Steps | 상태 | 비고 |
|-------|--------|-------|------|------|
| Round 1 | ruozrv5x | 150K | ✅ 완료 | hybrid drop, best 4.64m, success 1건 |
| Round 2 | z05fx7g9 | 150K | ✅ 완료 | best 2.53m, success 16건, post-success regression 발견 |
| Round 3 | lidq3ydu | 157K | 폐기 | PER+안전망, best 4.32m, success 8건. #021 gz timeout(첫 발생) |
| Round 4 | 4j46qwpk | 146K | 폐기 | per-step density 축소 → ent_coef 6.03 발산 → [[research/sac_reward_density_junsang]] |
| Round 5 | sdjytkpv/mnlr1zpe | 156K | 폐기 | terminal 처방도 발산(ent_coef 6.16) → #019 본질적 발산 확정 |
| Round 6 v1/v2 | bfv4la9a/6b8bslmz | 294K | 폐기 | DampedEntropySAC. best 4.36m@160k. cap 갇혀 발산 |
| Round 7 1차~v2 | iobwvcrm 외 | ~385K | 폐기 | target_entropy −15 근본 처방. critic 폭주 발견 |
| **Round 7 v3** | 436xl0bb | 685K | 🥇 **Phase 1 endpoint** | best 1.32m. Huber+target_q_clip+per-sample damping |
| redux v1 | ayi27a56 | 89K | preempt 보존 | target(4,3)+random_drop=0. **best 0.809m, 96.3% success** |
| **redux v2** | za9zxdh6 | ~390K | 🔄 진행 중 | curriculum 정밀화 (1m/1m/0.3m thresholds) |

> 전체 narrative: `local/design/model_history.md` · 공용 이력: [[experiments/training_history]] · 파라미터: `local/parameter_log.md`

---

## 내 노트 인덱스

### 연구 일지 (daily/)
- [[daily/daily_2026-06-05_junsang]] — Phase 1 마감(Round 7 v3) → eval 진단 → redux v1(96.3%)·v2 시작
- [[daily/daily_2026-06-03_junsang]] — Round 6 v2 OOM 분석 → bounded action+target_entropy 근본 처방 → Round 7 시작
- [[daily/daily_2026-05-31_junsang]] — Round 4 발산 원인 규명(per-step density) → Round 5 설계·시작(Hover Terminal Penalty)

### 실험 (experiments/)
- [[experiments/exp_005_phase1_redux_junsang]] — Phase 1 redux: target(4,3)+random_drop=0, curriculum 정밀화
- [[experiments/exp_004_round5_hover_junsang]] — Round 4 발산 → Round 5 Hover Terminal Penalty

### 연구 (research/)
- [[research/phase1_summary_junsang]] — Phase 1 종합 (Round 1~7 v3 + redux, 핵심 발견 시간순)
- [[research/sac_bounded_action_target_entropy_junsang]] — SAC 발산 근본 원인: bounded action + target_entropy(−15 처방) + critic 안정화
- [[research/sac_reward_density_junsang]] — per-step density와 SAC 발산 (Round 4 교훈)

### 에러 (errors/)
- [[errors/err_20260528_gz_timeout_recurrence_junsang]] — #021 gz timeout 반복 크래시 + 오진단 정정
- [[errors/err_20260520_spin_thread_recursive_reset]] — spin thread 재귀 reset 문제

---

## 공용 허브에 기여한 항목

> 공용 파일에 행/규칙을 추가한 내역 (충돌 추적용)

- [[experiments/training_history]] — 5월 run 13행 추가 (Round 1~5 + junsang_v2~v4)
- [[research/rl_rules]] — Rule 8(per-step density 금지), Rule 9(post-success regression) + Failure Mode 5행
- [[research/reward_design]] — Round 1~5 보상 변경 이력
