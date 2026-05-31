---
date: 2026-05-31
tags: [daily, log, sac, oscillation, reward-density, round5]
type: daily
status: complete
---

# 연구 일지 — 2026-05-31

---

## 오늘 한 일

- **Round 3 (lidq3ydu) 최종 평가** — 157k 크래시 결과 정리, post-success regression 패턴 재확인
- **Round 4 (4j46qwpk) 발산 원인 규명** — auto-entropy 폭주 메커니즘 분석
- **Round 5 (sdjytkpv) 설계 및 학습 시작** — Round 4 변경 복원 + Hover Terminal Penalty 신규
- **SAC oscillation 메커니즘 분석 문서 작성** (`archive/sac_oscillation_mechanism.md`)
- Issue #017 (Hover Exploit), Issue #018 (Vision-based Observation) 정리
- `design/design_review.md` 최종 설계 갱신

---

## 주요 결정 & 발견

- **Round 4 발산 주범 = per-step reward density 변경**
  - `w_heading` 0.7→0.3 축소 + `w_distance_penalty` 0.03 신규 도입 → auto-entropy가 균형점 상실
  - ent_coef 6.03 폭주, critic_loss 230k+ → 146k에서 중단
  - **교훈: SAC는 per-step reward density에 매우 민감 — 함부로 바꾸면 안 됨**
- **Post-success regression 재확인** (Round 2·3 공통): success 보상(magnitude 큼)이 critic을 흔들어 정책 붕괴.
  - Round 3: 100~125k 최우수(avg 13.9m, success 3건) → 125k 이후 avg 35.5m로 발산
- **정상 oscillation vs 진짜 발산 구별 기준 확립**:
  - 정상: critic_loss spike 후 100 batch 내 회복, ep_len 회복, 장기 추세 개선
  - 발산: critic_loss 1000+ 지속, ep_len 1 수렴, ent_coef 폭주
- **Round 5 처방 핵심**: per-step density는 손대지 않고, **episode 종료 시점에만** Hover Terminal Penalty(-15) 부과 → SAC 안정성 유지하며 hover exploit 차단.

---

## 코드 변경 사항

| 파일 | 변경 내용 |
|------|----------|
| reward (per-step) | `w_heading` 0.3 → **0.7** 복원 (Round 2 값) |
| reward (per-step) | `w_distance_penalty` 0.03 → **0** 제거 (Round 4 발산 원인) |
| episode 종료 | **Hover Terminal Penalty 신규** — sustained hover(연속 still >200 step)에 종료 시 -15 (drop 발생 시 제외) |
| PX4 logging | 로깅 비활성 유지 (20GB 누적 → Gazebo timeout 방지) |
| 유지 | PER(α=0.6, ε=0.1, cap=30), LR 1e-4, tau 0.002, Hard cap [-200,+300], sigmoid alt penalty |

> ⚠️ 보상 구조 변경 → Round 5는 **Fresh Start** (run `sdjytkpv`, 300k)

---

## 문제 & 해결

| 문제 | 해결 여부 | 메모 |
|------|----------|------|
| Round 4 auto-entropy 발산 | ✅ | per-step density 변경이 원인 → Round 5에서 복원 |
| Hover exploit (#017) | ⏳ | Hover Terminal Penalty로 대응, Round 5에서 검증 중 |
| Post-success regression | ⏳ | sparse+큰 magnitude의 SAC 본질적 oscillation, 안전망으로 완화 |
| Round 3 Gazebo 크래시 (157k) | ✅ | PX4 로그 20GB 누적 → 로깅 비활성으로 해결 |
| Vision observation (#018) | ❌ (보류) | synthetic vision → 실제 카메라, Phase 2 과제 |

---

## 실험 이력 요약

| Round | Run ID | Steps | 결과 |
|-------|--------|-------|------|
| 3 | lidq3ydu | 157k (크래시) | 104 drops, best 4.32m, success 8건 / 100~125k 최우수 후 발산 |
| 4 | 4j46qwpk | 146k (중단) | per-step density 변경 → auto-entropy 발산 (ent_coef 6.03) |
| 5 | sdjytkpv | 300k (학습 중) | Round 4 복원 + Hover Terminal Penalty / success 5%+ 목표 |

---

## 내일 할 일

- [ ] Round 5 (sdjytkpv) 학습 모니터링 (300k)
- [ ] critic_loss spike 발생 시 정상 oscillation vs 발산 구별 적용
- [ ] 100k 도달 시 중간 평가 (avg 명중거리 / success rate 추이)
- [ ] Hover Terminal Penalty 효과 확인 (sustained hover 감소 여부)

---

## 관련 노트

- [[research/reward_design]]
- [[research/rl_rules]]
- [[experiments/training_history]]
- local: `meeting_notes/meeting_notes_2026-05-31`
- local: `design/design_review` (Round 5 최종 설계)
- local: `archive/sac_oscillation_mechanism` (oscillation 4단계 분석)
- local: `issues/issue_017_hover_exploit`, `issues/issue_018_vision_based_observation`
