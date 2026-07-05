---
date: 2026-06-05
tags: [daily, log, phase1, redux, curriculum, eval, round7v3]
type: daily
status: complete
owner: junsang
---

# 연구 일지 — 2026-06-05

> 세션 기간: 2026-06-04 ~ 2026-06-05 · 출처: `local/meeting_notes/meeting_notes_2026-06-05.txt`, `local/design/model_history.md`, `local/issues/master.txt`

---

## 오늘 한 일

- **Round 7 v3 (436xl0bb) 685k 자연 종료 → Phase 1 공식 마감** + 609MB 백업
- **Phase 1 Eval 진단** — deterministic/A2 hybrid eval로 정책 한계 발견
- **Phase 2 검토 노트 작성** (`design/phase2_plan.md`) → 단일 파라미터 대신 **Phase 1 redux** 전환 결정
- **Phase 1 redux v1 (ayi27a56)** — target (4,3) + random_drop=0, 89k에서 96.3% success
- **Phase 1 redux v2 (za9zxdh6)** — tight thresholds + curriculum, resume 진행 중
- Representative Best metric 도입, current_success_streak metric 추가, train alias 정리

---

## 주요 결정 & 발견

- **Phase 1 마감**: Round 7 v3가 685k 자연 종료. 6,055 ep / 162 drops / 16 success / **best 1.32m**. SAC·인프라 처방 모두 입증(ent_coef 1.0→0.055, critic_loss 200k→35, #021 crash 0회).
- **핵심 발견 — 정책이 random_drop에 의존**: deterministic eval에서 0~1 drops/5ep, EP2·EP3 d_xy 정확 일치(10.89m) → **episode 의존성**(PX4 EKF state carry-over). Representative Best 분석 결과 1.32m은 **isolated lucky drop**으로 판명(`not_measurable`).
- **→ Phase 1 redux 전환**: 단일 파라미터 튜닝(Phase 2)으로는 부족. **task 자체를 쉽게 + drop 강제**.
  - target_enu (11,10) → **(4,3)** (spawn에서 14.87m → 5m)
  - random_drop_prob 0.005 → **0** (정책이 직접 drop trigger 강제)
- **redux v1 결과 — 대성공**: random_drop 없이 830 drops 전부 auto, **799 success (96.3% at 5m)**, **best 0.809m**. closer target + random_drop=0 조합이 매우 효과적.
- **새 문제 — fps 폭락**: drop이 잦아지자 `_kill_infra` 5s timeout 누적 → fps 18 → 2. → v2에서 2s로 처방.
- **redux v2 = curriculum**: 96% 정책 위에 정밀화만 추가 학습 (1m/1m/0.3m thresholds).

---

## 코드 변경 사항

| 파일 | 변경 내용 |
|------|----------|
| `hyperparams.yaml` | target_enu (11,10)→(4,3), random_drop_prob 0.005→0 (redux v1) |
| `hyperparams.yaml` | auto_drop_threshold 3.0→1.0, success_threshold 5.0→1.0, jackpot_threshold 0.1→0.3 (redux v2) |
| `drone_drop_env.py` | `_kill_episode` timeout 5s→2s, `_kill_infra` 5s→2s (fps 회복) |
| `drone_drop_env.py` | drop_calculator `x_marker_x` hardcoded 11.0 → cfg 사용 (버그 fix) |
| `x_marker_world.sdf` | x_marker_0 pose (11,10,0) → (4,3,0) |
| `train_sac.py` | `current_success_streak` metric, RepresentativeBestCallback (callback only) |

> redux v1은 Fresh start, v2는 v1의 89k preempt에서 resume(curriculum). 보상공식 변경 아님 → resume 정당.

---

## 문제 & 해결

| 문제 | 해결 여부 | 메모 |
|------|----------|------|
| 정책이 random_drop 의존 (deterministic 약함) | ✅ | redux: random_drop=0 + closer target → 정책 직접 drop 학습 |
| episode 의존성 (EKF carry-over) | ⏳ | A2 hybrid(gz_world_reset)로 부분 해소, 완전 해결 X |
| 14.87m 학습 난이도 과다 | ✅ | target (4,3)로 5m 단축 |
| drop 잦음 → fps 2 폭락 | ✅ | _kill_infra timeout 5s→2s |
| jackpot 0.1m 발화 불가 | ⏳ | v2에서 0.3m로 완화, 첫 발화 도전 |

---

## 실험 이력 요약

| Round | Run ID | Steps | 결과 |
|-------|--------|-------|------|
| Round 7 v3 | 436xl0bb | 685k 완주 | best 1.32m / 16 success. **Phase 1 endpoint** (609MB 백업) |
| redux v1 | ayi27a56 | 89k 수동 | 830 drops 전부 auto, **799 success(96.3%)**, best 0.809m |
| redux v2 | za9zxdh6 | ~390k 진행 중 | curriculum 정밀화 (1m/1m/0.3m), resume from 89k |

---

## 내일 할 일

- [ ] redux v2 (za9zxdh6) 모니터링 — 새 1m success_rate > 50%?
- [ ] 첫 jackpot 발화(drop_error < 0.3m) 여부 확인
- [ ] best drop < 0.3m 도달 여부
- [ ] fps 5~10 회복 확인 (_kill_infra 처방 효과)
- [ ] rolling success_rate carryover(0.97) 해소 후 실제 추세 확인

---

## 관련 노트

- [[research/sac_bounded_action_target_entropy_junsang]] — Round 7 v3 critic 안정화(Huber+target_q_clip) 포함 발산 처방 정리
- [[experiments/training_history]]
- [[research/rl_rules]]
- [[00_index_junsang]]
- [[daily/daily_2026-06-03_junsang]]
- local: `meeting_notes/meeting_notes_2026-06-05`, `design/model_history`, `design/phase2_plan`, `issues/master`
- local: `backups/phase1_final_round7_v3/` (Phase 1 endpoint)
