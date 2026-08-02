---
date: 2026-06-03
tags: [daily, log, sac, target-entropy, bounded-action, round6, round7]
type: daily
status: complete
owner: junsang
---

# 연구 일지 — 2026-06-03

> 세션 기간: 2026-06-01 ~ 2026-06-03 · 출처: `local/meeting_notes/meeting_notes_2026-06-03.txt`, `local/design/model_history.md`

---

## 오늘 한 일

- **Round 6 v2 (6b8bslmz) OOM 분석** — 294k 중단까지 학습 추이 + 발산 메커니즘 규명
- **SAC 발산의 근본 원인 정밀 분석** — bounded action space + target_entropy 상호작용
- **Round 7 근본 처방 결정** — target_entropy −5 → −15
- **SuccessReplay 시스템 도입** + Round 6 v2 best(4.36m) 모델 보존
- **GUI watcher 도입** (episode 사이 sim 자동 복구)
- Round 7 1차 학습 시작 (run iobwvcrm, 150k)

---

## 주요 결정 & 발견

- **근본 원인 확정**: SAC 발산은 처방 형태(per-step/terminal/damping)와 무관한, **bounded action(tanh squash) + default target_entropy(−5)의 구조적 문제**.
  - tanh squash의 Jacobian 보정 → `log_prob`가 일반적으로 큰 양수 → `log_prob > 5` 흔함
  - `target_entropy=−5`면 `−target_entropy=5` → 거의 항상 `alpha↑` 압력 → ent_coef가 cap까지 단조 증가
  - 비유: "중력이 위로 작용하는 세계 + 천장" → 천장에 영원히 붙음
- **처방**: `target_entropy = −15` → `log_prob > 15`는 극단적이라 대부분 `alpha↓` → cap에 안 닿거나 닿아도 자기복구. ("중력이 아래로")
- **Round 6 v2 발산 메커니즘**: hard cap 2.0이 ent_coef를 잡았으나 그 값에서 학습이 망가짐 → entropy bonus +7 >> reward 0.7 → policy random → critic이 추종하다 loss 14M+ 폭주 → OOM(당시 진단, 후에 #021로 정정).
- best 4.36m@160k는 fragile (GUI 확인 시 일부 drop/일부 crash) → success_replay에만 보존(옵션 C).

---

## 코드 변경 사항

| 파일 | 변경 내용 |
|------|----------|
| `hyperparams.yaml` | `target_entropy` −5 → **−15** (근본 처방) |
| `hyperparams.yaml` | `ent_coef_hard_cap` 2.0 → **1.0** (안전망 강화) |
| `hyperparams.yaml` | `total_timesteps` 300k → 150k (우선 검증), run_name → round7_target_entropy_conservative |
| `train_sac.py` | DampedEntropySAC 생성자/resume 경로에 target_entropy 전달 |
| `train_sac.py` | SuccessReplay (is_success+auto_drop 시 model.save), GUI watcher |

> 발산 방어 4계층: ① target_entropy −15 (근본) ② percentile damping ③ hard cap 1.0 ④ reward hard cap

---

## 문제 & 해결

| 문제 | 해결 여부 | 메모 |
|------|----------|------|
| Round 6 v2 ent_coef cap 갇힘 발산 | ✅ (원인규명) | target_entropy=−15로 근본 처방 → Round 7 |
| critic_loss 14M+ 폭주 → OOM | ⏳ | 당시 OOM 진단, 후에 #021(gz timeout)로 정정됨 |
| best 정책 fragile | ⏳ | success_replay 보존, 정밀도는 후속 과제 |

---

## 실험 이력 요약

| Round | Run ID | Steps | 결과 |
|-------|--------|-------|------|
| Round 6 v2 | 6b8bslmz | 294k 중단 | best 4.36m@160k, success 4건. cap 갇혀 발산 |
| Round 7 1차 | iobwvcrm | 150k (시작) | target_entropy=−15 첫 적용 |

---

## 내일 할 일

- [ ] Round 7 1차 모니터링 (ent_coef 0.3~0.5 안정 유지?)
- [ ] critic_loss 정상 범위(100~500) 확인
- [ ] success_rate 점진 증가 확인
- [ ] Round 6 v2 archive 처리 (milestone 이동)

---

## 관련 노트

- [[research/sac_bounded_action_target_entropy_junsang]] — 오늘 규명한 근본 원인 정리 (메커니즘 + 처방)
- [[research/sac_reward_density_junsang]] — SAC 발산 관련 (per-step density 관점)
- [[research/rl_rules]] — Rule 8/9
- [[experiments/training_history]]
- [[00_index_junsang]]
- local: `meeting_notes/meeting_notes_2026-06-03`, `design/model_history`, `issues/issue_019_sac_entropy_divergence`
