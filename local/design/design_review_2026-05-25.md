# Design Review — Drone-Bombard-Simulation (2026-05-25)

> **배경**: junsang_v4 200k + 150k 학습 결과 분석 → 구조적 문제 10건 식별 → Round 1 결정
>
> **이전 design_review**: [design_review_2026-05-23.md](design_review_2026-05-23.md) — Tier 1 (P1~P11) 처방
> **이전 curriculum**: [../archive/A_phased_curriculum_도입방안.md](../archive/A_phased_curriculum_도입방안.md) — 유효 원칙만 본 문서로 이관

작성: 2026-05-25
관련:
- [../issues/master.txt](../issues/master.txt) — 안건 전체 관리
- [../parameter_log.md](../parameter_log.md) — entry #15 (Round 1)
- [../meeting_notes/meeting_notes_2026-05-25.txt](../meeting_notes/meeting_notes_2026-05-25.txt)

---

## 1. 현재 상태 요약

junsang_v4 (Tier 1 적용) 학습 결과:
- drop은 발생하지만 정밀도 개선 없음 (drop_error 12~14m 수렴)
- total_success_count = 0 (d_error ≤ 0.5m 달성 못함)
- deterministic eval에서 drop 0회
- d_xy 불안정 (14m↔90m 진동) — 일관된 타겟 접근 실패
- inverted 44%로 최대 종료 원인

결론: Tier 1 처방은 "안전 제약"을 잡았지만, "정밀 학습"으로 이어지지 못함.
보상 구조 + drop 메커니즘이 구조적 병목.

---

## 2. 근본 문제 진단 (Issues #001~#010)

### 2-1. 보상 구조 문제

**#001 — Drop 보상이 거리에 둔감**
  drop_attempt_bonus(150)가 지배적. 15m drop과 5m drop의 차이가 21뿐.
  → 에이전트에게 정밀도 개선 동기 없음.

**#003 — 보상 스케일 과대**
  터미널 보상 분산(±400)이 per-step 접근 신호(+375)를 덮어버림.
  → critic이 "타겟 접근 = 좋다"를 학습하지 못함.
  → 드론이 이상한 방향으로 날아가는 근본 원인.

### 2-2. Drop 메커니즘 문제

**#006 — Auto_drop 구조적 한계**
  d_impact이 처음 threshold(10m) 이하가 되는 순간 강제 drop.
  에이전트가 "더 가까이 가서 정밀하게" 결정할 수 없음.

**#004 — Deterministic eval drop 0회**
  mean action이 drop zone에 도달하지 못함. drop이 noise 의존.

### 2-3. 에피소드 관리 문제

**#002 — 종료 조건 부족**
  지면 바운스 미차단, out_of_range 미구현.

### 2-4. 기타

**#007** CCIP 예측 갭 — 데이터 분석 필요
**#008** obs/action 충분성 — 분석 완료, 구조적으로 충분
**#009** Phase 1 졸업 기준 — 재정립 필요
**#010** 이중 보상 — Round 2 후보

---

## 3. Round 1 결정 — #001 + #002 + #003 + #006 동시 적용

### 3-1. 보상 재설계 (#001 + #003)

| 파라미터 | 이전 | Round 1 | 이유 |
|----------|------|---------|------|
| drop_attempt_bonus | 150 | **30** | precision 비중 상승 |
| k2_precision | 0.3 | **0.5** | 거리별 기울기 강화 |
| r_success_jackpot | 100 | **50** | 스케일 축소 |
| penalty_crash | -100 | **-50** | 스케일 축소 |
| penalty_overspeed | -50 | **-30** | 스케일 축소 |
| penalty_bad_attitude | -50 | **-30** | 스케일 축소 |
| truncation_penalty | -30 | **-15** | 스케일 축소 |

보상 curve: 0m=180, 1m=91, 2m=67, 5m=38, 10m=31, 15m=30

### 3-2. Hybrid Drop (#006)

| 파라미터 | 이전 | Round 1 |
|----------|------|---------|
| auto_drop_threshold | 10.0 | **2.0** (안전망) |
| manual_drop_threshold | (없음) | **0.5** (action[4] > 0.5 → drop) |

에이전트가 직접 drop 타이밍 결정. auto_drop은 2m 보험.

### 3-3. 종료 조건 (#002)

| 파라미터 | 이전 | Round 1 |
|----------|------|---------|
| min_altitude_start_step | 10 | **1** |
| ground_contact_altitude | (없음) | **0.5m** (무조건 crash) |
| max_distance | (없음) | **100m** (out_of_range truncate) |
| penalty_out_of_range | (없음) | **-30** |

---

## 4. Curriculum 원칙 (A_phased_curriculum에서 이관)

Phase 전환 시 반드시 지켜야 할 4가지 원칙:

1. **Phase 분리** — 시간상 명확한 경계. Phase 안에서는 환경 고정.
2. **Phase 사이 buffer reset** — 환경 바뀌면 경험도 새로. 오염 방지.
3. **Model resume** — 정책 학습 진척은 버리지 않음. buffer만 새로.
4. **계단식 (phase-based)** — 매 step 미세 변화가 아닌 구간 고정.

환경 변경 시 4가지 충돌 원천 (주의):
- ① Replay buffer 오염 — 옛 환경 경험과 새 경험 혼재
- ② Non-stationarity — MDP가 바뀌면 SAC 수렴 보장 깨짐
- ③ Attribution 상실 — 뭐가 효과 있었는지 구분 불가
- ④ Policy memory bias — 옛 정책이 새 환경에 적응 못함

→ Phase 전환 시: fresh buffer + model resume + 5k dry-run 검증

---

## 5. 검증 계획

### 5-1. 5k Dry-run (Round 1 코드 검증)

확인 항목:
- Training complete (crash 없이 완주)
- hybrid drop 작동 (manual + auto 둘 다 발생)
- out_of_range truncate 작동 (d_xy > 100m 에피소드 차단)
- ground_contact 작동 (altitude < 0.5m crash)
- truncate 비율 확인 (과도한 항목 없는지)

### 5-2. 150k 본학습 후 점검

issues/master.txt §4 점검 절차:
1. wandb 데이터 확인 (drop, d_xy, truncate, critic)
2. drop episode 분석 (error 분포, 추세)
3. deterministic evaluate (drop 발생 여부)
4. 안건별 판정 → 다음 Round 결정

---

## 6. 향후 방향

Round 1 결과에 따라:
- 성공 → Phase 2 (auto_drop 더 축소 또는 제거)
- 부분 성공 → Issue #010 (이중 보상) 추가 적용
- 실패 → 근본 재검토 (observation 변경, 알고리즘 변경 등)

Phase 1 졸업 기준 (#009)은 Round 1 결과를 보고 확정.
