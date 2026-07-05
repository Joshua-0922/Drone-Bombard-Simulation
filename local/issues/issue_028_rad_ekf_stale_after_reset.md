# Issue #028 — RAD Phase 1 학습 abort: PX4 EKF stale after reset

작성: 2026-06-30
갱신: 2026-07-05 (진단 방향 정정)
상태: 🟡 진단 정정 완료. NaN abort 는 self-healing (v2 A1+B3) 로 예방. 진짜 학습 실패 원인은 별개 (Issue #029) 로 이관.
관련:
- [../design/rad_v1_design.md](../design/rad_v1_design.md) — RAD framework
- [../design/design_review_2026-07-05.md](../design/design_review_2026-07-05.md) — 이번 주 종합
- [issue_029_curriculum_stage_spawn_ignorance.md](issue_029_curriculum_stage_spawn_ignorance.md) — 진짜 학습 실패 원인
- jekyun_v2 commit 09ea871 — v8 의 simulation offset patch (참고)
- memory: `feedback_simulation_offset_root_cause.md`

---

## ⚠️ 진단 방향 정정 (2026-07-05)

**원래 진단** (2026-06-30 작성):
- Root cause 를 "PX4 EKF stale after reset" 로 지목
- Cruise timeout, mid-air arm 문제 등을 원인 후보로 열거

**정정된 진단** (2026-07-05):
- v6 실측 (initial pos log) 로 cruise 는 **정상 작동** 확인
- CRUISE timeout log 매우 적음 (v4=0, v5=4, v6=0)
- 정책 시작 시점 target 거리 5.10m 로 안정
- 즉 cruise 는 원래 로직 대로 정상 (speed 도달 후 즉시 HANDOFF, 이동 거리만 짧음)

**NaN abort 자체는 별개 문제**:
- v2 부터 도입된 self-healing (A1 gradient clip + B3 weight NaN rollback) 로 예방
- v2~v6 학습에서 nan_rollback_count = 0 (발동 안 함)
- Gradient clip 이 예방 역할 지배적

**진짜 학습 실패 (stage3 통과 못 함) 원인은 Issue #029** — curriculum stage 조건이 spawn 위치 (target 거리 5m) 를 반영 안 함.

---

---

## 1. 증상

RAD v1 Phase 1 학습이 abort. 두 번 시도 모두 실패:

| 시도 | 도달 | abort 사유 |
|---|---|---|
| #1 (run g8mvzniw) | ~62k step / 552 ep | `reset() recursively > 2 times` |
| #2 (run v4u49i4z, recursion limit 2→5, QoS transient_local fix) | ~1.2k step / 20 ep | CRUISE timeout cycle. 더 빠른 collapse |

→ Recursion limit 늘려도 root cause 안 고침. QoS fix 도 부분 효과만.

---

## 2. 진단 — 핵심 발견

`/tmp/episode_0.log` 의 mission_manager_rad log:

```
PX4 armed at NED z=-23.51 — TAKEOFF.
Altitude Reached → YAW_INIT
Spawn yaw set: base=-73.7°, final=-21.5° → CRUISE
[그 후 log 없음 — CRUISE 단계에서 stuck]
```

**핵심**: `PX4 armed at NED z=-23.51` — drone 이 spawn (z=0.24m) 직후 arm 했는데 PX4 가 **z=23.51m altitude** 로 인식.

→ **PX4 EKF stale state** — set_pose 가 drone 위치 reset 했지만 PX4 EKF 가 이전 ep 의 위치 유지.

→ TAKEOFF 가 `altitude_threshold = max(arm_z, 0) - target_altitude*0.95 = -4.75` 검사 → drone z=-23.51 이미 -4.75 이하 → TAKEOFF 거짓 통과.

→ CRUISE 단계에서 drone 이 23m 에 있는 상태로 head 방향 1m/s 가속 시도 → drone vel 못 잡힘 → STATE_HANDOFF 못 감 → env 가 TRACKING 못 받음 → CRUISE timeout cycle.

---

## 3. v8 와의 차이 — RAD 가 더 sensitive 한 이유

v8 도 같은 simulation offset 문제 있었음 (jekyun_v2 4 patch 로 해결). 단 RAD 에서 더 자주 발생:

| 측면 | v8 | RAD |
|---|---|---|
| Spawn yaw | 고정 (always 0°) | 랜덤 ±90° |
| Cruise 메커니즘 | (1, -1) m/s 자동 이동 | head 방향 1m/s 가속 + speed_xy ≥ 0.95 검사 |
| EKF stale 영향 | drone 이 같은 방향 cruise → state 비교적 일관 | 매 ep 다른 trajectory → EKF state 다양 → corruption 빈도 ↑ |
| Cruise 종료 조건 | 단순 "cruise 자동 이동 시작" | `speed_xy ≥ cruise_target*0.95` 도달 검사 — drone vel 못 잡힘 시 stuck |

→ RAD 의 cruise 조건이 더 까다로움 + spawn yaw 다양성이 EKF stale 빈도 ↑.

---

## 4. 시도한 fix 들 (효과 미미)

| Fix | 효과 |
|---|---|
| recursion limit 2 → 5 | 더 많은 retry → 더 길게 stuck cycle 진행 → 결국 abort |
| `mission_state` publisher/subscriber TRANSIENT_LOCAL QoS | 첫 10 ep 만 OK. 그 후 같은 collapse |

→ 두 fix 모두 root cause (PX4 EKF stale) 안 고침.

---

## 5. Root cause 가설

| 가설 | 근거 |
|---|---|
| **A. set_pose 가 PX4 EKF reset 못 함** | v8 의 known issue (jekyun_v2 patch 가 해결). RAD 에서 다시 발생 |
| B. _gz_reset_poses 의 settle 시간 부족 | 현재 5s. EKF reconvergence 부족 가능 |
| C. mid-air arm 처리가 RAD 에서 잘못 작동 | mission_manager_rad 의 altitude_threshold 가 mid-air arm 처리 → false TAKEOFF 통과 |
| D. cruise 가 잘못된 위치 (23m) 에서 시작 → drone_controller 의 vel cmd 가 PX4 에 도달 못 함 | drone_controller log 가 PX4 connected 후 멈춤 |

→ 가설 A + C 조합이 가장 가능성 ↑.

---

## 6. 다음 session 의 작업 plan

### Step 1 — jekyun_v2 commit 09ea871 의 4 patch 가 RAD env 에 적용됐는지 검증

확인 항목:
- spin_thread join 보장 (drone_drop_env_rad.py line 646-652) ✅ 있음
- infra 재사용 시 EKF reset 강제 — 추적 필요
- resetWorld 직후 PX4 명령 차단 (settle) — `_gz_world_reset` line 2051 의 5s sleep ✅
- velocity callback flush — 확인 필요

### Step 2 — PX4 EKF 강제 reset 메커니즘 추가

후보:
- a. `_gz_reset_poses` 가 PX4 EKF reset MAVLink command 보냄 (예: SET_HOME_POSITION + reset estimator)
- b. _start_episode 마다 PX4 SITL 재시작 (모든 ep 마다 — 비용 ↑↑)
- c. mid-air arm 차단 — mission_manager_rad 가 arm 시 |z| > 1m 면 abort + retry

### Step 3 — mission_manager_rad 의 TAKEOFF 단계 robustness

- d. mid-air arm 거부 — `arm_z` 가 abnormal 시 mission_manager_rad 가 자살 → env 가 retry
- e. cruise 의 speed_xy 도달 timeout — 현재 무한 wait. 5s timeout 추가 → 못 도달 시 retry signal

### Step 4 — 더 강한 진단 logging

- `infra/reset_pre_v`, `infra/reset_post_cruise_v` 만 있음
- 추가: `infra/arm_z`, `infra/cruise_failed_at_yaw_init` 등

---

## 7. 시도한 학습의 archive

```
rl_checkpoints_rad/archive/
├── dryrun_5k_20260630_054433/   ← 정상 dryrun (이전)
├── phase1_62k_abort_20260630_090657/   ← #1 시도 (62k abort)
└── abort3_qos_fix_20260630_*/    ← #2 시도 (1.2k abort)
```

---

## 8. 관련 wandb runs

| run id | 시기 | step | 결과 |
|---|---|---|---|
| `g8mvzniw` | 2026-06-30 ~05:45-07:30 | ~62k / 552 ep | abort (recursion > 2) |
| `v4u49i4z` | 2026-06-30 ~18:30-19:00 | ~1.2k / 20 ep | abort (recursion > 5 시 cycle) |

→ wandb 의 `infra/reset_*` metric 분석 (analyze_reset_diag.py) 으로 EKF stale 패턴 정량 가능.

---

## 9. 학습 재개 전 체크리스트

- [ ] Step 1 — jekyun_v2 4 patch 코드 검증
- [ ] Step 2 — PX4 EKF 강제 reset 메커니즘 add
- [ ] Step 3 — mission_manager_rad robustness (mid-air arm 거부 또는 cruise timeout)
- [ ] Step 4 — 추가 logging
- [ ] Smoke test (50 ep) — abort 안 일어나는지
- [ ] Phase 1 본학습 재개
