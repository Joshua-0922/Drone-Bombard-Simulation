# A — Phased Curriculum Threshold 도입 방안

> **작성**: 2026-05-22 (junsang_v2 / N1=B v2 시절)
> **갱신**: 2026-05-23 (junsang_v4 / Tier 1 시점 — 현재 상태 반영)
>
> 상위 문서:
> - [design_review_2026-05-23.md](design_review_2026-05-23.md) §5 Tier 3 + §6-6 (high-level 흐름)
> - `~/Downloads/N1_drop_sparse_AB방안.md` (A vs B 의 원래 trade-off 분석)
>
> **2026-05-23 갱신 요약**: 이 문서의 phase curriculum 흐름은 **Tier 1 의 P6 처방으로 이미 진입 시작**. Phase 1 = 200k junsang_v4 본학습 (현재 진행 중). 옛 문서가 "200k B v2 결과에 따라 진입 결정" 가정이었으나, 현재는 처음부터 Phase 1 (threshold 10m) 로 시작. Phase 1 완주 후 § 4 시나리오 분기로 Phase 2/3 결정.

---

## 0. 한 줄 결론

> **A 는 step-by-step 자동 curriculum 이 아니라, 사람이 phase 마다 결정하는 계단식(staged) 으로 도입한다. Phase 안에선 환경 고정, phase 경계에서만 변화 + buffer reset. 코드 변경 없음 — yaml 만 손댐.**
>
> **[2026-05-23 갱신]** Phase 별 step 수는 50k (옛 가정) 가 아닌 **200k** (Tier 1 의 검증된 본학습 단위). 즉 phase 1~3 = 총 600k step.

---

## 1. 적용 시점

**[2026-05-22 시점 가정]** 200k B v2 학습이 종료된 직후. WandB 의 다음 지표로 시나리오 분기.

**[2026-05-23 갱신]** Tier 1 의 P6 처방으로 **Phase 1 (threshold 10m) 이미 진입** (200k junsang_v4). 각 phase 완주 후 다음 phase 진입 판단.

판단 지표 (junsang_v4 시점 — design_review §6-6 Step 3 와 일치):

```
성공 판정 metric (6개 중 5개+ 만족 시 성공):
  env/total_drop_count           : 누적 drop 횟수 (≥ 30)
  env/mean_d_xy (final)          : 평균 수평 거리 (≤ 10m)
  env/total_truncate_crash_count / env/total_episodes (final) : < 0.1 (entry #13 후)
  train/critic_loss (final)      : SAC critic 안정 (< 500)
  train/ent_coef (final)         : exploitation 전환 (< 0.5)
  Deterministic evaluate 5 epi   : drops ≥ 2

추가 참고 metric (entry #13 갱신 후 — 누적 count + x축 env/total_episodes):
  env/total_drop_terminated_count : drop 발동 누적 (Layer 4)
  env/total_success_count         : d_error ≤ 0.5m 누적
  env/total_jackpot_count         : d_error ≤ 0.1m 누적 (정밀 phase 핵심)
  env/total_truncate_<reason>_count : 5종 — crash/overspeed/ang_vel/inverted/timeout
  env/total_safety_violation_count : crash + overspeed 합산
  env/drop_error_actual_m         : 실제 명중 정밀도 (phase 별 감소 추세 핵심)
```

---

## 2. A 도입의 4가지 충돌 원천

A (`auto_drop_threshold` 를 풀거나 점진 조이기) 가 잘못 들어가면 발생하는 문제:

### ① Replay buffer 오염
- 500k buffer 에 B 만 경험한 "drop fires at d≤2m" 가 가득
- A 로 threshold 풀면 buffer 의 옛 경험과 새 경험이 섞임
- critic 이 두 환경을 동시에 fit → 학습 변동성 증가

### ② Non-stationarity (학습 도중 환경 변화)
- threshold 가 시간에 따라 바뀌면 MDP 가 stationary 아님
- SAC 의 수렴 보장이 형식적으로 깨짐
- 옛 정책이 새 환경에 안 맞아 일시적 성능 저하

### ③ Attribution 상실
- A 가 끝까지 켜진 상태로 학습 끝나면 "B 가 한 일" vs "A 가 한 일" 분리 불가
- 다음 의사결정에 데이터 부족

### ④ Policy memory bias
- B 200k 정책은 "drop 은 드물게 일어남" 분포를 내재
- A 갑자기 drop 빈도 늘려도 정책이 옛 가정으로 행동
- 적응 시간 필요

---

## 3. 핵심 개념 — Phase (계단) vs Continuous (경사로)

### Phased = 계단식 (✓ 채택)

학습을 시간상 분리된 구간으로 쪼개고, **각 구간 안에선 threshold 고정**.

```
Phase 1: threshold = 10  (50k step, 변함 없음)  ← 평지에서 적응
   ↓ 한 칸 내려옴 (threshold 점프)
Phase 2: threshold = 5   (50k step, 변함 없음)  ← 평지에서 적응
   ↓ 한 칸 더
Phase 3: threshold = 2   (50k step, 변함 없음)  ← 본래 목표
```

비유: 계단을 한 칸씩 내려옴. 한 칸 내려와서 안정될 때까지 머묾, 그 다음 또 한 칸.

### Continuous = 경사로 (✗ 채택 안 함)

매 step threshold 가 미세하게 변함.

```
step 0:       threshold = 10.000
step 1:       threshold =  9.9996
step 2:       threshold =  9.9992
...
step 200,000: threshold =  2.000
```

비유: 매 발걸음마다 발 밑이 미세하게 변함. 영원히 학습 따라잡기.

### 왜 phased 가 충돌 작은가

| 충돌 원천 | Continuous 에선 | Phased 에선 |
|---|---|---|
| ① Buffer 오염 | 매 step 다른 환경 경험 섞임 | Phase 시작 시 buffer reset 가능 |
| ② Non-stationarity | 영원히 학습 따라잡기 | Phase 안 stationary → SAC 수렴 가능, phase 경계만 변화 |
| ③ Attribution | 한 시점만 그 값이라 측정 불가 | Phase 별 평균 측정 가능 |
| ④ Policy bias | 옛 정책이 끝없이 안 맞음 | Phase 안에서 적응 시간 보장 |

---

## 4. Phase 1 (현재) 완주 후 시나리오 트리

**[2026-05-23 갱신]** 시나리오 트리의 진입 시점이 "B v2 종료" → "Phase 1 종료 (junsang_v4 200k)" 로 변경. 분기 기준은 design_review §6-6 Step 3 의 성공 판정과 정합.

```
                Phase 1 (junsang_v4 200k) 종료
                          │
            ┌─────────────┼─────────────┐
            ▼             ▼             ▼
   성공 (6개 중 5개+)  부분 성공        실패
   drops≥30 + d_xy≤10m drops 있지만   drops<5 또는
   + crash<10% 등      d_xy 정체       d_xy>30m
            │             │             │
            ▼             ▼             ▼
       시나리오 1     시나리오 2      시나리오 3
       Phase 2 진입   Tier 2 처방     Tier 1 재조정
       (threshold 5) (per-step ↑)    또는 Tier 3
                                     (Phase 1 retry)
```

**[옛 5/22 시점 시나리오 트리는 옆에 참고로 보존]**

```
                    200k B v2 종료
                          │
            ┌─────────────┼─────────────┐
            ▼             ▼             ▼
   drop_count ≥ 100   drop_count       drop_count
    + 정확도 양호      30~100 사이       < 30
    (B 성공)          (B 애매)         (B 부족)
            │             │             │
            ▼             ▼             ▼
       시나리오 1     시나리오 2      시나리오 3
       A 보류        Single-step A   Phased A
                     (1 phase)       (3 phase)
```

기준 수치는 가이드라인. 실제론 정확도 (drop_error) + crash rate + critic 안정성 종합 판단.

---

## 5. 시나리오별 설계

### 시나리오 1: drop_count ≥ 100 + 정확도 양호 — A 보류

B 단독으로 갭 메움.

- A 도입하지 않음
- (선택) 추가 50~100k 정밀도 학습 — 같은 hyperparameter, 계속 진행
- **충돌 원천: 없음** (A 안 도입)

비용: 0 또는 +50~100k (선택)

---

### 시나리오 2: drop_count 30~100 — Single-step A (1 phase)

가장 깨끗한 A 도입. 1 phase 만 적용.

**변경:**
```yaml
auto_drop_threshold: 2.0 → 5.0     # 단일 변경 후 고정 (curriculum 아님)
```

**충돌 회피 매트릭스:**

| 원천 | 회피 방법 |
|---|---|
| ① Buffer | **Fresh replay buffer** — 옛 buffer 폐기, model.zip 만 resume |
| ② Non-stationarity | **Single step** — threshold 한 번만 변화 후 stationary |
| ③ Attribution | Phase 분리 (B baseline 200k + A delta 50k) |
| ④ Policy bias | learning_starts=1000 으로 새 환경 warmup |

**실행:**
```bash
# 1. yaml 수정: auto_drop_threshold: 2.0 → 5.0
# 2. install/share 미러
# 3. model 만 resume, buffer 는 자동으로 새로 시작
ros2 run rl_navigation train_sac \
  --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_final.zip
```

비용: +50k step (~2-3시간 wall-clock, PX4_SIM_SPEED_FACTOR=10 기준)

**측정:** 50k 후 drop_count 증가, drop_error 평균 비교. B alone 데이터와 명확히 비교 가능.

---

### 시나리오 3: Phased curriculum (3 phase) — **[현재 우리 진행 흐름]**

**[2026-05-23 갱신]** Phase 당 step 수가 50k → **200k** (Tier 1 검증 단위) 로 확장. 또 시작 base 가 threshold 2.0 (옛) → 10.0 (Tier 1 P6 적용 후) 로 변경.

**변경 흐름 (현재):**
```
Phase 1: auto_drop_threshold = 10.0  (200k step)  ← 현재 진행 중 (junsang_v4)
   ↓ Phase 1 완주 후 success 판정
   ↓ checkpoint 저장 → archive
   ↓ yaml 수정 → install 미러
   ↓ fresh buffer + resume model
Phase 2: auto_drop_threshold = 5.0   (200k step)
   ↓ checkpoint 저장 → archive
   ↓ yaml 수정 → install 미러
   ↓ fresh buffer + resume model
Phase 3: auto_drop_threshold = 2.0   (200k step, 본래 목표)
```

각 phase 의 yaml 변경 (현재 시점):

| Phase | auto_drop_threshold | learning_starts | total_timesteps | wandb run_name |
|---|---|---|---|---|
| 1 (현재) | 10.0 | 1000 | 200000 | "junsang_v4" |
| 2 | 5.0 | 500 (단축) | 200000 | "junsang_v4_phase2" |
| 3 | 2.0 | 500 | 200000 | "junsang_v4_phase3" |

(learning_starts 단축은 phase 2~3 에서 buffer 가 작아 학습 데이터 부족 우려 → SAC random 탐색 단축으로 보완)

**각 phase 의 실행 (현재 시점):**
```bash
# Phase N 시작 전
# 1. 이전 phase 의 final 모델을 archive 로 (예: phase 1 → 2 전환)
docker exec drone-bombard-harmonic bash -c "\
  ARCHIVE=/workspace/ros2_ws/rl_checkpoints/archive/junsang_v4_phase1_threshold10_<date> && \
  mkdir -p \$ARCHIVE && \
  cd /workspace/ros2_ws/rl_checkpoints && \
  cp sac_drop_final.zip model.zip \$ARCHIVE/"

# 2. yaml 수정 (auto_drop_threshold + learning_starts + run_name)
sed -i 's/auto_drop_threshold: 10.0/auto_drop_threshold: 5.0/' \
  /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/config/hyperparams.yaml
sed -i 's/learning_starts: 1000/learning_starts: 500/' ...
sed -i 's/run_name: "junsang_v4"/run_name: "junsang_v4_phase2"/' ...

# 3. install 미러
docker cp .../hyperparams.yaml drone-bombard-harmonic:/workspace/ros2_ws/install/.../config/hyperparams.yaml

# 4. fresh buffer + resume model (SB3 자동 — buffer 새로 시작)
ros2 run rl_navigation train_sac \
  --resume /workspace/ros2_ws/rl_checkpoints/archive/junsang_v4_phase1_threshold10_<date>/sac_drop_final.zip
```

**Phase 별 분기 (실패 시) — [2026-05-23 갱신 — 우리 metric 기준]:**
```
Phase 1 결과 평가 (현재):
  ├─ 성공 (6개 중 5개+): drops≥30 + d_xy≤10m + crash<0.1 + critic<500 + ent<0.5
  │  + eval drops≥2 → Phase 2 진행
  ├─ 부분 성공: drops 있지만 d_xy 정체 → Tier 2 (per-step reward 강화) +
  │  Phase 1 추가 학습 (+100k)
  └─ 실패: drops<5, d_xy>30m, crash 다수 → Tier 1 수치 재조정
     (action_rate_limit 0.2→0.1, penalty_crash -100→-50) 또는 Tier 3 진입

Phase 2 결과 평가:
  ├─ drop_error 평균 5m 도달 → Phase 3 진행
  ├─ drop_error 큼 (5m 미만 못 도달) → Phase 2 추가 학습 (+100k)
  └─ drop_count 떨어짐 (정책 망가짐) → Phase 1 체크포인트로 회귀,
     threshold 7.0 으로 재설계

Phase 3 결과 평가:
  └─ 최종 평가 (drop_count, drop_error 1m 이내 목표, ep_rew_mean)
```

비용: +600k step (~30~36시간) — phase 별 200k. 분기 retry 시 더 늘어남.
(옛 5/22 시점 추정: phase 50k × 3 = 150k 였음)

---

## 6. 운영 가이드 (실용 단계)

### 코드 변경 필요한가?

- **시나리오 1:** 없음
- **시나리오 2:** 없음 (yaml 1줄)
- **시나리오 3:** 없음 (yaml + 사람이 phase 마다 수동 전환)

→ **자동 curriculum 로직을 코드에 넣지 *않음*.** 사람이 phase 마다 결과 보고 결정하는 게 adaptive + 안전.

### 체크포인트 관리

각 phase 의 최종 model 을 archive 에 별도 보존.

**[2026-05-23 갱신]** 현재 archive 폴더 구조:

```
rl_checkpoints/archive/
  ├── n1_d_baseline_2026-05-21/                          ← N1=D (5/21)
  ├── n1_b_w5_k05_2026-05-22/                            ← N1=B v1 (5/22)
  ├── dry_run_2026-05-21/                                ← dry-run 결과물
  ├── junsang_v2_200k_failed_2026-05-22/                 ← junsang_v2 (critic 발산)
  ├── junsang_v3_w9flirvp_failed_2026-05-23/             ← junsang_v3 (정책 학습 실패)
  ├── junsang_v4_dryrun5k_ujvpo8ry_2026-05-23/           ← Tier 1 5k 통과
  ├── junsang_v4_phase1_threshold10_<date>/              ← (Phase 1 200k 종료 시)
  ├── junsang_v4_phase2_threshold5_<date>/               ← (Phase 2 진입 시)
  └── junsang_v4_phase3_threshold2_<date>/               ← (Phase 3 진입 시)
```

(`sac_drop_milestone_<step>.zip` 자동 milestone 50k 마다도 archive 에 별도 보존됨)

### Replay buffer 처리

각 phase 시작 시 *옛 buffer 사용하지 않음*. SB3 의 SAC 는 `--resume <model.zip>` 만 하면 buffer 는 자동으로 새로 시작 (preempt_replay.pkl 사용 안 함).

옛 buffer 는 archive 에 보존만 — 나중에 재현이 필요할 경우 reference 용.

### Online vs Offline 모드

각 phase 학습 시 mode 는 자유:
- **Online:** 실시간 wandb 모니터링 가능. 노트북 안정적일 때.
- **Offline + sync 스크립트:** 네트워크 불안정 / 노트북 이동 시.

(이번 세션의 deadlock 사고 사례는 메모리에 기록됨.)

---

## 7. 모니터링 (각 phase 진행 중)

WandB 대시보드: https://wandb.ai/nayoonho0922-seoul-national-university/drone-bombard-sac

**[2026-05-23 갱신]** junsang_v4 의 신규 metric 6 종 포함:

| 지표 | 정상 시그널 | 위험 시그널 |
|---|---|---|
| **`env/total_drop_count`** (NEW) | phase 내 stair-step 단조 증가 | 정체 또는 거의 0 → threshold 더 풀기 |
| `env/drop_count` (per-rollout) | rollout 마다 증가 | 0 지속 → drop trigger 안 됨 |
| **`env/drop_terminated_rate`** (NEW) | 단조 ↑ (학습 진행 신호) | 정체 → drop 학습 못함 |
| `env/drop_error_actual_m` | phase 내 감소 추세 | 증가 → 정책 망가짐 (회귀 필요) |
| `env/mean_rew_impact` | phase 시작 시 일시 dip 후 회복 | 영원히 dip → buffer reset 의도와 다른 결과 |
| `rollout/ep_rew_mean` | phase 사이 일시 dip, 빠르게 회복 | 회복 안 함 → phase 디자인 잘못 |
| **`env/total_truncate_crash_count`** (entry #13) | 학습 후반 증가율 감소 (plateau) | 가파른 ↑ → 정책이 자주 추락 |
| `env/total_truncate_ang_vel_count` (entry #13) | 학습 진행 시 plateau | total_episodes 의 50%+ → limit_ang_vel/action_rate_limit 너무 빡셈 |
| `env/safety_violation_rate` | 학습 후반 낮음 | 증가 → 과속/충돌 다발 |
| **`train/critic_loss`** | 안정 (junsang_v4 M2=1: ~15~30 범위) | 폭주 (>5000) → terminal/step 균형 깨짐 (junsang_v2 패턴) |
| **`train/ent_coef`** | 단조 감소 (1.0 → 0.5 → 0.2) | 증가 또는 > 1.0 → SAC auto-temp 통제 불능 |
| `env/mean_d_impact` | 점진 감소 | 줄어드는데 drop_count 안 늘면 → drive-through hack 의심 |

---

## 8. 충돌 회피의 핵심 원칙 4가지

이 문서 전체의 골격:

1. **Phase 분리** — 시간상 명확한 경계. A 가 시작되는 순간을 기록 가능하게.
2. **Phase 사이 buffer reset** — 환경 바뀌면 경험도 새로. 오염 + attribution 둘 다 해결.
3. **Model resume (정책만 이어감)** — 정책 학습 진척은 버리지 않음.
4. **Step-based 가 아닌 phase-based curriculum** — 매 step 미세하게 바뀌는 게 아니라, "phase X 안에선 threshold 고정". 정책이 적응할 시간 보장.

---

## 9. 관련 문서 / 메모리

- `~/Downloads/N1_drop_sparse_AB방안.md` — A vs B 의 원래 trade-off 분석 (상위 문서)
- `~/local_dronebombard_simulation/meeting_notes/meeting_notes_2026-05-22.txt` — N1=B v2 결정 및 200k 학습 시작 기록
- `~/local_dronebombard_simulation/meeting_notes/meeting_notes_2026-05-23.txt` — 200k v2 실패 + branch 교체 + Tier 1 (P1-P11) 적용 + junsang_v4 진입 (이 문서는 5/22 v2 시점 작성 — 현재 시점 Tier 1 의 curriculum 처방으로 일부 변경됨)
- `~/.claude/projects/-home-juns/memory/project_junsang_v4_setup_2026-05-23.md` — 현재 학습 상태 메모리 (CURRENT)
- `~/.claude/projects/-home-juns/memory/feedback_train_sac_graceful_kill.md` — 학습 중단 시 SIGINT 사용 (phase 전환 시 사용됨)
- `Drone-Bombard-Simulation/CLAUDE.md` — RL 규칙. A 는 *보상* 변경이 아니라 *환경* 변경이라 fresh start 강제 아님, 단 buffer reset 권장.

---

## 부록 — Phase 전환 절차 체크리스트

**[2026-05-23 갱신]** 각 phase 전환 시 따라할 순서 (현재 시점):

```
[ ] 1. 이전 phase 학습 자연 완주 대기 (200k 도달 시 sac_drop_final.zip 저장)
       또는 SIGINT graceful kill (OOM/중단 시 — sac_drop_preempt.zip)
       (memory: feedback_train_sac_graceful_kill 참조)
[ ] 2. 200k 완주 후 결과 평가 (deterministic evaluate 5 epi 포함)
       성공 판정 (6개 중 5개+ 만족) — design_review §6-6 Step 3
[ ] 3. 이전 phase 산출물 archive 로 cp
       cp sac_drop_final.zip + model.zip
       → archive/junsang_v4_phase{N}_threshold{X}_<date>/
[ ] 4. yaml 수정 (auto_drop_threshold + learning_starts + run_name)
       - Phase 2: 10.0 → 5.0, learning_starts 1000 → 500, run_name "phase2"
       - Phase 3: 5.0 → 2.0, run_name "phase3"
[ ] 5. install/share 미러링 (docker cp)
       — yaml: cp src/.../config/hyperparams.yaml → install/.../config/hyperparams.yaml
       — drone_drop_env.py 변경 있으면 동일
       — **train_sac.py 도 미러** (entry #13 의 callback overhaul 적용 시점):
         docker cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py \
           drone-bombard-harmonic:/workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py
[ ] 6. host shell 사전 정리 (이전 sim process kill, /tmp 정리)
       guides/drone_sim_tmux_training_guide.txt §2 참조
[ ] 7. tmux + WANDB_MODE 결정 (online vs offline)
[ ] 8. 새 phase 학습 시작 (--resume 이전 phase model.zip, fresh buffer)
       ros2 run rl_navigation train_sac --resume <archive_path>/sac_drop_final.zip
[ ] 9. 학습 도중 wandb 대시보드 10~30분 단위 모니터링
[ ] 10. phase 종료 후 결과 평가 → 다음 phase 분기 결정 (§ 4 + § 5 참조)
```

---

================================================================================
 끝 — 다음 단계 [2026-05-23 갱신] : Phase 1 (junsang_v4 200k) 완주 후 결과 분석
       → § 4 시나리오 트리 분기 (성공 → Phase 2, 부분 성공 → Tier 2, 실패 → retry)
================================================================================
