# local_dronebombard_simulation

> Drone-Bombard-Simulation 프로젝트의 **호스트 측 문서/가이드/대화 기록 모음**.
> 코드 자체는 `/home/juns/Drone-Bombard-Simulation/` (git repo) 안.

---

## 빠른 시작 — 지금 무엇을 보면 되나?

**🔥 RAD v1 Phase 1 v1~v6 종합 (2026-07-05)**: [design/design_review_2026-07-05.md](design/design_review_2026-07-05.md) — 6번 시행 종합, **진짜 원인 확정 (Issue #029: curriculum stage 설계 결함)**, stage 재설계 안 (v7 계획).

**🆕 RAD v1 design (2026-06-30, 새 framework)**: [design/rad_v1_design.md](design/rad_v1_design.md) — Relative + Approach + Drop. 2 정책 hierarchical, obs 14d 상대좌표, spawn yaw 랜덤, z Hann reward, 7 final state 조건 jackpot. v8/v9a 와 완전 다른 framework.

**처음 보는 사람**: 이 README → [design/design_review_2026-07-05.md](design/design_review_2026-07-05.md) (최신 종합) → [design/rad_v1_design.md](design/rad_v1_design.md) (framework) → [issues/master.txt](issues/master.txt)
**안건(문제점) 확인**: [issues/master.txt](issues/master.txt) → 개별 issue 파일
**훈련을 돌리려면**: [guides/drone_sim_tmux_training_guide.txt](guides/drone_sim_tmux_training_guide.txt)
**학습된 모델을 GUI 로 검증**: [guides/dgui_usage_guide.md](guides/dgui_usage_guide.md) ★ 신규 (dgui 도구)
**학습 후 GUI 검증 (이전)**: [guides/post_training_verification_guide.txt](guides/post_training_verification_guide.txt)
**parameter 의미가 궁금하면**: [parameter_log.md](parameter_log.md) §1 (Glossary)
**parameter 가 언제/왜 바뀌었는지**: [parameter_log.md](parameter_log.md) §3~4 (최신 = #42 RAD v1)
**지난 세션 내용**: [meeting_notes/](meeting_notes/) — 최신 = `meeting_notes_2026-07-05.txt` (RAD v1 Phase 1 v1~v6 종합)
**회의 발표 자료 (2026-07-05)**: [presentations/rad_v1_phase1_weekly_2026-07-05.md](presentations/rad_v1_phase1_weekly_2026-07-05.md) — RAD v1 Phase 1 주간 종합 (팀 회의용)
**Phase 2 계획 (구)**: [design/phase2_plan.md](design/phase2_plan.md) — v8 phase 2 검토 (보류, RAD 가 대체)
**RAD 사용자 thoughts 원안**: [design/two_stage_learning_plan.md](design/two_stage_learning_plan.md)
**Phase 1 백업**: [backups/phase1_final_round7_v3/](backups/phase1_final_round7_v3/) — Round 7 v3 종료 상태
**Phase 1 redux v8 백업**: [backups/phase1_redux_v8_2026-06-21/](backups/phase1_redux_v8_2026-06-21/) — v8 완성 자산 8.6 GB
**평가 모델 카탈로그**: [../ros2_ws/eval_models/README.md](../ros2_ws/eval_models/README.md) — dgui 평가용 모델 모음
**평가 결과**: [eval_logs/](eval_logs/) — dgui 결과 json 자동 저장

---

## 학습 사이클 (전체 흐름)

```
문제점 식별 (issues 폴더)
   ↓
설계 결정 (design_review + issues master)
   ↓
yaml + code 적용 (drone_drop_env.py, train_sac.py)
   ↓
parameter_log 에 entry 추가
   ↓
5k dry-run (offline) ─── 코드 검증
   ↓
150k 본학습 (online, tmux, fresh start)
   ↓
결과 분석 (wandb + drop episodes)
   ↓
issues 업데이트 → 다음 Round 결정
```

---

## 파일 구조

```
Drone-Bombard-Simulation/local/
├── README.md                         ← 이 문서
├── parameter_log.md                  ← parameter 의미사전 + 수정 history (§4 #38 = v9a)
├── eval_config.yaml                  ← [신규] dgui 의 default 설정
│
├── design/                           ← 설계 문서
│   ├── design_review.md              ← [총 요약] 현재 최종 설계만 (항상 최신, 2026-07-05 갱신)
│   ├── design_review_2026-07-05.md   ← [최신] RAD v1 Phase 1 v1~v6 종합 + 진짜 원인 확정 + stage 재설계 안
│   ├── design_review_2026-06-30.md   ← RAD 도입 결정 narrative
│   ├── design_review_2026-06-27.md   ← v9a 결과 + 다음 처방 후보 (옵션 A~G)
│   ├── design_review_2026-05-25.md   ← Round 1 결정 + 상세 이력
│   ├── design_review_2026-05-23.md   ← Tier 1 (P1~P11) 처방 이력
│   ├── rad_v1_design.md              ← [신규 framework] RAD v1 single source of truth (§12 학습 이력 갱신 완료)
│   ├── model_history.md              ← 모든 라운드 narrative (v9a + ang_vel fix + v8 + v7 + v6 추가)
│   ├── phase2_plan.md                ← Phase 2 검토 (보류)
│   └── two_stage_learning_plan.md    ← [신규] 사용자 thoughts framework — 2 단계 모드 분리 (접근 → 투하), Phase A~E
│
├── issues/                           ← 안건 관리 (문제점, 분석, 해결방안)
│   ├── master.txt                    ← 전체 관리 매뉴얼 + 현황 (2026-06-27 갱신, v9a + 신규 안건)
│   ├── issue_001~023                 ← 개별 안건 (23건)
│   ├── issue_024_ang_vel_callback_disabled.md  ← [신규] PX4 dds_topics fix
│   ├── issue_025_finetune_step_insufficient.md ← [신규] v9a 17k step 부족
│   ├── issue_026_toss_environment_indistinguishable.md ← [신규] 환경의 hover vs toss 구분 부족
│   ├── issue_027_payload_tracking_after_detach.md ← payload tracking 100% 구현 후보
│   ├── issue_028_rad_ekf_stale_after_reset.md    ← RAD Phase 1 NaN abort (진단 방향 정정 완료 — 2026-07-05)
│   └── issue_029_curriculum_stage_spawn_ignorance.md ← [신규] 진짜 학습 실패 원인 (curriculum stage 설계 결함)
│
├── meeting_notes/                    ← 세션별 작업 정리 (누적)
│   ├── meeting_notes_2026-05-20.txt
│   ├── meeting_notes_2026-05-22.txt
│   ├── meeting_notes_2026-05-23.txt
│   ├── meeting_notes_2026-05-25.txt  ← 브레인스토밍 + Round 1 결정
│   ├── meeting_notes_2026-05-26.txt  ← Round 1 분석 + Round 2 조율
│   ├── meeting_notes_2026-05-30.txt  ← Round 2 완주 + Round 3 계획
│   ├── meeting_notes_2026-05-31.txt  ← Round 3 크래시 + Round 4 시작
│   ├── meeting_notes_2026-06-03.txt  ← Round 7 시작
│   ├── meeting_notes_2026-06-05.txt  ← Phase 1 마감 + Phase 1 redux v1
│   ├── meeting_notes_2026-06-22.txt  ← dgui 완성 + ang_vel fix + v9a 처방 결정
│   ├── meeting_notes_2026-06-26.txt  ← v9a monitor + SIGTERM stop + 평가
│   ├── meeting_notes_2026-06-27.txt  ← v9a 분석 + 다음 처방 후보 + 문서 정리
│   └── meeting_notes_2026-07-05.txt  ← [신규/최신] RAD v1 Phase 1 v1~v6 종합 + 진짜 원인 확정
│
├── guides/                           ← 실행 가이드
│   ├── drone_sim_tmux_training_guide.txt      ← 학습 실행 절차
│   ├── post_training_verification_guide.txt   ← 학습 후 GUI 검증 절차 (이전 도구)
│   ├── wandb_metrics_guide.txt                ← WandB metric 정의 및 해석
│   └── dgui_usage_guide.md                    ← [신규] dgui (evaluate_gui.py) 사용법
│
├── scripts/                          ← [신규]
│   └── evaluate_gui.py               ← dgui 도구 본체
│
├── eval_logs/                        ← [신규] dgui 평가 결과 json 자동 저장
│   └── eval_<timestamp>_<model>.json
│
├── evals/                            ← Evaluate 결과 (누적, 이전)
│   ├── n1b_v2_200k_2026-05-22/
│   └── junsang_v4_milestones_2026-05-23/
│
├── notes/                            ← [신규] Claude 작업용 notes (별도 프로세스가 ../notes 로 정리)
│
├── success_replay/  → ../ros2_ws/success_replay  (symlink, Round 6+)
│   └── {wandb_run_id}/               ← 학습 중 success+auto_drop 모델 저장
│       └── success_step{N}_err{X}m.zip
│
├── presentations/                    ← 팀 발표 자료
│   ├── rad_v1_phase1_weekly_2026-07-05.md  ← [신규/최신] RAD v1 Phase 1 v1~v6 주간 종합 (팀 회의용)
│   ├── v5_history_presentation.md
│   ├── v5_history_presentation.pptx
│   └── charts/
│
├── conversation_backups/             ← Claude Code 대화 백업
├── backups/                          ← 큰 데이터 snapshot
│   ├── phase1_final_round7_v3/       ← Round 7 v3 종료 상태
│   ├── phase1_redux_v8_2026-06-21/   ← [신규] v8 완성 자산 8.6 GB
│   └── hyperparams_v8_pre_angvel_fix_20260622_042523.yaml ← [신규] ang_vel fix 전 backup
│
└── archive/                          ← 완료/이전 문서 보관

ros2_ws/eval_models/                  ← [신규] dgui 평가용 모델 (host = container bind mount)
├── README.md
├── v8_peak_step217040_err0.87m.zip   ← peak window best (10 ep 50%, BEST BASELINE)
├── v8_best_step157201_err0.07m.zip   ← global best drop (jackpot 0.07m)
├── v8_final_step303801.zip            ← 학습 종료 시점
├── v9a_preempt_step313k.zip           ← v9a SIGTERM preempt (10 ep 33%)
└── v9a_step432806.zip                 ← v9a resume rolling ckpt (CUDA error 산물, 5 ep 67%/hover_timeout 2)
```

---

## 핵심 문서 — 각 1줄 설명

| 파일 | 역할 |
|---|---|
| **issues/master.txt** | 안건 전체 현황 + 의존 관계 + 점검 절차. **결정 전 필독** (2026-06-27 갱신) |
| **design/design_review.md** | 현재 최종 설계 요약 (v9a 활성 + 다음 처방 후보) |
| **design/design_review_2026-06-27.md** | v9a 결과 + 다음 처방 후보 (target 거리 제한, drop 좌표 강제, payload tracking) |
| **design/design_review_2026-05-25.md** | Round 1 결정 상세 이력 + curriculum 원칙 |
| **design/model_history.md** | 모든 라운드 narrative (v9a + ang_vel fix + v8 + v7 + v6 신규 추가) |
| **parameter_log.md** | (§1) parameter 의미 사전 + (§3~4) 시간순 수정 history (#38 = v9a) |
| **guides/training_guide** | tmux 기반 학습 실행 절차 |
| **guides/dgui_usage_guide.md** | dgui (학습된 모델 GUI 평가) 사용법 — 신규 (2026-06-22) |
| **guides/verification_guide** | 학습 후 drop episode GUI 재생 절차 (이전 도구) |
| **meeting_notes/_2026-06-27.txt** | 최신 세션: v9a 분석 + 다음 처방 후보 + 문서 정리 |
| **meeting_notes/_2026-06-22.txt** | dgui 완성 + ang_vel callback fix + v9a 처방 결정 + 학습 시작 |

---

## 현재 상태 (2026-07-05)

### RAD v1 Phase 1 학습 v1~v6 (2026-06-30 ~ 2026-07-05)

**목표**: 300k step 완주 + sphere entry 성공률 90%+ 달성.

**시행 이력**:
- v1 (원본): NaN abort at 62k
- v2 (self-healing A1+B3): NaN abort at 171k
- v3 (curriculum 5-stage + regression): Stage 1 정체
- v4 (reward magnitude 축소 + q_clip): Stage 1↔2 cycle 반복
- v5 (stage 2 완화): 여전 실패 (hover 65%)
- **v6 (spawn_yaw ±45°, hover -30, initial pos log)**: Stage 3 진입 후 실패 cycle 반복

**진짜 원인 확정 (Issue #029)**:
- Initial pos log 실측: 정책 시작 시점 target 거리 = 5.10m
- Stage1 (radius 6m), stage2 (radius 5.5m) 는 spawn 이 이미 안 → trivial
- Stage3 (radius 4.53m) 만 학습 필요 → 정책 능력 부재 → 완전 실패
- **Curriculum stage 조건이 spawn 위치 반영 안 함**

**개발된 인프라 (재사용)**:
- Self-healing SAC (A1 gradient clip + B3 weight NaN rollback)
- Curriculum + Regression 시스템 (5-stage, window/threshold/cooldown)
- Initial position 실측 도구 (근본 진단 결정적)
- Terminal type monitor (ep 종료 원인 분류)

**다음 단계**: Stage 재설계 (v7)
- Stage1: radius 4.80m (spawn 겨우 밖)
- Stage2: 4.65m, Stage3: 4.53m, Stage4/5: +조건

자세히: [design/design_review_2026-07-05.md](design/design_review_2026-07-05.md), [issues/issue_029_curriculum_stage_spawn_ignorance.md](issues/issue_029_curriculum_stage_spawn_ignorance.md), [meeting_notes/meeting_notes_2026-07-05.txt](meeting_notes/meeting_notes_2026-07-05.txt)

---

## 이전 상태 (2026-06-27)

### Phase 1 redux 의 진짜 baseline = v8 완성 (96bokgae)
- run_name: `phase1_redux_v8_no_invalid_penalty`
- 303k step, 8,736 ep, 3,342 drops, **2,694 successes (80.6%)**, 13 jackpots, **best 0.07m**, mean 1.85m
- 핵심 처방: `invalid_drop_penalty 50→0`, `invalid_drop_threshold 50→95`, D1 = 매 drop 후 _kill_infra
- **toss 전략 발견** — drone 이 marker 지나친 후 멈춤 + pitch back + drop → payload toss
- 모든 ep 일관 행동 = 정책 완전 수렴
- 백업: `local/backups/phase1_redux_v8_2026-06-21/` (8.6 GB)
- 평가용 모델 3 개: `ros2_ws/eval_models/v8_{peak,best,final}_*.zip`

### v9a fine-tune (zjexq20k 313k = v8 + 17k step SIGTERM)
- 처방 (사용자 의도): `w_dist 1.0 → 1.5` + `drop_angaccel_penalty_scale 0.5` + `drop_angaccel_window_n 5`
- 5 ep dgui 평가: success 80%, mean 1.89m, **max ang_vel 2.10 rad/s (-16% vs v8)** ← drop_angaccel 효과 명확
- w_dist 1.5 효과 미미, 정책의 toss 그대로 유지
- 결론: 17k step 부족 (Issue #025). 사용자 의도 (지나치는 현상 해결) 안 됨.
- 모델: `eval_models/v9a_preempt_step313k.zip`

### v9a resume (xzoz52cw 313k → 432k, CUDA error 종료)
- 의도: 313k preempt 에서 resume + 300k 추가 학습 (~600k 까지)
- 실제: +119k step (7시간 18분) 진행 후 CUDA `unspecified launch failure` → container Exit 137
- preempt save 실패. rolling checkpoint 5k 마다 저장된 `sac_drop_432806_steps.zip` 구원
- 5 ep dgui: drops 3/5, success 2/3 (67%), mean 1.96m, **hover_timeout 2 ⚠️** (신규 failure mode)
- 모델: `eval_models/v9a_step432806.zip`. 추가 학습이 정책 악화 (catastrophic forgetting 의심)

### 10 EP 통계 비교 (2026-06-27) — **v8 = best baseline 확정**
| 모델 | step | drops | success≤2m | mean err | hover_timeout |
|---|---|---|---|---|---|
| **v8** (96bokgae) | 217k | 10/10 | **5/10 = 50%** | 2.002m | 0 |
| v9a 313k | 313k | 9/10 | 3/9 = 33% | 2.006m | 1 |
| v9a 432k (5 ep) | 432k | 3/5 | 2/3 = 67% | 1.96m | 2 ⚠️ |

- 이전 5 ep 100%/80% 는 **표본 운** (binomial SE 22%, 10 ep 16%)
- v8 가 v9a 보다 success 17% ↑, mean err 둘 다 ≈ 2.0m (threshold 경계)
- **사용자 결정**: v9a 처방 미달성 → 새 design framework (2 단계 모드 분리) 진행

### ang_vel callback fix (Issue #024, 2026-06-22)
- 발견: v8 학습 전체가 obs[6:9] (ang_vel) = 0 으로 진행됨에도 80% success
- root cause: PX4 `dds_topics.yaml` 의 `vehicle_angular_velocity` 가 PX4 default 에서 주석 처리
- fix: uncomment + PX4 rebuild + `limit_ang_vel: 2.0 → 10.0`
- v8 모델 영향 미미 (학습 시 항상 0 이었으므로 weights 가 그 input 에 무지)

### 신규 도구 — dgui (2026-06-22)
- `local/scripts/evaluate_gui.py` + `local/eval_config.yaml`
- alias `dgui` (~/.bashrc), 5 단계 처방 적용 (D-1, M-1, B, settle, H, camera)
- 모델 폴더: `ros2_ws/eval_models/` (host = container bind mount)
- 결과 자동 저장: `local/eval_logs/eval_<ts>_<model>.json`
- 사용법: [guides/dgui_usage_guide.md](guides/dgui_usage_guide.md)

### v6 / v7 / v8 history (간략, 2026-06-11 ~ 06-21)
- **v6** (phase1_redux_v6_hover_drop_fix): drop_wait_timeout 10→3 (#022 fps), hover_drop_block 폐기 → success 11%, invalid drop 50% 여전
- **v7** (phase1_redux_v7_safe_attach): 옵션 C + invalid_drop_penalty 50 유지 → **drop 0 (학습 실패)** — penalty 50 이 drop 회피 학습 강제
- **v8**: invalid_drop_penalty 0 + threshold 95 → 80.6% success 달성 (위 baseline)

### 신규 안건 (2026-06-22 ~ 06-27)
- **#024**: PX4 ang_vel callback disabled — 해결됨
- **#025**: Fine-tune step 수 부족 — Round 별 확인
- **#026**: Toss = 환경 자연 해법 (사용자 의도 ≠ 학습 행동) — 방향 결정 필요
- **#027**: Payload tracking 후 detach (사용자 의도 100% 구현 후보) — 검토

### 큰 설계 변경 (design/)
- **2 단계 모드 분리 학습 framework** (사용자 thoughts)
  - issues 가 아닌 design 폴더에 plan (큰 변경, ~10 일 학습)
  - 원본: `local/thoughts`
  - plan: [design/two_stage_learning_plan.md](design/two_stage_learning_plan.md)
  - 분석 보존: `local/archive/issue_028_two_stage_approach_drop_modes.md`

### 다음 단계 (사용자 최종 결정 2026-06-27)
- **v9a 처방으로는 사용자 의도 (지나치는 현상 해결) 미달성** 확인
- v8 = best baseline 유지 (이후 비교 기준)
- **새 design framework (2 단계 모드 분리) 의 구조부터 철저히 재논의** — 7 카테고리 top-down
  (MDP / reward / env / policy / training / eval / deploy)
- 모든 문서 업데이트 → compact → 카테고리 1 부터 재시작
- plan 참고: [design/two_stage_learning_plan.md](design/two_stage_learning_plan.md)

(폐기된 v9 옵션들 — fine-tune 더 진행, target 거리 제한, drop 좌표 강제, payload tracking,
 환경 변경, fresh start 는 새 framework 논의 후 재평가)

자세히: [design/design_review_2026-06-27.md](design/design_review_2026-06-27.md)

---

## 이전 상태 (2026-06-05)

- **Branch**: `junsang` (GitHub: Joshua-0922/Drone-Bombard-Simulation)
- **Phase 1 마감** — Round 7 v3 (436xl0bb, 685k 자연 종료):
  - **6,055 episodes, 162 drops, 87 auto, 16 successes, best 1.32m**
  - 모든 처방 효과 입증:
    - per-sample damping → ent_coef cap 1.0 → 0.055 회복
    - Huber + target_q_clip=500 → critic_loss 200k → 35 안정
    - 1·2차 처방 (#021) → forced restart 29회 정상, gz timeout 0회
  - 백업: `local/backups/phase1_final_round7_v3/` (609 MB)
    - sac_drop_final.zip, success_replay_436xl0bb/ (15 models), drop_episodes/, wandb_run/, 코드/설정 snapshot
- **Phase 1 eval 진단**:
  - deterministic eval 5 EP: 0-1 drops, episode 의존성 발견
  - 정책이 random_drop 보조 없이는 reliable auto_drop 못함
  - 14.87m 거리는 학습 난이도 너무 높음
- **Phase 1 redux v1** (ayi27a56, 89k 수동 중단):
  - target_enu (11, 10) → (4, 3) — spawn 부터 5m
  - random_drop_prob 0.005 → 0
  - 결과: 1,205 episodes, 830 drops, **799 successes (96.3% at 5m)**
  - **best drop 0.809m** (Round 7 v3 의 1.32m 갱신)
  - fps 18 → 2 (drop 빈번 → infra restart 누적)
  - preempt 백업: `archive/phase1_redux_v1_pause_89k/` (86 MB)
- **Phase 1 redux v2 실패** (za9zxdh6, 143k 수동 중단):
  - Curriculum gap 너무 컸음 (5m → 1m, 4× 정밀화)
  - step 123k 이후 17k+ step 0 drops (정책 drop 행동 잃음)
  - pos_scale=50 등 mis-scaled 발견
  - 보존: archive/phase1_redux_v2_failed_143k/ (case study)
- **Phase 1 redux v3** (254k 수동, jelly-baseline):
  - Scale 처방: pos_scale 50→**5**, action_vx 8→**3**, action_vy 5→**3**, max_distance 100→**20**, k_drop_proximity 0.15→**0.4**
  - 임계 완화: auto/success 1.0 → **2.0** (점진적 curriculum)
  - 결과: 396 drops, drop_error mean 3.59m
  - **당시 결론 (잘못됨)**: gap mean 1.73m, correlation 0.955 → velocity inheritance failure 가설
  - **재해석 (2026-06-07)**: drop_error ≈ d_xy 는 SDF dimensions=2 의 측정 artifact 였음 (issue #023)
- **Phase 1 redux v4** (4hz2y01h, 280k 수동, ⛔ 폐기):
  - 옵션 A (ApplyLinkWrench + EntityWrench impulse) 적용 시도
  - 188 drops 결과: drop_error mean 3.585m (v3 의 3.59m 와 동일)
  - 가설 자체가 잘못이라 효과 없음. v3 preempt 도 잘못된 reward 위에서 학습됐음.
- **Phase 1 redux v5** (시작 전, **진짜 root cause fix**):
  - 처방: `payload_0~3/model.sdf` 의 `OdometryPublisher` 에 `<dimensions>3</dimensions>` 추가
  - 진짜 원인: gz-sim8 OdometryPublisher 의 `<dimensions>` default=2 → z 항상 0 publish
  - drop_calculator 가 분리 즉시 impact 처리 (payload_z ≤ 0.04) → drop_error = drone d_xy
  - 검증: drone alt 4.06m → payload odom z = 4.053m ✓, test_v1 transfer 103.1%
  - Option A 코드 전부 제거, ApplyLinkWrench plugin 제거, `w_prediction=0`
  - **Fresh start** (v3/v4 preempt 안 씀 — 잘못된 reward 위 학습)
  - Issue #023: payload_odom_2d_bug 참조
- **학습 데이터 정리** (2026-06-05, ~10 GB 절약):
  - 총 12.7 GB → **1.9 GB**
  - 삭제: 옛 backups, 옛 wandb runs, archive 의 superseded round, success_replay 중복
  - 유지: phase1_final_round7_v3 (609MB), phase1_redux_v1_pause_89k (86MB), phase1_redux_v2_failed_143k (86MB)
- **Model history 문서**: `local/design/model_history.md` — 모든 라운드의 narrative 정리
- **Phase 2 검토 노트**: `local/design/phase2_plan.md` — 변경 후보들의 분석
- **이전 라운드 요약** (간략):
  - Round 1~3: 학습 안정성 처방 누적
  - Round 4-6: SAC entropy 발산 (Issue #019) 진단 + 처방 진화
  - Round 7 1차~v2: target_entropy=-15 도입 + #021 인프라 버그 수습
  - Round 7 v3: critic 안정 처방 + Phase 1 완성
- **검증 방법**: deterministic + stochastic eval + GUI
- **모델 저장 시스템**:
  - **SuccessReplay** (Round 6+): `is_success AND drop_trigger=='auto'` 만 저장
    - 위치: `local/success_replay/{wandb_run_id}/` (symlink → ros2_ws/success_replay)
  - 160k Round 6 v2 best: `local/success_replay/round6_v2_recovered/success_step160625_err4.36m.zip`
  - 1.32m Round 7 v3 best: `local/backups/phase1_final_round7_v3/success_replay_436xl0bb/success_step454091_err1.32m`
- **Issues**: 21건 (#021 신규). Phase 1 redux 적용 6건 (#017, #019, #020, #021 + target/random_drop)

---

## 외부 reference

- **코드 + 문서**: `/home/juns/Drone-Bombard-Simulation/` (코드는 git, 문서는 local/)
- **컨테이너**: `drone-bombard-harmonic` (bind mount: ros2_ws, gazebo_models)
- **디스크**: /home 204G (75G 사용, 120G 여유)
- **WandB**: https://wandb.ai/nayoonho0922-seoul-national-university/drone-bombard-sac
- **Claude 메모리**: `~/.claude/projects/-home-juns/memory/`

---

## 작성 / 관리 원칙

- **issues**: master.txt로 전체 관리. 결정 시 모든 관련 issue 읽기.
- **meeting_notes**: 매 세션마다 새 파일. 이전 노트 수정 X.
- **parameter_log**: 한 파일에 entry 추가. 의미 사전 (§1) 갱신.
- **design_review.md**: 총 요약 — 현재 최종 설계만 유지. Round마다 갱신.
- **design_review_날짜.md**: 상세 이력 — Round별 새 파일. 이전판 유지 (참고용).
- **이 README**: 새 파일 추가/이동 시 갱신.
