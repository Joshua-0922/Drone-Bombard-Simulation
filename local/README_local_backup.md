# local_dronebombard_simulation

> Drone-Bombard-Simulation 프로젝트의 **호스트 측 문서/가이드/대화 기록 모음**.
> 코드 자체는 `/home/juns/Drone-Bombard-Simulation/` (git repo) 안.

---

## 빠른 시작 — 지금 무엇을 보면 되나?

**처음 보는 사람**: 이 README → [design/design_review.md](design/design_review.md) → [issues/master.txt](issues/master.txt)
**안건(문제점) 확인**: [issues/master.txt](issues/master.txt) → 개별 issue 파일
**훈련을 돌리려면**: [guides/drone_sim_tmux_training_guide.txt](guides/drone_sim_tmux_training_guide.txt)
**학습 후 GUI 검증**: [guides/post_training_verification_guide.txt](guides/post_training_verification_guide.txt)
**parameter 의미가 궁금하면**: [parameter_log.md](parameter_log.md) §1 (Glossary)
**parameter 가 언제/왜 바뀌었는지**: [parameter_log.md](parameter_log.md) §3~4
**지난 세션 내용**: [meeting_notes/](meeting_notes/) — 최신 = `meeting_notes_2026-06-03.txt`
**Phase 2 계획**: [design/phase2_plan.md](design/phase2_plan.md) — 변경 후보들의 검토 + 실험 큐
**Phase 1 백업**: [backups/phase1_final_round7_v3/](backups/phase1_final_round7_v3/) — Round 7 v3 종료 상태

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
├── parameter_log.md                  ← parameter 의미사전 + 수정 history
│
├── design/                           ← 설계 문서
│   ├── design_review.md              ← [총 요약] 현재 최종 설계만 (항상 최신)
│   ├── design_review_2026-05-25.md   ← Round 1 결정 + 상세 이력
│   └── design_review_2026-05-23.md   ← Tier 1 (P1~P11) 처방 이력
│
├── issues/                           ← 안건 관리 (문제점, 분석, 해결방안)
│   ├── master.txt                    ← 전체 관리 매뉴얼 + 현황 + 점검 절차
│   ├── issue_001~010                 ← 개별 안건 (10건)
│   └── (학습 결과마다 업데이트)
│
├── meeting_notes/                    ← 세션별 작업 정리 (누적)
│   ├── meeting_notes_2026-05-20.txt
│   ├── meeting_notes_2026-05-22.txt
│   ├── meeting_notes_2026-05-23.txt
│   ├── meeting_notes_2026-05-25.txt  ← 브레인스토밍 + Round 1 결정
│   ├── meeting_notes_2026-05-26.txt  ← Round 1 분석 + Round 2 조율
│   ├── meeting_notes_2026-05-30.txt  ← Round 2 완주 + Reset 버그 + Round 3 계획
│   └── meeting_notes_2026-05-31.txt  ← [최신] Round 3 크래시 + Hover exploit + Round 4 시작
│
├── guides/                           ← 실행 가이드
│   ├── drone_sim_tmux_training_guide.txt      ← 학습 실행 절차
│   ├── post_training_verification_guide.txt   ← 학습 후 GUI 검증 절차
│   └── wandb_metrics_guide.txt                ← WandB metric 정의 및 해석
│
├── evals/                            ← Evaluate 결과 (누적)
│   ├── n1b_v2_200k_2026-05-22/
│   └── junsang_v4_milestones_2026-05-23/
│
├── success_replay/  → ../ros2_ws/success_replay  (symlink, Round 6+)
│   └── {wandb_run_id}/               ← 학습 중 success+auto_drop 모델 저장
│       └── success_step{N}_err{X}m.zip
│
├── conversation_backups/             ← Claude Code 대화 백업
├── backups/                          ← 큰 데이터 snapshot
│
└── archive/                          ← 완료/이전 문서 보관
    ├── A_phased_curriculum_도입방안.md  ← curriculum 원칙은 design_review로 이관
    ├── sac_oscillation_mechanism.md     ← SAC 진동 메커니즘 참고
    └── (기타 이전 가이드, plan 등)
```

---

## 핵심 문서 — 각 1줄 설명

| 파일 | 역할 |
|---|---|
| **issues/master.txt** | 안건 전체 현황 + 의존 관계 + 점검 절차. **결정 전 필독** |
| **design/design_review.md** | 현재 최종 설계 요약 (보상, drop, 종료 조건, SAC, 환경) |
| **design/design_review_2026-05-25.md** | Round 1 결정 상세 이력 + curriculum 원칙 |
| **parameter_log.md** | (§1) parameter 의미 사전 + (§3~4) 시간순 수정 history |
| **guides/training_guide** | tmux 기반 학습 실행 절차 |
| **guides/verification_guide** | 학습 후 drop episode GUI 재생 절차 |
| **meeting_notes/_2026-05-25.txt** | 최근 세션: 문제점 10건 진단 + Round 1 결정 |

---

## 현재 상태 (2026-06-05)

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
- **Phase 1 redux v2 진행 중** (za9zxdh6, resume from 89k):
  - **curriculum learning** — 이전 96% (5m) 정책 위에 정밀화 학습
  - auto_drop_threshold 3.0 → **1.0m**
  - success_threshold 5.0 → **1.0m**
  - jackpot_threshold 0.1 → **0.3m** (도달 가능 영역)
  - `_kill_infra` timeout 5s → **2s** (fps 회복)
  - 코드 추가: `env/current_success_streak` metric (callback only, 다음 학습부터)
  - 누적 target ~390k step
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
