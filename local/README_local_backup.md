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
**지난 세션 내용**: [meeting_notes/](meeting_notes/) — 최신 = `meeting_notes_2026-05-25.txt`

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
├── conversation_backups/             ← Claude Code 대화 백업
├── backups/                          ← 큰 데이터 snapshot
│
└── archive/                          ← 완료/이전 문서 보관
    ├── A_phased_curriculum_도입방안.md  ← curriculum 원칙은 design_review로 이관
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

## 현재 상태 (2026-05-30)

- **Branch**: `junsang` (GitHub: Joshua-0922/Drone-Bombard-Simulation)
- **Round 1** (ruozrv5x, 150k): 432 drops, best 4.64m, avg 14.02m, success 1건
- **Round 2** (z05fx7g9, 150k):
  - 427 drops, best **2.53m**, avg 19.09m, success **16건 (16배 증가)**
  - 종료 조건 진화: max_altitude/stagnation 제거 → drop 시점 고도 페널티
  - Deterministic eval: drop 0건, 모두 crash 종료
  - 발견: Reset 버그 (n_steps=1 가짜 success 13건) — **해결**
  - 발견: **Post-success regression** — success 직후 발산 패턴
- Round 3 첫 시도 (q13hli0y): 지수 페널티 폭주 (-6.77e+9), 30k 중단
- Round 3 수정 학습 (lidq3ydu, 157k 크래시):
  - 104 drops, **best 4.32m**, success 8건 (Round 2 대비 **2배 속도**)
  - 100~125k 최우수 (avg 13.9m, success 3건)
  - PX4 로그 20GB 누적 → Gazebo timeout 크래시
- Round 4 발산 (4j46qwpk, 146k 중단):
  - per-step density 변경 → SAC auto-entropy 양성 피드백 발산
  - ent_coef 6.03, critic_loss 230k+
  - 교훈: per-step 보상 magnitude 변경 위험
- **Round 5 학습 중** (sdjytkpv, 300k):
  - Round 4 처방 전면 복원 (w_heading 0.7, distance_penalty 0)
  - **신규**: Hover Terminal Penalty
    - max_consecutive_still > 200 step → -15 (episode 종료 시)
    - Drop 시 제외 — per-step density 보존 → SAC 안정성 유지
  - PER + LR/Tau + 안전장치 4종 + PX4 로깅 비활성 유지
- **검증 방법**: deterministic evaluate + GUI
- **best drop 모델 저장**: auto drop 최고 기록 시 가중치 .zip 자동 저장
- **Issues**: 18건. Round 5 적용 1건(#017 재처방), 보류 1건(#018 Vision)

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
