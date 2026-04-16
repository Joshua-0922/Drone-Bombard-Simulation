# CLAUDE.md

---

## 세션 시작 프로토콜 (MANDATORY)

가장 최근 `notes/daily/daily_{YYYY-MM-DD}.md` 를 먼저 읽어 전날 컨텍스트를 복원한다.

---

## Obsidian 연구 비서 규칙 (MANDATORY)

모든 연구 기록·에러 해결·실험 결과는 `notes/` 폴더에 마크다운으로 작성한다.

```
notes/
├── 00_index.md          # 전체 대시보드 (항상 최신 유지)
├── daily/               # 하루 마감 연구 일지
├── research/            # 이론·설계·아키텍처
├── experiments/         # 학습 실험 (WandB 연동)
├── errors/              # 에러 해결 기록
├── sessions/            # 세션 작업 상세 기록
└── references/          # 논문·문서
```

**파일 네이밍:** `research/{topic}.md` / `experiments/exp_{NNN}_{run_id}_{title}.md` / `errors/err_{YYYYMMDD}_{slug}.md` / `sessions/session_{YYYY-MM-DD}.md` / `daily/daily_{YYYY-MM-DD}.md`

**YAML frontmatter 필수:** `date`, `tags`, `status`, `type` (experiments는 `wandb_run` 추가)

**수식:** 모든 수식은 LaTeX (`$...$` / `$$...$$`)

### Obsidian 그래프 관리 규칙

**고립 노드 금지** — 새 파일 생성 시 반드시 아래 허브 중 하나에서 wikilink를 추가한다.

| 허브 노트 | 역할 |
|-----------|------|
| `[[00_index]]` | 전체 대시보드, 모든 주요 노트의 진입점 |
| `[[research/system_overview]]` | 시스템 컴포넌트 허브 |
| `[[research/rl_rules]]` | RL 규칙·실험 허브 |
| `[[experiments/training_history]]` | 실험 이력 허브 |

**wikilink 규칙:** 상대경로 사용 `[[research/reward_design]]` / 새 실험 → `training_history` + `00_index` 양쪽 링크 / 새 에러 → 원인 실험 노트 + `00_index` 링크 / daily → 작성/수정 노트 전부 wikilink

### 세션 종료 전 자동 수행

1. `notes/sessions/session_{YYYY-MM-DD}.md` 생성/업데이트
2. `notes/00_index.md` 현재 상태 업데이트
3. 에러 해결 → `notes/errors/` 기록
4. 실험 시작/완료 → `notes/experiments/` 기록

### VM 종료 전 자동 수행 (MANDATORY)

`notes/daily/daily_{YYYY-MM-DD}.md` 생성/업데이트. 양식: `notes/daily/_template.md`

필수 섹션: **오늘 한 일** / **주요 결정 & 발견** / **코드 변경 사항** / **문제 & 해결** / **내일 할 일** / **관련 노트**

---

## Development Environment

개발은 **Docker 컨테이너 내부**에서만. 호스트 VM은 git 작업 전용.

상세 명령: `[[sessions/commands]]`

---

## RL Training (Method A, Self-Managed)

`DroneDropEnv._start_infra()`가 모든 인프라를 내부 관리.

> **⚠️ source 순서 필수:** `/root/ros2_ws/install/setup.bash` → `/workspace/ros2_ws/install/setup.bash`
> 순서 틀리면 `px4_msgs` import 에러로 에피소드 노드 silent crash.

상세 명령: `[[sessions/commands]]`

---

## Auto-Logging & Git Sync (MANDATORY)

모든 주요 작업·코드 수정·세션 종료 전 Claude는 **자동으로**:

1. `RL_Project_Log.md` 4-section 구조로 업데이트 (허락 불필요)
2. Git push (`git add . && git commit -m "Auto-sync: [요약]" && git push origin main`)

**RL_Project_Log.md 구조:** `# 1. Current State` / `# 2. Recent Progress` / `# 3. Remaining Tasks` / `# 4. Training History (추가만 가능)`

---

## RL 핵심 규칙 (MANDATORY)

상세: `[[research/rl_rules]]`

1. **Fail-Fast:** 코드 변경 후 dry-run (2–3 에피소드) → 성공 확인 후 full training.
2. **병렬화:** `num_envs=1` 고정 (Gazebo lockstep 병목).
3. **보상 공식 변경 → 항상 Fresh Start.** Replay buffer 재사용 금지.
4. **거리 보상:** 권장 선형 보상. 지수 포텐셜 사용 전 $e^{-k_1 d_{max}} > 10^{-4}$ 확인.
5. **체크포인트:** 물리 폭발 후 preempt 재개 금지 → rolling checkpoint 사용.
6. **WandB:** 첫 롤아웃 후 `env/mean_rew_dist ≠ 0` 확인.

### Known Failure Modes

| 증상 | 원인 | 해결 |
|------|------|------|
| `mean_rew_dist = 0` | 지수 포텐셜 포화 | 선형 보상으로 전환 |
| `mean_d_xy` → 1e11 | Gazebo ODE 폭발 | 3중 방어 레이어 (`[[errors/err_20260320_physics_explosion]]`) |
| CRUISE 타임아웃 반복 | PX4 arm race | `reset()` 재시도; fps 하락 확인 |
| fps 급감 | CRUISE 65 s 대기 / ODE 크래시 | 로그에서 "Timed out waiting for CRUISE" |

---

## 시스템 아키텍처 & 상세 레퍼런스

→ `[[research/system_overview]]` (패키지, 토픽, 좌표계, 브리지, YOLO)
→ `[[research/reward_design]]` (보상 함수 LaTeX 상세)
→ `[[research/rl_rules]]` (RL 규칙 상세, WandB 메트릭)
→ `[[research/architecture]]` (Method A 아키텍처)
→ `[[sessions/commands]]` (Docker, 빌드, 학습, Git 명령 전체)
