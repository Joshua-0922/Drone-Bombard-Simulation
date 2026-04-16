# CLAUDE.md

---

## 세션 시작 프로토콜

MEMORY.md는 자동 로드됨. 추가 컨텍스트 필요 시 최근 daily 노트 읽기.

---

## Obsidian 연구 비서 규칙 (MANDATORY)

모든 연구 기록·에러 해결·실험 결과는 `notes/` 폴더에 마크다운으로 작성.

**파일 네이밍:**
- `research/{topic}.md`
- `experiments/exp_{NNN}_{run_id}_{title}.md`
- `errors/err_{YYYYMMDD}_{slug}.md`
- `sessions/session_{YYYY-MM-DD}.md`
- `daily/daily_{YYYY-MM-DD}.md`

**YAML frontmatter 필수:** `date`, `tags`, `status`, `type` (experiments는 `wandb_run` 추가)

**수식:** LaTeX (`$...$` / `$$...$$`)

### 고립 노드 금지

새 파일 생성 시 반드시 아래 허브 중 하나에서 wikilink 추가:

| 허브 노트 | 역할 |
|-----------|------|
| `[[00_index]]` | 전체 대시보드 |
| `[[research/rl_rules]]` | RL 규칙·실험 허브 |
| `[[experiments/training_history]]` | 실험 이력 허브 |

wikilink는 상대경로 사용: `[[research/reward_design]]`

### 세션 종료 전 자동 수행

1. `notes/sessions/session_{YYYY-MM-DD}.md` 생성/업데이트
2. `notes/00_index.md` 현재 상태 업데이트
3. 에러 해결 → `notes/errors/` 기록
4. 실험 시작/완료 → `notes/experiments/` 기록
5. Claude Code memory 업데이트 (project_state.md)

### VM 종료 전 자동 수행 (MANDATORY)

`notes/daily/daily_{YYYY-MM-DD}.md` 생성/업데이트. 양식: `notes/daily/_template.md`

필수 섹션: **오늘 한 일** / **주요 결정 & 발견** / **코드 변경 사항** / **문제 & 해결** / **내일 할 일** / **관련 노트**

---

## Development Environment

개발은 **Docker 컨테이너 내부**에서만. 호스트 VM은 git 작업 전용.

상세 명령: `[[sessions/commands]]`

---

## RL Training (Method A)

> **⚠️ source 순서 필수:** `/root/ros2_ws/install/setup.bash` → `/workspace/ros2_ws/install/setup.bash`
> 순서 틀리면 `px4_msgs` import 에러로 에피소드 노드 silent crash.

**핵심 규칙 (상세: `[[research/rl_rules]]`):**
- 코드 변경 후 dry-run(2–3 에피소드) 먼저, 성공 확인 후 full training
- **보상 공식 변경 → 반드시 Fresh Start** (replay buffer 재사용 금지)
- 첫 롤아웃 후 `env/mean_rew_dist ≠ 0` 확인

상세 명령: `[[sessions/commands]]`

---

## Auto-Logging & Git Sync (MANDATORY)

모든 주요 작업·코드 수정·세션 종료 전 Claude는 **자동으로**:

1. `RL_Project_Log.md` 4-section 구조로 업데이트 (허락 불필요)
2. Git push (`git add . && git commit -m "Auto-sync: [요약]" && git push origin main`)

**RL_Project_Log.md 구조:** `# 1. Current State` / `# 2. Recent Progress` / `# 3. Remaining Tasks` / `# 4. Training History (추가만 가능)`
