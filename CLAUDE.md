# CLAUDE.md

---

## 세션 시작 프로토콜

MEMORY.md는 자동 로드됨. 추가 컨텍스트 필요 시 최근 daily 노트 읽기.

**코드 구조를 파악해야 할 때는 grep 전에 지식 그래프부터 조회** (아래 graphify 섹션).

---

## graphify — 코드·노트 지식 그래프 (탐색 우선순위)

`graphify-out/graph.json`에 코드 심볼과 `notes/` 문서가 **하나의 그래프로 연결**되어 있음.
tree-sitter AST 기반이라 조회에 LLM 토큰이 들지 않음.

### 언제 쓰나 — grep보다 먼저

| 상황 | 쓸 명령 |
|------|---------|
| "X는 어디서 어떻게 동작하나" | `graphify query "X" --budget 2000` |
| "X를 고치면 뭐가 깨지나" | `graphify affected "X" --depth 2` |
| "이 심볼 주변 설명" | `graphify explain "X"` |
| "A와 B가 어떻게 이어지나" | `graphify path "A" "B"` |
| "핵심 모듈이 뭔가" | `graphify god-nodes --top 10` |

> **파일명·심볼명을 이미 아는 단일 조회는 그냥 Read/Grep이 빠름.**
> 그래프는 "여러 파일에 걸쳐 흩어진 구조를 훑어야 할 때"만 이득.

조회 결과에는 `src=<파일> loc=<줄번호>`가 붙으므로, 그래프로 **후보를 좁힌 뒤 실제 파일을 Read**하는 순서로 쓸 것.
그래프는 심볼 위치 인덱스이지 코드 내용이 아님 — 그래프만 보고 코드 동작을 단정하지 말 것.

### 그래프 갱신

```bash
graphify update .    # 변경된 코드만 재추출 (LLM 불필요, 무료)
```

- **코드를 수정했으면 갱신할 것.** 안 하면 그래프가 옛 구조를 가리킴 (stale 위험).
- `graphify-out/`은 `.gitignore` 대상 — 커밋하지 말 것. 로컬에서 재생성.
- 제외 경로는 `.graphifyignore`에 있음. **이 파일은 반드시 유지할 것:**
  없이 돌리면 YOLO 데이터셋·비행 영상까지 잡혀 4,789 파일/7.1M 단어가 되고
  (있으면 253 파일/268K 단어), `notes/.obsidian/plugins/dataview/main.js`가
  그래프 상위 허브를 전부 차지함.

### Obsidian vault와의 관계

`notes/`는 사람이 읽는 **연구 기록**(왜 이 값인가, 무엇을 시도했고 왜 실패했나),
graphify 그래프는 기계가 조회하는 **구조 인덱스**(무엇이 무엇을 호출하나). 역할이 다르므로 **둘 다 유지**.

그래프는 코드 심볼과 노트를 이미 교차 연결함 — 예: reset 관련 질의 하나로
`DroneDropEnv._try_soft_reset()` (drone_drop_env.py:1592)와
`notes/research/legacy/reset_throughput_bottleneck.md`가 같이 나옴.
따라서 **"이 repo는 그래프화되어 있다"는 안내용 노트를 vault에 따로 만들지 않음** (내용 없는 포인터 노드 = 관리 부채).

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

### 실험 완료 시 자동 수행 (MANDATORY)

실험(dry-run 포함) 결과가 나오면 **항상 아래 3단계를 함께** 수행:

1. **`experiments/exp_{NNN}_{run_id}_{title}.md`** 생성/업데이트
   - WandB run ID, 설정, 결과 수치 기록
   - `[[research/{finding}]]` wikilink 포함

2. **`research/{finding}.md`** 생성 (새로운 발견이 있을 때)
   - 원인 분석, 적용 규칙, 향후 조건 포함
   - `[[experiments/exp_{NNN}]]` 역링크 포함

3. **허브 노드 3곳 모두 업데이트**
   - `experiments/training_history.md` — 실험 행 추가
   - `research/rl_rules.md` — 새 Rule 추가 (발견이 규칙화될 때)
   - `notes/00_index.md` — 실험 현황 테이블 + 연구 노트 인덱스 갱신

```
실험 완료
    ├── experiments/exp_{NNN}.md  ←→  research/{finding}.md
    │         ↓                              ↓
    ├── training_history.md          rl_rules.md (Rule 추가)
    └──────────────────────────────────────────────────────
                          00_index.md (양쪽 링크)
```

### 세션 종료 전 자동 수행

1. `notes/sessions/session_{YYYY-MM-DD}.md` 생성/업데이트
2. `notes/00_index.md` 현재 상태 업데이트
3. 에러 해결 → `notes/errors/` 기록
4. 실험 시작/완료 → `notes/experiments/` 기록 (위 실험 완료 3단계 포함)
5. Claude Code memory 업데이트 (project_state.md)

### VM 종료 전 자동 수행 (MANDATORY)

`notes/daily/daily_{YYYY-MM-DD}.md` 생성/업데이트. 양식: `notes/daily/_template.md`

필수 섹션: **오늘 한 일** / **주요 결정 & 발견** / **코드 변경 사항** / **문제 & 해결** / **내일 할 일** / **관련 노트**

---

## Development Environment (Isaac Lab, 2026-09 기준)

- 코드는 repo `isaac_lab/`에서 편집하고, **실행은 컨테이너 `isaac-verify`의 `/tmp/rebuild`** 에서 한다. 편집 후 반드시 `docker cp isaac_lab/. isaac-verify:/tmp/rebuild/`.
- 파이썬은 `/workspace/isaaclab/isaaclab.sh -p <script>`. 호스트 python3에는 numpy/torch가 없다.
- 학습 체크포인트·덤프·평가 JSON은 전부 컨테이너 `/tmp` (컨테이너 stop에는 살아남고, 삭제에는 안 남는다).
- wandb 키는 호스트 `/opt/drone-bombard/.wandb.env` → `docker exec --env-file`.

상세 명령: `[[sessions/commands]]` · 파일 역할: `[[research/isaac_lab_architecture]]` · 코드 위치: `[[research/code_map]]`

---

## RL Training & Residual SL

**본 방법 = L0(PPO 비행 정책, 동결) + GRU-S 잔차(지도학습) + EMA α=0.3.** 파이프라인: `[[research/l1_sl_pipeline]]`.

**핵심 규칙 (상세: `[[research/rl_rules]]`):**
- 코드 변경 후 단위테스트(`tests/`) → dry-run(소수 iteration·에피소드) → 본 실행
- L0 평가에는 `--no_residual` 필수(기본값이 잔차 활성) — 없으면 92.7%가 37.5%로 보인다
- 팔 비교는 항상 paired eval, seed {3000,4000,5000} × 200, `_agg_table1.py`의 시드셋 assert 통과
- 실험 하나 = 노트 하나(`experiments/exp_NNN`), 허브 3곳 갱신, 결과 JSON 경로 기록

---

## Auto-Logging & Git Sync (MANDATORY)

모든 주요 작업·코드 수정·세션 종료 전 Claude는 **자동으로**:

1. `RL_Project_Log.md` 4-section 구조로 업데이트 (허락 불필요)
2. Git push (`git add . && git commit -m "Auto-sync: [요약]" && git push origin main`)

**RL_Project_Log.md 구조:** `# 1. Current State` / `# 2. Recent Progress` / `# 3. Remaining Tasks` / `# 4. Training History (추가만 가능)`

## Korean Usage

한국어를 사용할 때 과다한 구어체나 너무 줄여서 말하는 것은 자제하기. 
