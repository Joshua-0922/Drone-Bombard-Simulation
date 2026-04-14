---
date: 2026-04-14
tags: [index, dashboard]
status: active
type: index
---

# 드론 정밀 투하 연구 — Obsidian 대시보드

> **프로젝트:** CCIP 기반 잔차 강화학습 활용 드론 정밀 투하
> **스택:** ROS 2 Humble · Docker · PX4 SITL · SAC (SB3) · Gazebo Harmonic · L4 GPU

---

## 현재 상태 (2026-04-14)

- **알고리즘:** SAC, `net_arch=[256,256]`, L4 GPU
- **보상 함수:** 2026-03-22 패치 완료, **Fresh 1M-step 학습 대기 중**
- **마지막 정상 체크포인트:** `sac_drop_preempt.zip` (run `8otphxy8`, ~114K steps)
- **다음 행동:** `ros2 run rl_navigation train_sac` (fresh start, no --resume)

---

## 폴더 구조

| 폴더 | 용도 | 네이밍 규칙 |
|------|------|------------|
| `research/` | 이론·설계·아키텍처 | `{topic_slug}.md` |
| `experiments/` | 학습 실험 (WandB 연동) | `exp_{NNN}_{wandb_id}_{title}.md` |
| `errors/` | 에러 해결 기록 | `err_{YYYYMMDD}_{slug}.md` |
| `sessions/` | 세션별 작업 일지 | `session_{YYYY-MM-DD}.md` |
| `references/` | 논문·문서 메모 | `ref_{slug}.md` |

---

## 노트 인덱스

### 연구 (research/)
- [[research/reward_design]] — 4-layer 보상 함수 (LaTeX 수식)
- [[research/architecture]] — Method A (1-World-4-Payload) 아키텍처
- [[research/system_overview]] — 전체 시스템 (패키지, 토픽, 좌표계, 브리지)
- [[research/rl_rules]] — RL 실험 규칙, WandB 메트릭, Known Failure Modes

### 실험 (experiments/)
- [[experiments/training_history]] — 전체 WandB 학습 히스토리
- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 거리 보상 + CRUISE retry
- [[experiments/exp_002_reward_shaping_patches]] — 보상 패치 Fresh Training (대기 중)

### 에러 (errors/)
- [[errors/err_20260320_physics_explosion]] — Gazebo ODE 물리 폭발 3중 방어
- [[errors/err_20260319_ode_aabb_crash]] — 드론 스폰 고도 ODE AABB 크래시

### 세션 (sessions/)
- [[sessions/session_2026-04-14]] — Obsidian 시스템 초기화 + 파일 간소화
