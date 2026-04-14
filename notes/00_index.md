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

## 폴더 구조

| 폴더 | 용도 | 네이밍 규칙 |
|------|------|------------|
| `research/` | 이론·설계·아키텍처 노트 | `{topic_slug}.md` |
| `experiments/` | 학습 실험 기록 (WandB 연동) | `exp_{NNN}_{wandb_id}_{title}.md` |
| `errors/` | 에러 해결 기록 | `err_{YYYYMMDD}_{slug}.md` |
| `sessions/` | 세션별 작업 일지 | `session_{YYYY-MM-DD}.md` |
| `references/` | 논문·문서 메모 | `ref_{slug}.md` |

---

## 현재 상태 (2026-04-14)

- **알고리즘:** SAC, `net_arch=[256,256]`, L4 GPU
- **최신 보상 함수:** [[research/reward_design]] (2026-03-22 패치 후 미학습)
- **대기 중인 작업:** 보상 패치 적용 후 Fresh 1M-step 학습 시작
- **마지막 정상 체크포인트:** `sac_drop_preempt.zip` (run `8otphxy8`)

---

## 노트 인덱스

### 연구
- [[research/reward_design]] — 4-layer 계층형 보상 함수 설계
- [[research/architecture]] — Method A (1-World-4-Payload) 아키텍처

### 실험
- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 거리 보상 + CRUISE retry
- [[experiments/exp_002_reward_shaping_patches]] — 안티-밀킹 패치 (미학습)

### 에러
- [[errors/err_20260320_physics_explosion]] — Gazebo ODE 폭발 방어 3중 레이어
- [[errors/err_20260319_ode_aabb_crash]] — 드론 스폰 고도 ODE AABB 크래시

### 세션
- [[sessions/session_2026-04-14]] — 오늘 세션 (Obsidian 시스템 초기화)
