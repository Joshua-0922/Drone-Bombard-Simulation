---
date: 2026-04-14
tags: [index, dashboard]
status: active
type: index
---

# 드론 정밀 투하 연구 — Obsidian 대시보드

> **프로젝트:** CCIP 기반 잔차 강화학습 활용 드론 정밀 투하
> **스택:** ROS 2 Humble · Docker · PX4 SITL · SAC (SB3) · Gazebo Harmonic · L4 GPU
>
> **개인 대시보드:** [[00_index_junsang]]

---

## 현재 상태 (2026-04-23)

- **알고리즘:** SAC, `net_arch=[256,256]`, L4 GPU
- **보상 함수:** 2026-03-22 패치 완료, **Fresh 1M-step 학습 대기 중**
- **RTF:** **2** (dry-run 실험으로 RTF=2 최적 확정, avg 59.5 fps)
- **마지막 정상 체크포인트:** `sac_drop_preempt.zip` (run `8otphxy8`, ~114K steps)
- **VM:** Spot VM 전환 완료 (IP: `130.211.241.166`) — startup.sh + watchdog CF + Scheduler 배포 완료
- **자동화:** 선점 → 5분 내 자동 재시작 파이프라인 완성 (무인 야간 학습 가능)
- **Phase 1 계획:** CCIP 기반 자율 접근 비행 제어기 → [[research/phase1_plan]]
- **다음 행동:** hyperparams_rtf2.yaml 수정 → colcon build 확인 → Exp 004-dryrun → Exp 005a 야간 (Spot VM 무인 실행)

---

## 실험 현황

| # | Run ID | Steps | 상태 | 비고 |
|---|--------|-------|------|------|
| 001 | 8otphxy8 | 114K | ✅ 완료 | 선형 보상 + CRUISE retry |
| 002 | — | 0 | ⏳ 대기 | 보상 패치 Fresh Start 필요, RTF=2 |
| 003 (dry-run) | mtx7ud6o/x8jq9fsy/u8w3xn0w | 5500×3 | ✅ 완료 | RTF 1/2/4 비교 → RTF=2 최적 |

## 에러 현황

| 파일 | 상태 | 요약 |
|------|------|------|
| [[errors/err_20260320_physics_explosion]] | ✅ 해결 | ODE 물리 폭발 3중 방어 |
| [[errors/err_20260319_ode_aabb_crash]] | ✅ 해결 | 드론 스폰 고도 ODE AABB 크래시 |

---

## 빠른 참조

| 주제 | 파일 |
|------|------|
| Guacamole/VNC 설정·접속 | `notes/Environment/README.md` |
| 자주 쓰는 명령어 | `notes/sessions/commands.md` |
| RL 규칙 | `notes/research/rl_rules.md` |
| VM 완전 복구 가이드 | `notes/Environment/README.md` |
| 보상 함수 설계 | `notes/research/reward_design.md` |
| 전체 시스템 아키텍처 | `notes/research/system_overview.md` |

---

## 폴더 구조

| 폴더 | 용도 | 네이밍 규칙 |
|------|------|------------|
| `research/` | 이론·설계·아키텍처 | `{topic_slug}.md` |
| `experiments/` | 학습 실험 (WandB 연동) | `exp_{NNN}_{wandb_id}_{title}.md` |
| `errors/` | 에러 해결 기록 | `err_{YYYYMMDD}_{slug}.md` |
| `sessions/` | 세션별 작업 일지 | `session_{YYYY-MM-DD}.md` |
| `references/` | 논문·문서 메모 | `ref_{slug}.md` |
| `Environment/` | 인프라 설정 백업 | `{service}.{ext}` |

---

## 노트 인덱스

### 연구 (research/)
- [[research/phase1_plan]] — Phase 1 CCIP 기반 자율 접근 연구 계획 (8주, 5/8-6/30)
- [[research/reward_design]] — 4-layer 보상 함수 (LaTeX 수식)
- [[research/architecture]] — Method A (1-World-4-Payload) 아키텍처
- [[research/system_overview]] — 전체 시스템 (패키지, 토픽, 좌표계, 브리지)
- [[research/rl_rules]] — RL 실험 규칙, WandB 메트릭, Known Failure Modes
- [[research/rtf_fps_analysis]] — RTF vs FPS 분석. RTF=2 최적, Python 루프 병목 규명

### 실험 (experiments/)
- [[experiments/training_history]] — 전체 WandB 학습 히스토리
- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 거리 보상 + CRUISE retry
- [[experiments/exp_002_reward_shaping_patches]] — 보상 패치 Fresh Training (대기 중)
- [[experiments/exp_003_rtf_dryrun]] — RTF 1/2/4 dry-run 비교. RTF=2 최적 확정.

### 에러 (errors/)
- [[errors/err_20260320_physics_explosion]] — Gazebo ODE 물리 폭발 3중 방어
- [[errors/err_20260319_ode_aabb_crash]] — 드론 스폰 고도 ODE AABB 크래시

### 연구 일지 (daily/)
- [[daily/daily_2026-04-23]] — Spot VM 이전 완료 (startup.sh + watchdog CF + create_spot_vm.sh) + IP 변경 대응
- [[daily/daily_2026-04-17]] — Phase 1 코드 전체 구현 (변경 1-10, obs 15→17, CCIP auto-drop)
- [[daily/daily_2026-04-16]] — WandB 연결 + RTF dry-run 실험 + 인프라 고장 해결
- [[daily/daily_2026-04-14]] — Guacamole HTTPS + Obsidian 설치 + wikilink 정비

### 세션 (sessions/)
- [[sessions/session_2026-04-16]] — RTF dry-run, docker commit, airframe 수정
- [[sessions/session_2026-04-14]] — Obsidian 시스템 초기화 + 파일 간소화

### 환경 설정 (Environment/)
- [[Environment/README]] — VM 완전 복구 가이드
- [[Environment/docker-compose.yml]] — guacd + guacamole + postgres + nginx
- [[Environment/nginx.conf]] — WebSocket 리버스 프록시 설정
- [[Environment/vncserver.service]] — TigerVNC systemd 유닛
