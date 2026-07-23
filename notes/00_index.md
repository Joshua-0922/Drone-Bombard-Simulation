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
| **L4 VM 접속 가이드(팀원용)·STOCKOUT 재시도** | `notes/Environment/vm_access_guide_junsang.md` |
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
- ⭐ [[research/isaac_model_intro_junsang]] — [Isaac] **모델 소개**: 한 에피소드 흐름(스폰→랜덤표적·바람→순항→인지→탄도보정→하강 투하→물리 낙하 명중)으로 보는 온보딩 (+각 단계 어떻게 해냈나·코드 포인터)
- [[research/phase1_plan]] — Phase 1 CCIP 기반 자율 접근 연구 계획 (8주, 5/8-6/30)
- [[research/reward_design]] — 4-layer 보상 함수 (LaTeX 수식)
- [[research/architecture]] — Method A (1-World-4-Payload) 아키텍처
- [[research/system_overview]] — 전체 시스템 (패키지, 토픽, 좌표계, 브리지)
- [[research/rl_rules]] — RL 실험 규칙, WandB 메트릭, Known Failure Modes
- [[research/rtf_fps_analysis]] — RTF vs FPS 분석. RTF=2 최적, Python 루프 병목 규명
- [[research/isaac_cruise_handoff_junsang]] — [Isaac] cruise 핸드오프: reset 시 컨트롤러 setpoint seed (Rule 10)
- [[research/isaac_v11_v13_design_guide_junsang]] — [Isaac] v11~v13 설계 가이드: 기존 migration 모델 대비 변경점
- [[research/isaac_expansion_roadmap_junsang]] — [Isaac] 확장 로드맵: CCIP residual+DR 다음단계 개요 + 남은 축 전체
- [[research/isaac_v18_curriculum_continuation_junsang]] — [Isaac] v18 커리큘럼 백업 & Phase 3+ 재개 가이드 (체크포인트 경로·재개 명령)
- [[research/isaac_model_spec_junsang]] — ⭐ [Isaac] 모델 스펙: 전체 파라미터 한눈에 (현재 모델·cfg 값·버전별 차이)
- [[research/isaac_viz_tools_junsang]] — [Isaac] 시각화/검증 도구 (play.py 모드·마커·라이브스트림·시행착오)
- [[research/isaac_v19_collapse_nodrop_junsang]] — [Isaac] v19 no-drop 붕괴 진단(상주 CCIP 보상) + A(포텐셜)·B(loiter)·D(best저장) 처방

### 실험 (experiments/)
- [[experiments/training_history]] — 전체 WandB 학습 히스토리
- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 거리 보상 + CRUISE retry
- [[experiments/exp_002_reward_shaping_patches]] — 보상 패치 Fresh Training (대기 중)
- [[experiments/exp_003_rtf_dryrun]] — RTF 1/2/4 dry-run 비교. RTF=2 최적 확정.
- [[experiments/exp_006_v11_dryrun_junsang]] — [Isaac] v11 완화 테스트 dry-run 성공 (success 100%, 착탄 ~0.75m)
- [[experiments/exp_007_v12_random_marker_junsang]] — [Isaac] v12 첫 확장: 랜덤 marker 일반화 성공 (success 100%)
- [[experiments/exp_008_v13_partial_obs_junsang]] — [Isaac] v13 부분관측: blind 순항→7m 탐지 학습 성공 (success 100%)
- [[experiments/exp_009_v14_ccip_residual_junsang]] — [Isaac] v14 DR+CCIP residual (Stage A): residual ON>OFF, 대조군
- [[experiments/exp_010_v15_airframe_wind_junsang]] — [Isaac] v15 바람이 기체 작용 (wind-test 검증, residual 포화 발견→wind 2.0)
- [[experiments/exp_011_v16_physical_drop_junsang]] — [Isaac] v16 실제 물리 payload drop (drop-test PASS, success 0.80)
- [[experiments/exp_012_v17_pixel_vision_junsang]] — [Isaac] v17 픽셀 양자화 vision (대략 위치→접근하며 정밀, success ~0.8)
- [[experiments/exp_013_v18_integration_curriculum_junsang]] — [Isaac] v18 능력 통합 + 커리큘럼 (데드락 발견→warm-start 해결, success 1.0)
- [[experiments/exp_014_v19_full_integration_junsang]] — [Isaac] v19 전체 통합 + 실제 물리 drop (success 1.0, 실제 착탄 0.56m)
- [[experiments/exp_015_v19_abd_retrain_junsang]] — [Isaac] v19 no-drop 붕괴 수정(A 포텐셜 shaping+B loiter+D best저장) 재학습: 붕괴 없이 success 76.7%·착탄 0.563m
- [[experiments/exp_016_v19_precision_landing_junsang]] — [Isaac] v19 정밀도 향상(연속 착지보상): success 100%·착탄 0.356m (0.563→0.356, 37%↓)

### 에러 (errors/)
- [[errors/err_20260320_physics_explosion]] — Gazebo ODE 물리 폭발 3중 방어
- [[errors/err_20260319_ode_aabb_crash]] — 드론 스폰 고도 ODE AABB 크래시

### 연구 일지 (daily/)
- [[daily/daily_2026-07-16_junsang]] — [Isaac] v14 DR+residual → v15 기체바람 → v16 실제 물리 drop 완료
- [[daily/daily_2026-07-15_junsang]] — [Isaac] v11 완화 → v12 랜덤 → v13 부분관측 전부 100%, 라이브 GUI, 디스크 진단
- [[daily/daily_2026-04-23]] — Spot VM 이전 완료 (startup.sh + watchdog CF + create_spot_vm.sh) + IP 변경 대응
- [[daily/daily_2026-04-17]] — Phase 1 코드 전체 구현 (변경 1-10, obs 15→17, CCIP auto-drop)
- [[daily/daily_2026-04-16]] — WandB 연결 + RTF dry-run 실험 + 인프라 고장 해결
- [[daily/daily_2026-04-14]] — Guacamole HTTPS + Obsidian 설치 + wikilink 정비

### 세션 (sessions/)
- [[sessions/session_2026-07-16_junsang]] — [Isaac] v14/v15/v16 세션 (DR+residual→기체바람→물리 drop)
- [[sessions/session_2026-07-15_junsang]] — [Isaac] v11/v12/v13 확장 세션 (완화→랜덤→부분관측, 전부 100%)
- [[sessions/session_2026-04-16]] — RTF dry-run, docker commit, airframe 수정
- [[sessions/session_2026-04-14]] — Obsidian 시스템 초기화 + 파일 간소화

### 환경 설정 (Environment/)
- [[Environment/README]] — VM 완전 복구 가이드
- [[Environment/docker-compose.yml]] — guacd + guacamole + postgres + nginx
- [[Environment/nginx.conf]] — WebSocket 리버스 프록시 설정
- [[Environment/vncserver.service]] — TigerVNC systemd 유닛
