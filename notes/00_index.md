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

## 현재 상태 (2026-06-17)

- **알고리즘:** SAC, `net_arch=[256,256]`, L4 GPU
- **현재 학습:** `rl_yolo_v13_terminal_reward` (**iyhfy5ps, fresh 재시작 0→500K, arm_bail=20**)
- **이번 세션 (2026-06-17) — v13 처리량 병목 진단 & 수정:**
  - v13(46y4xtiw) ~10h에 29.9K(6%)뿐, fps≈0.83, ETA ~6.5일. 지배적 싱크 = `PX4 not armed after 10s` bail.
  - **진단(armdiag dry-run, xgzum51v):** 컨트롤러에 `PREFLIGHT-PASS` dt 계측 추가 + `arm_bail_timeout=25s`.
    EKF 재수렴이 **bimodal — 0.0s(7/12) 또는 13–16s(5/12 ≈ 42%)**. 25s 창에서 **bail 0 / SUCCESS 4**.
  - **결론:** v12의 `arm_bail_timeout=10s`가 복구(13–16s) 직전에 멀쩡한 PX4를 단두대질 → full restart 강제.
    stuck-EKF는 *full-restart-only가 아니라 recoverable-with-time*.
  - **Fix:** `hyperparams_v13.yaml` `arm_bail_timeout: 10.0 → 20.0`.
  - **⚠️ 인시던트:** armdiag dry-run이 **YAML 중복 `checkpoint_dir` 키**로 메인 dir에서 실행 → v13 30K 체크포인트 파괴(복구 불가). → fresh 재시작(iyhfy5ps). [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]]
  - **프로덕션 검증(iyhfy5ps):** bail 0, late-EKF 14.1/14.8/15.5s ×3 전부 회복(구 10s면 bail).
  - 상세: [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / [[research/cruise_timeout_arming]] / Rule 11
  - ⚠️ **여전히 OPEN:** YOLO `target_lost_rate` ~29%; teleport 후 EKF 13–16s 재수렴 자체 (별도 과제)

### 이전 세션 (2026-06-16)
- **방식:** Vision 기반 — YOLO X마커 탐지 → TRACKING, RL이 시각 서보잉 학습
- **이번 세션 (2026-06-16) — v12 정체 진단 + v13 종단 보상 재설계:**
  - v12(93K): ep_rew≈-20.5(추세 없음), ep_len 40→16, success **0회**, d_xy~1.0m는 잘 도달
  - 근본 원인: **종단 overshoot 트랩** — 정하방 카메라(commit 24135e9)로 핸드오프가 ~1m가 되면서
    overshoot 가드가 step 1부터 무장 + 8 m/s 액추에이터가 0.5m 성공원을 지나침 → 매 에피소드 -20
  - v11→v12 회귀 (v11은 전방 카메라로 접근 활주로 확보 → 404 successes)
  - **v13 수정 (config-only, Fresh Start 필요):** overshoot threshold 1.5→0.6, success_radius 0.5→0.8,
    action vx/vy 8/5→4/3, w_proximity 0.3→0.6, radius 5→2 → `hyperparams_v13.yaml`
  - 상세: [[research/terminal_overshoot_trap]] / Rule 10
  - ⚠️ **여전히 OPEN:** YOLO `target_lost_rate` ~29% bimodal — 미해결
- **선행 (2026-06-15):** Arming throughput fix — teleport 후 stale EKF arm 거부 수정 → [[research/cruise_timeout_arming]]
- **Phase 1 계획:** CCIP 기반 자율 접근 비행 제어기 → [[research/phase1_plan]]

---

## 실험 현황

| # | Run ID | Steps | 상태 | 비고 |
|---|--------|-------|------|------|
| 001 | 8otphxy8 | 114K | ✅ 완료 | 선형 보상 + CRUISE retry |
| 002 | — | 0 | ✅ 완료 | 보상 패치 적용 (학습 없음) |
| 003 (dry-run) | mtx7ud6o/x8jq9fsy/u8w3xn0w | 5500×3 | ✅ 완료 | RTF 1/2/4 비교 → RTF=2 최적 |
| 004 | esmtny0a | 33K+ | ✅ 폐기 | Vision YOLO 접근 + EKF East 버그 수정 → [[experiments/exp_004_rl_yolo_debug_vision]] |
| 005 | rl_yolo_v12_arm_fix | 0→500K | ⛔ 중단 | Arming fix는 작동하나 종단 overshoot 트랩으로 success 0 → v13으로 대체 → [[research/terminal_overshoot_trap]] |
| 006 | rl_yolo_v13_terminal_reward (46y4xtiw→iyhfy5ps) | 0→500K | 🔄 fresh 재시작 | 종단 보상 재설계. 46y4xtiw 30K에서 처리량 진단 위해 stop → ⚠️ armdiag dry-run이 30K 체크포인트 파괴 → **fresh 재시작 iyhfy5ps (arm_bail=20)** → [[research/terminal_overshoot_trap]] / [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] |
| 006b (dry-run) | v13_armdiag_dryrun (xgzum51v) | 1000 | ✅ 완료 | arm_bail 진단: EKF 재수렴 bimodal(0s/13–16s), 25s에서 bail 0. **Fix: arm_bail_timeout 10→20** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11 |

## 에러 현황

| 파일 | 상태 | 요약 |
|------|------|------|
| [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] | ✅ 해결 | armdiag dry-run이 YAML 중복 키로 v13 30K 체크포인트 파괴. 재발방지: startup `Checkpoints:` 로그로 격리 검증 |
| [[errors/err_20260615_cruise-timeout-arming]] | ✅ 해결 | CRUISE 타임아웃 = teleport 후 PX4 arm 거부 (stale EKF) |
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
- [[research/ekf_east_reversal]] — ⚠️ RETRACTED: "EKF East 반전"은 오진이었음 (실제 PX4 East=+Gazebo East). 정정: [[coordinate-frames]] / [[daily/daily_2026-06-14]]
- [[research/rtf_fps_analysis]] — RTF vs FPS 분석. RTF=2 최적, Python 루프 병목 규명
- [[research/cruise_timeout_arming]] — CRUISE 타임아웃 = teleport 후 EKF 재수렴 bimodal(0s/13–16s). **06-17 정정: v12의 10s 컷이 복구 직전 단두대질 → arm_bail 10→20s** (Rule 11).
- [[research/terminal_overshoot_trap]] — v12 종단 정체 = overshoot 해자 트랩 (정하방 카메라 핸드오프 ~1m). v13 보상 재설계.

### 실험 (experiments/)
- [[experiments/training_history]] — 전체 WandB 학습 히스토리
- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 거리 보상 + CRUISE retry
- [[experiments/exp_002_reward_shaping_patches]] — 보상 패치 Fresh Training (대기 중)
- [[experiments/exp_003_rtf_dryrun]] — RTF 1/2/4 dry-run 비교. RTF=2 최적 확정.
- [[experiments/exp_004_rl_yolo_debug_vision]] — Vision YOLO TRACKING + EKF East 좌표 버그 수정.
- [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] — Arming-rejection throughput fix. v11 분석 + 수정 3종 + dry-run 검증.
- [[experiments/exp_006_xgzum51v_armdiag_dryrun]] — arm_bail 진단. EKF 재수렴 bimodal 계측 → 10s 컷이 진짜 병목. Fix: arm_bail 10→20s.

### 에러 (errors/)
- [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] — armdiag dry-run이 YAML 중복 `checkpoint_dir` 키로 v13 30K 체크포인트 파괴. fresh-start 삭제 footgun + 격리 검증 규칙.
- [[errors/err_20260615_cruise-timeout-arming]] — CRUISE 타임아웃 = teleport 후 PX4 arm 거부 (stale EKF). arm 게이팅 + early-bail.
- [[errors/err_20260320_physics_explosion]] — Gazebo ODE 물리 폭발 3중 방어
- [[errors/err_20260319_ode_aabb_crash]] — 드론 스폰 고도 ODE AABB 크래시

### 연구 일지 (daily/)
- [[daily/daily_2026-06-16]] — v12 정체 진단(종단 overshoot 트랩) + v13 종단 보상 재설계 prep
- [[daily/daily_2026-06-14]] — 06-12 이후 종합: 보상 함수 재설계(v9) + YOLO hold(v9b) + throughput 최적화(v9c·v9d)
- [[daily/daily_2026-06-12]] — EKF East 반전 수정 + proximity trigger 정상화 + rl_yolo fresh start
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
