---
date: 2026-04-14
tags: [experiment, history, wandb, SAC]
status: active
type: reference
---

# 전체 학습 히스토리 (RL_Project_Log.md에서 이전)

> **Append-only.** 새 run은 아래 테이블에 추가하고 개별 `exp_NNN_*.md` 파일도 생성.

---

| 날짜 | Run ID | Steps | Mean Drop Error | 요약 |
|------|--------|-------|-----------------|------|
| 2026-03-12 | vekkz83a | 386 | — | 첫 run 중단; 0 FPS (60 s/episode RTF 고정). Phase 7 최적화 적용. |
| 2026-03-13 | vekkz83a | resumed | — | preempt + replay buffer 재개. RTF=0, headless. |
| 2026-03-17 | apax52d7 | ~3K | — | dartsim 크래시 수정, EKF yaw fix (COM_ARM_WO_GPS=1). ~25 steps/sec. |
| 2026-03-17 | 27mbu6qk | 49,242 | — | replay buffer 비호환 수정 후 재개. fps 2→4. ep_rew=-367. |
| 2026-03-18 | 9nbwg71r | 53,428 | — | Phase 12: OFFBOARD retry race 수정. fps 4→23. |
| 2026-03-18 | 2h1cvmer | 78,200 | — | Phase 13 초기: multi-instance 수정, stale shm, NaN obs 크래시. |
| 2026-03-18 | 53xx3o8u | 80,292+ | — | Phase 13 최종: PX4_GZ_MODEL_POSE 수정, EKF warmup 5s, RTF=1. **fps=12 안정.** |
| 2026-03-18 | dy97unuj | 546K–560K | — | **불량 run** — px4_msgs source 누락, 모든 에피소드 IDLE 고착. 종료. |
| 2026-03-18 | izf10080 | 564K–568K | — | **오염 버퍼 run** — ep_rew=-1,900. 종료. |
| 2026-03-18 | pbpqa0rp | 0 (fresh) | — | **Phase 1 Fresh Start.** manual drop 비활성, WandB callback 추가. ep_rew=-91 @6K. |
| 2026-03-19 | a9f6lk57 | — | — | **Debug run** — 자기관리 인프라 첫 시도. z=5 스폰 → ODE crash. 종료. |
| 2026-03-19 | 6dopfyjn | ~96K–102K | — | **Debug run** — world reset 제거. COM_OF_LOSS_T race 수정. |
| 2026-03-19 | nynxn6b5 | 102K+ | — | **자기관리 인프라 안정.** z=0 스폰, COM_OF_LOSS_T=10s. **fps=30-31.** |
| 2026-03-20 | ljbn3wfg | 8K (dry-run) | — | **Method A dry-run.** PX4_SIM_MODEL=gz_x500_bombard_r0, px4_msgs fix. **31 fps, 16 에피소드, 0 ODE.** |
| 2026-03-20 | cj3ytvq2 | 20K | — | **Method A 생산 학습 FRESH START.** num_envs=1, RTF=1, 33 fps. ep_rew=−545 @20K. |
| 2026-03-20 | mjfet61f | 52K | — | WandB 보상 모니터링 추가. **KILLED** — d_xy=1.98e11 물리 폭발. 버퍼 오염. |
| 2026-03-20 | 53samoqz | 95K | — | 폭발 복구. physics explosion guard 추가. **KILLED** — WandB 콜백 글리치 값 누산 버그. |
| 2026-03-20 | naf4zyhm | 104K | — | 3중 폭발 방어 적용. **KILLED** — `mean_rew_dist=0` (지수 포텐셜 포화 k1=1.0). |
| 2026-03-20 | 8otphxy8 | 114K | — | **선형 거리 보상 + CRUISE retry.** 마지막 정상 베이스라인. → [[exp_001_8otphxy8_linear_reward]] |
| 2026-03-22 | — | — | — | **보상 패치 적용 (학습 없음).** anti-milking, w_time 5×, truncation penalty. Fresh start 필요. → [[exp_002_reward_shaping_patches]] |
