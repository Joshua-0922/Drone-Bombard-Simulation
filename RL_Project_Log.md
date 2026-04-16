# RL Training Pipeline — Project Log

> **Branch:** `main` | **VM 프리엠션 후 컨텍스트 복구용 로그**

---

# 1. Current State

**업데이트:** 2026-04-16

### 활성 보상 공식 (2026-03-22 패치, 미적용)

| Layer | 공식 | 파라미터 |
|-------|------|---------|
| R1 Safety | `−10` if alt < 2 m; `−8` if speed > 20 m/s | step > 20 이후 |
| R2 Stability | `−0.05 − 0.05‖ω‖² − 0.05‖Δa‖²` | w_time=0.05 (패치됨) |
| R3 Distance | `1.0 × (d_prev − d_xy)` | 선형, 포화 없음 |
| R3 Orient | `1.0 × cos θ × min(v_xy/2, 1)` | speed gate (안티-밀킹 패치) |
| R4 Drop | `50·exp(−5·d_err) [+100 jackpot if d≤0.1m]` | auto-drop at d_xy≤0.5m |
| Truncation | `−50` if step=500 and not dropped | 패치됨 |

### 학습 환경

| 파라미터 | 값 |
|---------|-----|
| Algorithm | SAC, `net_arch=[256,256]`, `device=cuda` |
| `num_envs` | 1 (Gazebo lockstep 병목으로 고정) |
| `total_timesteps` | 1,000,000 |
| RTF | **2** (RTF dry-run 결과: RTF=2 최적, avg 59.5 fps) |
| 예상 fps | **~60** |

### 체크포인트

- **보상 패치 전 마지막 정상:** `sac_drop_preempt.zip` (run `8otphxy8`, ~114K steps)
- **다음 학습:** Fresh start 필요 (보상 공식 변경으로 재개 금지)

---

# 2. Recent Progress

- **2026-04-16:** RTF dry-run 3종 완료 (RTF 1/2/4). **RTF=2 최적** (avg 59.5 fps, 61s/4Kstep). RTF=4는 Python 병목으로 역전. Exp 002 RTF=2로 결정.
- **2026-04-16:** WandB API key 영구 연결 (`/opt/drone-bombard/.wandb.env`, `--env-file` 방식). Docker image `drone-bombard-px4built:latest` — PX4 빌드 + 커스텀 airframes 4016-4019 포함.
- **2026-04-14:** Obsidian 연구 비서 시스템 초기화. `notes/` 구조 구축, CLAUDE.md + RL_Project_Log.md 간소화.
- **2026-03-22:** 보상 해킹 분석 → 4개 anti-milking 패치 적용 (학습 대기 중).
- **2026-03-20:** 선형 거리 보상 도입 (지수 포텐셜 교체), CRUISE retry, 3중 물리 폭발 방어.
- **2026-03-20:** Method A (1-World-4-Payload) 아키텍처 완성 및 dry-run 통과 (31 fps).
- **2026-03-19:** 자기관리 인프라 안정화 (z=0 스폰, COM_OF_LOSS_T=10s, fps=30-31).

---

# 3. Remaining Tasks (Next Steps)

- [ ] **Fresh 1M-step 학습 시작** — 보상 패치 이후 fresh start (run `exp_002`), **RTF=2**로 진행
- [x] **RTF > 1 조사** — dry-run 완료. RTF=2 최적 확정.
- [ ] **EvalCallback 추가** — 10K steps마다 deterministic 평가, `best_model.zip` 저장
- [ ] **커스텀 SB3 policy** — PyTorch AMP (mixed precision)으로 L4 GPU 활용도 향상
- [ ] **Phase 2 커리큘럼** — policy가 표적 도달 안정화 후 manual drop 재활성화
- [ ] **PX4 로그 /dev/null 리다이렉트** — `/tmp/px4_{i}.log` 100+ MB 증가 방지

---

# 4. Training History

> **전체 히스토리:** `notes/experiments/training_history.md`
> **개별 실험 노트:** `notes/experiments/exp_NNN_*.md`

최근 주요 runs:

| 날짜 | Run ID | Steps | 요약 |
|------|--------|-------|------|
| 2026-03-20 | 8otphxy8 | 114K | 선형 거리 보상 + CRUISE retry. 마지막 정상 베이스라인. |
| 2026-03-22 | — | — | 보상 패치 적용 (학습 없음). Fresh start 대기 중. |
| 2026-04-16 | mtx7ud6o/x8jq9fsy/u8w3xn0w | 5500×3 | RTF 1/2/4 dry-run. RTF=2 최적 (59.5 fps). |
| _TBD_ | exp_002 | — | 보상 패치 후 Fresh 1M-step 학습, RTF=2 |
