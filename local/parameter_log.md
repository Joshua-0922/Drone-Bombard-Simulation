# Parameter Log — Drone-Bombard-Simulation

> **목적**: 모든 parameter 의 의미 사전 (§1) + 시간순 변경/추가/제거 history (§3 이후).
> "이 parameter 가 뭐 하는 거지?" → §1 / "왜 언제 바뀌었지?" → §3 이후.
>
> **추가 규칙**: parameter 가 변경/추가/제거되거나 새 run 시작 시 §3 표에 한 줄 + §4 entry 한 블록 추가.

작성 시작: 2026-05-22
관련:
- [meeting_notes/meeting_notes_2026-05-20.txt](meeting_notes/meeting_notes_2026-05-20.txt)
- [meeting_notes/meeting_notes_2026-05-22.txt](meeting_notes/meeting_notes_2026-05-22.txt)
- [meeting_notes/meeting_notes_2026-05-23.txt](meeting_notes/meeting_notes_2026-05-23.txt)
- [design_review_2026-05-23.md](design_review_2026-05-23.md)
- [archive/change_inventory_pre_pull_2026-05-22.md](archive/change_inventory_pre_pull_2026-05-22.md)
- [../Drone-Bombard-Simulation/RL_analysis.md](../Drone-Bombard-Simulation/RL_analysis.md)

---

## 1. Parameter 의미 사전 (Glossary)

`hyperparams.yaml` 의 각 key 의미 + 영향 + trade-off.

### 1-1. `training:` — 학습 메타

| Key | 의미 | 영향 / Trade-off |
|---|---|---|
| `total_timesteps` | 학습 총 env step 수 (`env.step()` 호출 총 횟수) | 클수록 학습 길어짐. SAC 는 `gradient_steps` 배수만큼 network update 함 |
| `checkpoint_freq` | N step 마다 model `.zip` 저장 (CheckpointCallback) | 작을수록 보존 잦지만 디스크 사용↑ |
| `max_checkpoints_kept` | rolling 으로 마지막 N 개만 보관, 옛것 삭제 | 디스크 보호 |
| `checkpoint_dir` | 체크포인트/best_model/archive 폴더 root | — |
| `num_envs` | 병렬 환경 수 (SubprocVecEnv) | >1 이면 Gazebo lockstep overhead. 현재 1 권장 |
| `env_stagger_secs` | SubprocVecEnv instance 초기화 간격 (s) | num_envs>1 시 CPU spike 방지 |
| `eval_freq` | BestModelCallback 의 평가 주기 (env step) | smoke/본학습 분리 가능. 작을수록 best 갱신 잦음 |
| `eval_episodes` | episodes per eval (reserved for future EvalCallback) | 현재 BestModelCallback 은 rollout 의 ep_rew_mean 사용 |

### 1-2. `sac:` — Stable-Baselines3 SAC 알고리즘

| Key | 의미 | 영향 / Trade-off |
|---|---|---|
| `learning_rate` | actor/critic Adam optimizer LR | 표준 3e-4. 크면 발산, 작으면 학습 느림 |
| `buffer_size` | replay buffer 최대 크기 (transitions) | 작으면 옛 경험 사라져 catastrophic forgetting. 너무 크면 RAM↑ |
| `batch_size` | gradient step 당 mini-batch 크기 | 표준 256. 크면 안정성↑ but throughput↓ |
| `tau` | target network soft update rate (`τ·new + (1−τ)·target`) | 0.005 표준. 작을수록 안정 but 학습 느림 |
| `gamma` | discount factor. effective horizon ≈ `1/(1−γ)` step | 0.99 → ~100 step, 0.995 → ~200 step, 0.999 → ~1000 step. 크면 long-term credit 강화 but Q variance↑ |
| `learning_starts` | 이 step 까지 random action (gradient update 안 함) | 초기 replay 다양성 확보. 너무 짧으면 critic 발산 |
| `gradient_steps` | env step 당 network update 횟수 (UTD ratio) | 1=환경 step 마다 1 update. 4=4 update. 클수록 sample reuse↑ but critic 폭주 위험 |
| `net_arch` | actor/critic hidden layer 크기 list | [256,256] 표준. 크면 표현력↑ but 데이터 더 필요 |
| `device` | "cuda" / "cpu" | GPU 가속 |

### 1-3. `environment:` — env / 좌표 / 정규화

| Key | 의미 | 영향 / Trade-off |
|---|---|---|
| `target_enu_x`, `target_enu_y` | 타겟 위치 (ENU 좌표, m) | 학습 / 평가의 목표점 |
| `pos_scale` | position observation 정규화 scale (m) — `obs = pos / 50` | obs ∈ [-1,1] 유지. 큰 비행 영역 학습 시 늘림 |
| `vel_scale` | velocity obs 정규화 (m/s) | |
| `ang_vel_scale` | angular velocity obs 정규화 | |
| `action_vx_scale` | action[0] (-1~1) → vx 명령 변환 (m/s) | 크면 빠른 maneuver but 불안정 |
| `action_vy_scale`, `action_vz_scale` | y, z 방향 동일 | |
| `action_yaw_scale` | action[3] → yaw rate (rad/s) | 1.0 = ~57°/s. 너무 작으면 회전 느림 |
| `max_steps` | episode 최대 step (truncation 기준) | 길수록 episode 길어짐. 500 = ~25s at 20Hz |
| `min_altitude` | crash penalty 임계 고도 (m) | 너무 높으면 takeoff 어려움 |
| `min_altitude_start_step` | 이 step 이후부터 min_altitude 적용 | takeoff 유예 (보통 20 step ≈ 1초) |
| `obs_wait_timeout` | PX4 message 대기 timeout (s) | 작을수록 빨리 step 진행. PX4_SIM_SPEED_FACTOR 와 곱해서 wall-clock 결정 |
| `cruise_poll_timeout` | CRUISE 도달 대기 timeout (s) | 너무 짧으면 false negative |
| `use_vision` | 카메라 obs 사용 여부 (true/false) | RL 학습은 false (camera 없는 world). vision 미션은 true |

### 1-4. `reward:` — 4-Layer 보상 구조

**Layer 1 (안전, 매 step)**
| Key | 의미 |
|---|---|
| `penalty_crash` | altitude < min_altitude (after start_step) 시 negative reward |
| `penalty_overspeed` | speed > 20 m/s 시 negative reward |
| `penalty_target_lost` | vision confidence == 0 시 (use_vision 일 때만) |

**Layer 2 (안정/효율, 매 step)**
| Key | 의미 |
|---|---|
| `w_time` | per-step time penalty (시간 흐름 자체에 벌점) |
| `w_ang_vel` | `‖ω‖²` 제곱 penalty weight (기체 불안정도) |
| `w_action_smooth` | `‖a_t − a_{t-1}‖²` penalty (급격 조작 억제) |

**Layer 3 (접근, 매 step)** — `R3 = w_dist·(d_prev−d_now) + w_heading·cos(θ)·gate + w_impact·exp(−k_impact·d_impact)`
| Key | 의미 |
|---|---|
| `w_dist` | 타겟까지 거리 감소량 보상 weight (potential-based, 안정) |
| `w_heading` | 진행 방향과 타겟 방향 정렬 cos 보상 |
| `speed_gate_enabled` | `gate(v_xy) = min(v_xy/2, 1)` — 정지 상태에서 heading/impact reward 수집 차단 (anti-milking) |
| `w_impact` | CCIP 예측 d_impact 의 exp 보상 weight. **non-potential-based** → speed_gate 없으면 loitering hack 위험 |
| `k_impact` | r3_impact decay constant. 작을수록 멀리서도 신호 살림 (k=0.05 면 80m 에서 ~0.087, k=0.1 면 ~0.0017) |
| `k1_potential` | **UNUSED** — 옛 exp(-k1·d) potential 잔재 |

**Layer 4 (terminal, drop 시점)** — `R4 = drop_attempt_bonus + w_drop_base·exp(−k2·d_error) + r_success_jackpot·[d_error≤0.1] − penalty_instability`
| Key | 의미 |
|---|---|
| `auto_drop_threshold` | CCIP d_impact ≤ 이 값이면 자동 drop. 작을수록 정확 but drop 경험 빈도↓ |
| `drop_wait_timeout` | drop 후 actual landing 결과 대기 (s) |
| `drop_attempt_bonus` | drop 시도 자체에 부여 (jekyun v2 신규) — sparse → dense |
| `w_drop_base` | drop accuracy reward base weight. exp(-k2·d_error) 와 곱해짐 |
| `k2_precision` | drop precision decay. 작을수록 먼 거리도 의미있는 보상 (k2=5 면 2m drop ≈ 0, k2=0.3 면 2m drop ≈ 0.55) |
| `r_success_jackpot` | success_threshold 안 drop 시 추가 bonus |
| `success_threshold` | jackpot 거리 임계 (m) |
| `penalty_instability` | drop 시 |ω|>limit_ang_vel 또는 tilt>limit_tilt 면 negative |
| `limit_ang_vel`, `limit_tilt` | instability 판정 임계 |
| `truncation_penalty` | max_steps 초과 (timeout) 시 penalty. 너무 크면 "drop 안 하느니 빨리 죽기" |
| `g` | 중력 (m/s²). CCIP 계산용 |

### 1-5. `wandb:` — 로깅

| Key | 의미 |
|---|---|
| `project` | wandb project 명 |
| `entity` | wandb team/org slug |
| `run_name` | run 표시명 (null = wandb 자동 생성) |
| `tags` | run tag list (검색용) |
| `save_model_artifact` | 최종 모델을 wandb artifact 로 업로드 |
| `log_freq` | WandbCallback gradient_save_freq (작을수록 로그 잦음, training loop block 위험) |
| `mode` | "online" / "offline". offline 은 별도 sync 스크립트 필요 |

### 1-6. 코드 측 callback (yaml 외)

| Key | 의미 |
|---|---|
| `env/mean_rew_impact` | rollout 평균 r3_impact (jekyun_v2 에는 없음, junsang 추가) |
| `env/mean_rew_dist/orient/ctrl/drop` | rollout 평균 각 layer reward |
| `env/drop_count` | per-rollout drop 수 (drop 없는 rollout 은 로깅 안 됨) |
| `env/drop_error_actual_m` | rollout 평균 실제 drop 오차 (m) |
| `env/mean_d_xy`, `mean_d_impact` | rollout 평균 거리 |
| **[2026-05-23 entry #13 이후 — 누적 count metric]** | (x축은 `env/total_episodes` 로 wandb 자동 매핑) |
| `env/total_episodes` | universal episode counter (x축 base) |
| `env/total_drop_terminated_count` | drop 발동 누적 (Layer 4) — 옛 `env/total_drop_count` 이름 변경 |
| `env/total_success_count` | d_error ≤ 0.5m 누적 |
| `env/total_jackpot_count` | d_error ≤ 0.1m 누적 (success_threshold) |
| `env/total_truncate_<reason>_count` | crash/overspeed/ang_vel/inverted/timeout 5종 누적 |
| `env/total_safety_violation_count` | crash + overspeed 합산 누적 |
| `env/total_physics_glitch_count` | physics 이상 누적 (NaN 등) — 옛 `physics_glitch_count` 이름 변경 |
| **[entry #13 에서 제거됨]** | env/success_rate, env/safety_violation_rate, env/truncate_*_rate, env/drop_terminated_rate (rate 8개) — 누적 count 로 대체 |

---

## 2. 현재 활성 설정 snapshot (2026-05-22 — junsang_v2)

**Branch**: `jekyun_v2` (local) = `origin/jekyun_v2` (commit 46bdb17)
**Run name**: "junsang_v2"
**Yaml 핵심 값** (기본값 외):

| Section.Key | 현재 값 | 출처 entry |
|---|---|---|
| training.total_timesteps | **200000** (본학습) — dry-run 시 5k | #12 |
| training.eval_freq | 10000 | #4 (L6) |
| training.eval_episodes | 3 | #4 (L6) |
| sac.buffer_size | 500000 | #4 (M1) |
| sac.gamma | 0.995 | #4 (H3) |
| sac.gradient_steps | **1** (M2 REVERTED) | #11 (유지) |
| environment.action_vx_scale | **8.0** (P1) | #12 |
| environment.action_rate_limit | **0.2** (P2, NEW) | #12 |
| environment.min_altitude | **3.0** (P4) | #12 |
| environment.min_altitude_start_step | **10** (P4) | #12 |
| reward.auto_drop_threshold | **10.0** (P6) | #12 |
| reward.penalty_crash | **-100.0** (P5) | #12 |
| reward.penalty_overspeed | **-50.0** (P3) | #12 |
| reward.truncation_penalty | **-30.0** (P8) | #12 |
| reward.limit_inverted_tilt | **1.047** (P11, NEW) | #12 |
| reward.penalty_bad_attitude | **-50.0** (P11, NEW) | #12 |
| reward.auto_drop_threshold | 4.0 | jekyun v2 (base) |
| reward.w_dist | 0.5 | jekyun v3 (base) |
| reward.w_heading | 0.7 | jekyun v3 (base) |
| reward.w_impact | 0.4 | jekyun v3 (base) |
| reward.k_impact | 0.05 | jekyun v2 (base) |
| reward.k2_precision | 0.3 | jekyun v2 (base) |
| reward.w_drop_base | 100.0 | jekyun v2 (base) |
| reward.drop_attempt_bonus | 150.0 | jekyun v3 (base, NEW key in v2) |
| reward.truncation_penalty | -80.0 | jekyun v3 (base, NEW key in v2) |
| wandb.entity | nayoonho0922-seoul-national-university | (5/21 적용) |
| wandb.run_name | "junsang_v4" | #12 |

**코드 변경 (junsang_v2)**:
- `train_sac.py`: `_step_rew_impact` callback, `_total_drop_count` counter, `eval_freq` from yaml
- `drone_drop_env.py`: jekyun_v2 그대로 (spin thread + reset guard + tiered recovery + infra kill 강화)

---

## 3. 변경/Run 요약 표 (시간순, 새 것이 아래)

| # | 일시 | Run name | Branch | 한 줄 요약 | 결과 |
|---|---|---|---|---|---|
| 1 | 2026-05-19 | — | junsang | 카메라 파이프라인 복구 (vision world 분리) | OK |
| 2 | 2026-05-19~20 | — | junsang | PX4 airframe rootfs cp (r0~r3) | OK |
| 3 | 2026-05-20 | 0rho5l9f | junsang | SAC 5k smoke (default 설정) | drops=1, error=23.95m |
| 4 | 2026-05-20 | pwkujvev | junsang | H1+H2+H3+M1+M2 + 10k smoke | drops=1, error=14.87m (38%↓) |
| 5 | 2026-05-21~22 | — | junsang | N1=D 200k 시도 ~37k 중단 | drop sparse 부각 |
| 6 | 2026-05-22 | — | junsang | N1=B v1 (w=5) ~16k | mean_rew_impact=0.567 |
| 7 | 2026-05-22 | um8txjvk | junsang | **N1=B v2 (w=8) 200k — 실패** | 0/5 drops, takeoff도 실패 |
| 8 | 2026-05-22 | — | → jekyun_v2 | Branch 교체 (14m offset 원인 진단) | jekyun_v2 base 채택 |
| 9 | 2026-05-22 | tzbebmm4 | jekyun_v2 (junsang_v2) | dry-run 1k | Training complete, sim 정상 |
| 10 | 2026-05-22 | zn7xrm7e | jekyun_v2 (junsang_v2) | 200k 본학습 (online) | **실패** — critic 폭주 (17K @ 63k), ent_coef 1.13, deterministic 정책 망가짐 |
| 11 | 2026-05-23 | w9flirvp | jekyun_v2 (junsang_v3) | M2 REVERTED (gradient_steps 4→1) | **실패** — SAC 안정성 회복했으나 정책 학습 안 됨 (mean_d_xy 20m 정체, 159k 까지) — env/reward 측 근본 문제 |
| 12 | 2026-05-23 | ujvpo8ry(5k), krxfl97k(200k) | jekyun_v2 (junsang_v4) | **Tier 1 (P1~P11) 적용** — action scale 8 + rate limit + early terminate + curriculum 10m + bad_attitude check | drop_error 12~14m 수렴, success=0, inverted 44% |
| 13 | 2026-05-23 | — | jekyun_v2 (junsang_v4) | WandB callback overhaul (rate → 누적 count) | src만 수정, install 미러는 #14 |
| 14 | 2026-05-24~25 | wdwoim34(19k crash), xk7rw5e1(112k kill) | jekyun_v2 (junsang_v4_150k) | DropEpisodeRecorder 추가 + 150k 재학습 | drop_error 13.15m, success=0, 이전과 동일 양상 |
| 15 | 2026-05-25 | ruozrv5x | jekyun_v2 (round1) | **Round 1: #001+#002+#003+#006** — hybrid drop + 보상 스케일 축소 + 종료 조건 정비 | 432 drops, best 4.64m, avg 14.02m, success 1건. d_xy 11.3m 정체 |
| 16 | 2026-05-26~30 | dbi74uif(첫시도), z05fx7g9(최종) | jekyun_v2/junsang (round2) | **Round 2: gradient + max800 + 종료조건 진화** | dbi74uif: 접근실패 (start_step 150 문제). z05fx7g9: success 16건(16배), best 2.53m, 평균 19.09m, post-success regression 발견 |
| 17 | 2026-05-30~31 | q13hli0y(발산), lidq3ydu(157k 크래시) | junsang (round3) | **Round 3 (조합 C): PER + LR 1e-4 + Tau 0.002 + Sigmoid alt + 4가지 안전장치** | 104 drops, success 8건 (Round 2 대비 2배 속도), PX4 로그 20GB 누적 → gz timeout 크래시 |
| 18 | 2026-05-31 | vo1l9wl6(14k 버그), 4j46qwpk(146k 발산) | junsang (round4) | **Round 4 (A+C): Hover per-step 차단** — w_heading 0.3, w_distance_penalty 0.03 | 학습 발산 — ent_coef 6.03 폭주, critic_loss 230k+. 원인: per-step density 축소 → reward sparsity → SAC auto-entropy 양성 피드백 |
| 19 | 2026-05-31 | sdjytkpv (학습 중) | junsang (round5) | **Round 5: Hover terminal penalty** — Round 4 복원, episode 종료 시 -15 (sustained hover만) | (학습 중 — 300k) |

---

## 4. 상세 entry (시간순, 새 것이 아래)

> **형식 규칙** (사용자 명시):
> - 변경: `<key>: <old> → <new>` + 이유
> - 추가: `NEW: <key> = <value>` + 이유
> - 제거: `REMOVED: <key>` + 이유

---

### #1. 2026-05-19 — 카메라 파이프라인 복구

**No yaml change** (코드/world 변경만).

**World/Launch 변경**:
- `gazebo_models/worlds/x_marker_world_vision.sdf` — NEW 파일 (base 의 복사 + Sensors+ogre2 추가)
- `drone_mission.launch.py` L196/204/321: `x_marker_world.sdf → x_marker_world_vision.sdf`

**이유**: base world 에 `gz::sim::systems::Sensors` 플러그인 의도적 제외 (RL 학습 안정성). vision 미션은 카메라 필요 → 분리.

---

### #2. 2026-05-19~20 — PX4 airframe rootfs 동기화

**No yaml change**.

**Container 측 변경**:
- ROMFS 의 `4016~4019_gz_x500_bombard_r{0,1,2,3}` → rootfs 로 cp

**이유**: 기본 PX4 빌드가 rootfs 에 4015 만 채움. 학습 코드는 `PX4_SIM_MODEL=gz_x500_bombard_r{instance_id}` 사용 → 누락된 airframe 으로 silent fail.

---

### #3. 2026-05-20 — SAC 5k smoke (default 설정)

WandB run: `0rho5l9f`
**No parameter change** — 기본값으로 baseline 측정.

**결과**:
- ep_rew_mean: -874 → -936 (평탄/악화)
- drops=1, drop_error=23.95m, mean_rew_drop=0
- safety_violation_rate=1.0, fps=22~37

---

### #4. 2026-05-20 — H1+H2+H3+M1+M2 적용 (10k smoke `pwkujvev`)

**변경**:
- `sac.buffer_size`: 100000 → 500000  (M1)
  - **이유**: 본학습 1M step 시 마지막 100k 만 남아 catastrophic forgetting. 초기 다양성 보존.
- `sac.gamma`: 0.99 → 0.995  (H3)
  - **이유**: effective horizon 100→200 step. 장기 maneuvering credit assignment 강화. H1 의 reward 평탄화와 같이 적용.
- `sac.gradient_steps`: 1 → 4  (M2)
  - **이유**: off-policy sample reuse. fps 22~37 환경에서 update 효율 향상.
- `reward.w_drop_base`: 50 → 20  (H1)
- `reward.r_success_jackpot`: 100 → 30  (H1)
- `reward.penalty_instability`: 50 → 15  (H1)
  - **(H1 묶음 이유)**: critic 이 step reward ±0.5 와 terminal ±100 spike 를 동시에 fit → variance 폭주. terminal scale 축소로 안정화.
- **action_space**: Box(5,) → Box(4,)  (H2, drone_drop_env.py 코드)
  - **이유**: action[4] manual drop trigger 가 CCIP auto-drop 과 충돌, gradient 거의 못 받음. CCIP only 정책으로 단순화.

**추가** (NEW):
- `training.eval_freq` = 10000  (L6, yaml 신규 키)
- `training.eval_episodes` = 3  (L6, yaml 신규 키)
  - **이유**: BestModelCallback 이 hardcoded 10_000 이었음. smoke/본학습 분리 가능하게 config 로 빼냄.

**결과**:
- ep_rew_mean: -1340→-823→-511 (단조 개선)
- drops=1, drop_error=14.87m (이전 23.95m 대비 38%↓)
- critic_loss 11.4→49.1 폭주→32.9 안정화 (M2 안전성 확인)
- 한계: drops=1 (sparse) → N1 안건으로 이행

---

### #5. 2026-05-21~22 — N1=D 200k 시도 (중단)

**No parameter change** — N1 옵션 D ("변수 추가 없이 1M 가서 봄") 채택.

**진행**: ~37k step.
**중단 이유**: drop_count=1 지속, drop sparse signal 문제 부각. `Downloads/N1_drop_sparse_AB방안.md` 분석으로 N1=B (가중치만 강화) 선도입 결정.

---

### #6. 2026-05-22 — N1=B v1 (~16k)

**변경**:
- `reward.w_impact`: 2.0 → 5.0
  - **이유**: drop sparse → 위치 신호 강화. potential-based 아니지만 가중치만 키워 저위험.
- `reward.k_impact`: 0.05 (유지)
  - **이유**: 별도 자료 권고 0.1 은 80m 에서 reward≈0.0017 (다른 항 대비 3자릿수 작음) — 본학습 초반 의미 없음. 0.05 유지.

**추가** (NEW, drone_drop_env.py 코드):
- `r3_impact` 에 `* speed_gate` 곱
  - **이유**: r3_impact 가 potential-based 아닌 단일 상태 함수 → loitering hack 위험. r3_orient 의 anti-milking 패턴 재사용.

**추가** (train_sac.py callback):
- `env/mean_rew_impact` 로깅 (3곳, callback 누락 수정)
  - **이유**: rew_impact 가 info dict 에 있지만 callback 수집 누락. N1=B 의 핵심 측정 지표라 필수.

**결과** (16k 부분 학습):
- mean_rew_impact: 0.567 (메커니즘 작동 확인, dry-run 의 0.00144 대비 ~400배)
- mean_rew_orient: -0.998 → +0.036 (드론이 타겟 방향 학습)

---

### #7. 2026-05-22 — **N1=B v2 — 200k 실패** (`um8txjvk`)

**변경** (#6 대비):
- `reward.w_impact`: 5.0 → **8.0**
- `reward.k_impact`: 0.05 → **0.03**
  - **이유**: v1 의 16k 데이터에서 메커니즘 작동 확인 → "위에서부터 줄이기" 전략으로 ②(w=8, k=0.03) 후보 채택. 후보 비교: ① w=5,k=0.025 (보수), ② w=8,k=0.03 (균형), ③ w=10,k=0.025 (공격적, terminal jackpot 침범 위험).

**추가** (NEW, train_sac.py):
- `env/total_drop_count` 누적 카운터 (3곳)
  - **이유**: env/drop_count 는 rollout-reset 됨 → wandb 차트가 "계속 1" 로 보이는 혼란. 누적 카운터로 stair-step 차트 가능.
  - **적용 시점**: Python 모듈 캐싱 때문에 현재 run 에는 영향 없음. 다음 학습부터 반영.

**Wandb mode 변경**:
- `wandb.mode`: offline → **online** (사용자 선택)
  - **이유**: incremental sync 스크립트 대신 직접. deadlock 위험 인지 후 진행.

**결과 (재앙)**:
- 200k step 완주 + `sac_drop_final.zip` 저장
- evaluate 5 epi (deterministic): **0/5 drops**, mean reward -6060
- trajectory: Target(11,10) → End(-16,23) 반대 방향 비행
- 사용자 GUI 관찰: "처음부터 바닥에 부딪히면서 튕기다 어디론가" → **takeoff 도 실패**
- 정책 완전 무용 → 학습 폐기

**진행 중 진단 (200k 도중)**:
- wandb API 로 19 drops 분석 → drop_error_actual_m 이 **17/19 회 정확히 14.8661m**
- CCIP 1m 예측 vs 실제 14.87m → 14m systematic offset (시뮬 측 구조적 결함 의심)

---

### #8. 2026-05-22 — Branch 교체 (junsang → jekyun_v2)

**No yaml change** — branch checkout 만.

**이유**:
- 14m offset 이 RL parameter 로 해결 불가 (sim 측 결함 의심)
- 사용자가 다른 branch (`origin/jekyun_v2`) 에서 정상 동작 확인
- jekyun_v2 의 commit 09ea871 의 **인프라 안정성 patch 4가지**:
  1. `_spin_loop` (resilient spin thread)
  2. `_reset_depth` recursion guard
  3. tiered CRUISE recovery
  4. `_kill_infra` pkill 폴백
- jekyun_v2 의 `RL_analysis.md` (431줄) 가 우리 200K 실패와 같은 실패 모드 (360K v2 mean_d_xy=19.9m, 안 움직임) 분석 + v3 처방 검증
- 14m offset 진짜 원인 추정: spin thread crash → callback freeze → mission_state freeze → CCIP stale data, 또는 infra reuse 옛 PX4 잔재 → 좌표 오염

**Backup**: `/home/juns/backup_pre_pull_2026-05-22/` (4.7GB, 8 카테고리)

---

### #9. 2026-05-22 — junsang_v2 셋업 + dry-run 1k (`tzbebmm4`)

**Base**: jekyun_v2 (commit 46bdb17). reward v3 + 인프라 patch 그대로 수용.

**Jekyun_v2 의 reward 측 활성 값** (우리가 안 건드림, 참고용):
- `auto_drop_threshold`: 2.0 → 4.0 (jekyun, drop 빈도↑)
- `w_dist`: 1.0 → 0.5 (jekyun)
- `w_heading`: 1.0 → 0.7 (jekyun v3)
- `w_impact`: 2.0 → 0.4 (jekyun v3, 우리 8.0 의 1/20)
- `k2_precision`: 5.0 → 0.3 (jekyun, 먼 거리도 보상)
- `w_drop_base`: 50.0 → 100.0 (jekyun)
- NEW `drop_attempt_bonus`: 150.0 (jekyun v3)
- NEW `truncation_penalty`: -80.0 (jekyun v3)

**Re-apply on jekyun_v2**:
- `sac.buffer_size`: 100000 → 500000 (우리 M1 재적용)
- `sac.gamma`: 0.99 → 0.995 (우리 H3 재적용)
- `sac.gradient_steps`: 1 → 4 (우리 M2 재적용)
- `training.eval_freq` = 10000 NEW (우리 L6 재적용)
- `training.eval_episodes` = 3 NEW
- `training.total_timesteps`: 1000000 → **1000** (dry-run 임시, 원래 200000)
  - **이유**: jekyun_v2 default 1M 이지만, 우리는 200k 우선 검증 (사용자 결정). dry-run 은 1k.
- `wandb.run_name`: "L4-AutoDrop-v2" → "junsang_v2"
- train_sac.py: `_step_rew_impact` callback, `_total_drop_count` counter, `eval_freq` from yaml

**채택 안 함** (REMOVED from history):
- H1 (terminal scale 축소) — jekyun v3 는 반대 방향
- H2 (action 5d→4d) — jekyun_v2 는 5d 유지
- N1=B v2 (w_impact=8) — jekyun 분석으로 per-step 강화 실패 모드 확정
- speed_gate 코드 patch — jekyun_v2 yaml 에 이미 `speed_gate_enabled: true`
- vision world 분리 (commit 29b1c9b) — RL 학습엔 무관 (보류)

**Dry-run 1k 결과** (`tzbebmm4`, offline):
- **Training complete** ✓ (sac_drop_final.zip 저장)
- PX4 ready 17초, RuntimeError/CRUISE timeout 없음 → sim 정상 동작
- env/mean_d_xy: 27.87m (learning_starts=1000 = total_timesteps 라 random 탐색만)
- env/total_drop_count: 0 (1k 짧음, drop 발생 안 함)
- env/mean_rew_impact: 0.099 (callback 정상 작동)
- env/safety_violation_rate: 1.0 (random 단계 정상)

**진단 patch (임시)**:
- `drone_drop_env.py` L1210: `px4_log = open(os.devnull, 'w')` → `open('/tmp/px4.log', 'w')`
  - **이유**: 5k 시도 시 CRUISE timeout 원인 진단용 (junsang 5/19 패턴 재사용)
- **REVERT 완료**: PX4 log 416MB 폭증 (pxh prompt spam), 진단 가치 없음 확인 후 원복 + log 삭제

**다음**: 200k 본학습 또는 5k 중간 검증.

---

### #10. 2026-05-22 — **200k 본학습 시작 (junsang_v2, online)**

WandB run: (진행 중, online mode)
**Base**: #9 (dry-run 1k 성공)

**변경**:
- `training.total_timesteps`: 1000 → **200000**
  - **이유**: dry-run 1k 성공 (Training complete, sim 정상 동작 확인) → 본학습 진입.

**환경**:
- `WANDB_MODE=online` (사용자 결정, deadlock 위험 인지 후 진행)
- 컨테이너 재시작 후 (이전 OOM exit 137 발생) — PX4 airframes rootfs 5개 정상 확인 후 진입

**결과**: (진행 중, 200k ≈ 7시간 예상)

**모니터링 지표**:
- `env/total_drop_count` (stair-step ↑)
- `env/mean_d_xy`, `env/mean_d_impact` (점진 ↓)
- `env/mean_rew_impact` (w_impact=0.4 라 작은 값)
- `env/drop_error_actual_m` (drop 발생 시)
- `env/safety_violation_rate` (learning_starts 후 ↓)
- `rollout/ep_rew_mean` (단조 또는 점진 개선)

**완주 후 결정 트리** (jekyun RL_analysis 의 예상치):
- `mean_d_xy ≤ 5m` → 정상 수렴, 추가 학습 또는 정밀화
- `mean_d_xy 5~15m` → 처방 가이드 §3 진단 매트릭스
- `mean_d_xy > 15m`, drops=0 → jekyun Stage 1 재조정 필요

**즉시 중단 조건**:
- RuntimeError "reset() recursive" → spin thread 또는 infra 측 문제 재발 → 로그 확인
- 30k step 까지 drops=0 → reward 설계 재검토

**결과 (200k 완주)** — WandB run `zn7xrm7e`, state=finished, **학습 실패**:

| 지표 | 값 | 평가 |
|---|---|---|
| `train/critic_loss` | **17,047** (200k 시점) | **발산** (정상 ~100) — 63k 시점 685→8,400 점프 후 회복 못함 |
| `train/actor_loss` | **-2,027** | **발산** (정상 -50~-200) |
| `train/ent_coef` | **1.113** | **통제 불능** — 시작 1.0 → 0.5 → **1.35 → 1.13 진동** (정상은 단조 감소 0.1~0.5) |
| `train/n_updates` | 795,996 | M2=4 정상 동작 (200k × 4) |
| `env/total_drop_count` | **106** | 의미있는 개선 (이전 N1=B v2 의 0 대비). drop_attempt_bonus=150 효과 ✓ |
| `env/drop_error_actual_m` (final) | 9.02m | 14m offset 없음 ✓ (인프라 patch 효과) |
| `env/mean_d_xy` (final) | 32.68m | 정책 망가짐 — target 에서 멀어짐 |
| `env/safety_violation_rate` | 1.0 | 마지막까지 crash 빈발 |
| Deterministic evaluate | **"전혀 학습되지 않음"** | 사용자 GUI 관찰 |

**Timeline (40 samples)**:
- Phase 1 (0~30k): critic 27→222 정상, d_xy 117→12 (target 접근), drops 1→15 ✓
- Phase 2 (30~60k): critic 222→657 증가 시작 ⚠
- **Phase 3 (63k 부근)**: critic 685 → **8,400** (10배 점프) 🚨
- Phase 4 (71k~200k): critic 5K~21K 진동, actor -1700~-2900, ent_coef 1.05~1.35 진동, d_xy 11~290m 진동

**원인 진단**:
- **M2 (gradient_steps=4)** 이 **jekyun v3 의 큰 terminal reward (drop_attempt_bonus 150 + w_drop_base 100 + jackpot 100 = 최대 +350, truncation -80)** 와 충돌
- critic 이 step reward (±0.5) 와 terminal spike (+250~350) 를 4배 빠르게 fit 하려다 발산
- **H3 (gamma 0.995)** 도 future reward 누적 효과 키워 Q variance ↑
- 63k 시점 = buffer 의 ~13% 시점에 학습 본격화되며 임계 도달

**Archive**: `/workspace/ros2_ws/rl_checkpoints/archive/junsang_v2_200k_failed_2026-05-22/` (22MB, 7 zip — 75k~95k + final + model)

---

### #11. 2026-05-23 — **junsang_v3: M2 REVERTED** (gradient_steps 4→1) — 단독 lever

**Base**: #10 (junsang_v2, 200k 학습 실패)

**변경**:
- `sac.gradient_steps`: 4 → **1**
  - **이유**: #10 에서 63k 시점 critic 685→8,400 점프 + 200k 까지 발산 지속. M2 만 단독 변경하여 critic 폭주 원인 검증 (one-lever-at-a-time 원칙).
- `wandb.run_name`: "junsang_v2" → "junsang_v3"
  - **이유**: 학습 구분 명확화.

**유지** (변경 안 함):
- `sac.buffer_size`: 500000 (M1 유지)
- `sac.gamma`: 0.995 (H3 유지)
- 모든 jekyun_v3 reward (drop_attempt_bonus 150 등)
- train_sac.py callback 들
- drone_drop_env.py: jekyun_v2 그대로

**검증 가설**:
- 만약 critic 안정 (loss < 300) → M2 가 critic 폭주의 단독 원인 확정
- 여전히 폭주 → terminal scale 또는 H3 가 추가 원인 (Stage 2 lever 필요)

**예상**:
- jekyun v1 (118K, gradient_steps=1, w_drop_base=50) 는 정상 수렴했음 → gradient_steps=1 이 그 안정성의 핵심 가능성
- M1 (큰 buffer) + H3 (큰 gamma) 만 추가는 SAC 측 효과 작음
- 200k 후 critic 안정 + drops 의미있게 증가 + d_xy 단조 감소 → 본격 학습 가능 환경 확인

**환경**:
- `WANDB_MODE=online`
- Container 안의 yaml + train_sac.py + drone_drop_env.py 모두 #10 과 동일 (gradient_steps 만 변경)

**상태**: 사용자 직접 실행 예정 (tmux 패턴). 명령은 #10 과 동일 (`ros2 run rl_navigation train_sac`).

---

### #12. 2026-05-23 — **junsang_v4: Tier 1 (P1~P11) 적용**

**Base**: #11 (junsang_v3, 정책 학습 실패)
**참조**: [design_review_2026-05-23.md](design_review_2026-05-23.md) (9회차 검토 완료)

**변경 — yaml (11 키)**:
- `environment.action_vx_scale`: 15.0 → **8.0**  (P1)
  - **이유**: 사용자 GUI "시작하자마자 튀어나가는" → 최대 수평 속도 제한
- `environment.action_rate_limit`: NEW → **0.2**  (P2)
  - **이유**: 사용자 "가속도 제한" → step 당 |Δa| ≤ 0.2 hard clip
- `environment.min_altitude`: 2.0 → **3.0**  (P4)
- `environment.min_altitude_start_step`: 20 → **10**  (P4)
  - **이유**: 사용자 "고도 최저선" → 상향 + 검사 빠르게
- `reward.auto_drop_threshold`: 4.0 → **10.0**  (P6)
  - **이유**: 사용자 "10m 안 일단 성공" → curriculum phase 1
- `reward.penalty_crash`: -10.0 → **-100.0**  (P5)
- `reward.penalty_overspeed`: -8.0 → **-50.0**  (P3)
- `reward.truncation_penalty`: -80.0 → **-30.0**  (P8)
  - **이유**: 사용자 "바닥 닿으면 안 됨" + timeout 압도 완화
- `reward.limit_inverted_tilt`: NEW → **1.047** (60°)  (P11)
- `reward.penalty_bad_attitude`: NEW → **-50.0**  (P11)
  - **이유**: 사용자 "절대 안 좋은 행동" (급격 회전/inverted) → step 별 검사 + truncate
- `wandb.run_name`: "junsang_v3" → **"junsang_v4"**

**변경 — drone_drop_env.py 코드**:
- cfg 로드 5 키 추가 (`_cfg_action_rate_limit`, `_cfg_limit_inverted_tilt`, `_cfg_penalty_bad_attitude`, `_cfg_drop_attempt_bonus`, `_cfg_truncation_penalty`)
  - **숨겨진 버그 발견**: `drop_attempt_bonus=150` 과 `truncation_penalty=-80` 이 코드에 **hardcoded** (L653, L695) — yaml 변경 효과 없었음. cfg 로드 + 코드 사용 변경 (이전 entry 의 yaml 변경이 사실상 무력했음)
- step() L572 직후: P2 action rate clip 추가
- step() L688 의 _compute_reward 호출 후: P11 safety 검사 + truncated 결정 (crash/overspeed/ang_vel/inverted)
- step() L692 의 max_steps timeout: `info['truncate_reason'] = 'timeout'` 추가

**변경 — train_sac.py callback**:
- `WandbMetricsCallback.__init__`: `_truncate_counts` dict + `_rollout_done_episodes` counter 추가
- `_on_step`: `info['truncate_reason']` 집계
- `_on_rollout_end`: 6 metric wandb 로깅 (`env/truncate_<crash|overspeed|ang_vel|inverted|timeout>_rate` + `env/drop_terminated_rate`)

**채택 안 함 (Tier 2 보류)**:
- P7 (per-step reward 강화) — orbit-milking 위험, Tier 1 결과 보고 결정
- action_vz_scale 축소
- limit_tilt 강화 (drop 시점만)
- wandb_watch.py 외부 모니터링

**Archive (이전 run)**: `/workspace/ros2_ws/rl_checkpoints/archive/junsang_v3_w9flirvp_failed_2026-05-23/` (7 zip, 22MB)

**Syntax 검증**: `python3 -c 'ast.parse(...)'` 양 파일 통과 ✓

**다음 단계**:
1. 5k dry-run (yaml total_timesteps 임시 5000) — WANDB_MODE=offline
2. 통과 기준 (정량): RuntimeError 없음, Training complete, total_drop_count≥1, safety_violation_rate<1.0, mean_d_xy 측정됨, ep_len_mean<500
   (entry #13 이후 metric 명: `env/total_drop_terminated_count` ≥ 1, `env/total_safety_violation_count` / `env/total_episodes` < 1.0)
3. 임계 적정성: `env/truncate_ang_vel_rate` < 0.5 (만약 ≥50% 이면 limit_ang_vel/action_rate_limit 완화 검토)
   (entry #13 이후: `env/total_truncate_ang_vel_count` / `env/total_episodes` < 0.5)
4. 답답함 점검: mean_d_xy 가 5k 동안 감소 (시작 50~100m → 끝 30~50m). 안 줄어들면 action_rate_limit 0.2→0.3
5. 5k 통과 → 사용자가 200k 본학습 (WANDB_MODE=online)

---

### #13. 2026-05-23 — **WandB callback metric overhaul (rate → 누적 count, episode x축)**

**Base**: #12 (junsang_v4 Tier 1, 현재 phase 1 진행 중)
**적용 시점**: **src 만 수정 — install 미러는 Phase 2 진입 시점**. Phase 1 학습 영향 0.

**문제**:
- rollout 의 episode 수 적을 때 rate metric (예: `env/truncate_crash_rate`) 가 0/N or N/N 형태 → wandb 차트가 0/1 binary 진동
- x축 = `time/total_timesteps` 이라 episode 길이 비균질 (54 step vs 500 step) → 추세 보기 어려움

**해결**:
- 모든 rate metric → 누적 count 로 변환 (monotonic stair-step)
- universal `env/total_episodes` counter 추가 (x축 기준)
- `wandb.define_metric('env/total_*', step_metric='env/total_episodes')` 자동 매핑

**제거** (rate 8개):
- `env/truncate_<crash|overspeed|ang_vel|inverted|timeout>_rate` (5개)
- `env/drop_terminated_rate`
- `env/success_rate`
- `env/safety_violation_rate`

**추가** (누적 count 9개):
- `env/total_episodes` (universal counter, x축)
- `env/total_drop_terminated_count` (기존 `total_drop_count` 의 이름 변경)
- `env/total_success_count` (d_error ≤ 0.5m 누적)
- `env/total_jackpot_count` (d_error ≤ 0.1m 누적, success_threshold)
- `env/total_truncate_<crash|overspeed|ang_vel|inverted|timeout>_count` (5개)
- `env/total_safety_violation_count` (crash + overspeed 합산)

**유지** (사용자 결정 #4):
- `env/drop_count` (per-rollout, 그대로)
- `env/mean_*` (mean metric, 그대로)
- `env/physics_glitch_count` (이미 누적)
- `train/*`, `rollout/*` (SAC/SB3 기본)

**코드 변경** (train_sac.py 의 WandbMetricsCallback):
- `__init__`: 누적 변수 추가 (`_total_episodes`, `_total_drop_terminated`, `_total_success_count`, `_total_jackpot_count`). 이전 `_safety_violation_count`, `_total_steps`, `_rollout_done_episodes` 제거 (rate 계산용이라 불요)
- `_on_step`: episode 종료 (done) 시 누적 카운터 갱신. truncate_counts 도 rollout reset 없이 누적.
- `_on_rollout_end`: rate 계산 코드 모두 제거 (`if self._rollout_done_episodes > 0` 블록 + `safety_violation_rate` 블록). 누적 log_dict 항목 추가.
- `main()` 의 `wandb.init()` 직후: `wandb.define_metric('env/total_*', step_metric='env/total_episodes')` 추가.

**검증**:
- syntax OK (ast.parse)
- src 18 occurrence 새 키 ✓
- install 0 occurrence (학습 영향 0) ✓

**Phase 2 진입 시점에 install 미러** ([drone_sim_tmux_training_guide.txt](guides/drone_sim_tmux_training_guide.txt) 의 phase 전환 절차 + [A_phased_curriculum_도입방안.md](A_phased_curriculum_도입방안.md) 부록 체크리스트 참조):

```bash
docker cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py \
  drone-bombard-harmonic:/workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py
```

---

### #14. 2026-05-24~25 — junsang_v4_150k 재학습 + DropEpisodeRecorder

WandB run: `wdwoim34` (19k에서 infra crash), `xk7rw5e1` (15k resume → 112k에서 사용자 kill)
**Base**: #12 (junsang_v4 Tier 1)

**추가** (코드):
- `train_sac.py`: `DropEpisodeRecorderCallback` — drop 에피소드마다 action/obs/reward 저장 (.npz)
- `replay_drop_episode.py`: 저장된 drop episode GUI 재생 스크립트
- `train_sac.py`: callback overhaul (#13) install 미러링 완료

**변경** (yaml):
- `training.total_timesteps`: 200000 → **150000**
- `wandb.run_name`: "junsang_v4" → **"junsang_v4_150k"**

**결과** (112k step, 1112 episodes):
- total_drops: 220, total_success: **0**
- drop_error: mean 13.15m, median 12.89m, min 2.32m
- inverted: 486 (**44%** — 최대 종료 원인)
- d_xy: 12.7m (최종), 14~90m 불안정 진동
- DropEpisodeRecorder: 275 episode 정상 저장

**결론**: 이전 학습(200k)과 동일한 양상. 파라미터가 아닌 구조적 문제 확인 → Round 1 결정.

---

### #15. 2026-05-25 — **Round 1: Issue #001+#002+#003+#006 동시 적용** (코드 수정 완료)

**Base**: #14 (junsang_v4_150k 결과 분석)
**참조**: issues/master.txt, issues/issue_001~010

**변경 — yaml (14 키)**:

  Issue #001 (보상 정밀도):
  - `reward.drop_attempt_bonus`: 150.0 → **30.0**
  - `reward.k2_precision`: 0.3 → **0.5**
  - `reward.r_success_jackpot`: 100.0 → **50.0**

  Issue #003 (스케일 축소):
  - `reward.penalty_crash`: -100.0 → **-50.0**
  - `reward.penalty_overspeed`: -50.0 → **-30.0**
  - `reward.penalty_bad_attitude`: -50.0 → **-30.0**
  - `reward.truncation_penalty`: -30.0 → **-15.0**

  Issue #006 (hybrid drop):
  - `reward.auto_drop_threshold`: 10.0 → **2.0**
  - NEW `reward.manual_drop_threshold` = **0.5**
  - NEW `reward.min_drop_step` = **10** (최초 50 → dry-run 후 10으로 조정)

  Issue #010 (prediction bonus, 보수적 도입):
  - NEW `reward.w_prediction` = **20.0**
  - NEW `reward.k_prediction` = **0.1**

  Issue #002 (종료 조건):
  - `environment.min_altitude_start_step`: 10 → **1**
  - NEW `environment.ground_contact_altitude` = **0.5**
  - NEW `environment.max_distance` = **100.0**
  - NEW `reward.penalty_out_of_range` = **-30.0**

  기타:
  - `wandb.run_name`: "junsang_v4_150k" → **"round1_hybrid_drop"**

**변경 — drone_drop_env.py**:
  - hybrid drop: `(action[4] > 0.5 and step >= 10) or (d_impact <= 2.0)` and not dropped
  - prediction_bonus: `w_prediction * exp(-k_prediction * |d_impact - actual_error|)`
  - ground contact: `altitude < 0.5m → crash` (무조건, step 무관)
  - out_of_range: `d_xy > 100m → truncate + penalty -30`
  - `info['drop_trigger']` = 'manual' / 'auto' 추가
  - default 값 전부 Round 1 값으로 정합

**변경 — train_sac.py**:
  - `_truncate_counts`에 `'out_of_range': 0` 추가
  - `_total_drop_manual` / `_total_drop_auto` 카운터 + wandb 로깅 추가

**보상 curve** (prediction gap=0 가정):
  d_error 0m=200(+50 jackpot), 1m=111, 2m=87, 5m=58, 10m=51, 15m=50

**추가 변경 (dry-run/학습 중 발견)**:
  - NEW `reward.k_drop_proximity` = **0.3**
    bonus = 30 * exp(-0.3 * d_xy). 즉시 drop 방지 — 가까이 가야 bonus 받음.
    d_xy=50m→0, 14m→0.5, 3m→12.2, 0m→30.
    (run 2zrnjipt에서 step 10마다 즉시 drop 패턴 발견 → 추가)

**추가 변경 (Issue #013 발견 후)**:
  - `reward.success_threshold`: 0.1 → **5.0** (total_success_count 중간 목표)
  - NEW `reward.jackpot_threshold` = **0.1** (total_jackpot_count 최종 목표, 분리)
  - SDF 수정: 8개 model.sdf `<topic>` → `<detach_topic>` (Critical #013)

**추가 변경 (Issue #014 해결 후)**:
  - x500_bombard/model.sdf: <include merge="true"> → **inline** 변환 (#014)
  - drone_drop_env.py: PX4_SIM_MODEL → **PX4_GZ_MODEL_NAME** (pre-spawn 연결)
  - drone_drop_env.py: world SDF에 드론 **pre-spawn inject** 로직 추가
  - drone_drop_env.py: Gazebo **paused 시작 → PX4 연결 후 unpause**
  - world SDF payload_0 z: 0.36(테스트 잔여) → **0.14** 원복

**추가 변경 (manual drop exploit 발견)**:
  - manual drop 비활성 (action[4] 무시)
  - NEW `reward.random_drop_start_step` = **150**
  - NEW `reward.random_drop_prob` = **0.005** (0.5%/step)
  - `reward.auto_drop_threshold`: 2.0 → **3.0**
  - `reward.manual_drop_threshold`, `reward.min_drop_step` 제거
  - train_sac.py: total_drop_manual → **total_drop_random**

**이전 시도**:
  vf5dgx55~r5zp3k1c — payload 미분리 (#013/#014)
  v6i3vy4p — manual drop "즉시 drop" exploit 발견 → random drop으로 전환
**추가 변경 (검증 방법 전환)**:
  - train_sac.py: DropEpisodeRecorderCallback에 **best drop 모델 저장** 추가
    auto drop 성공 시 best_drop_error 갱신되면 모델 .zip 저장
    저장 위치: drop_episodes/best_drops/best_err{X}m_step{Y}.zip
  - action replay(open loop) 폐기 → deterministic evaluate(closed loop)로 검증
    Issue #015: replay는 원래 궤적 재현 불가 (420 step 뒤 30m 차이)

**인프라 정리**:
  - 폴더 통합: local_dronebombard_simulation/ → Drone-Bombard-Simulation/local/
  - 디스크 정리: PX4 로그 33G + /tmp 9.5G + 빌드캐시 1.2G = 44G 회수
  - training guide에 PX4 로그 삭제 루틴 추가

**현재**: run ruozrv5x 150k 완주. random drop + auto drop + 실제 물리 payload 낙하.
  문서 위치: /home/juns/Drone-Bombard-Simulation/local/

---

### #16. 2026-05-26 — **Round 2: gradient 완만화 (접근+drop 보상 개선)**

WandB run: `(예정)`
**Base**: #15 (Round 1 — run ruozrv5x 150k 완주)

**Round 1 분석 결과**:
  432 drops, best 4.64m, avg 14.02m, success 1건 (0.2%)
  d_xy 최소값 평균 11.3m — auto_drop 3m 도달 0건
  10m 구간 drop 보상이 2.2점으로 noise에 묻혀 학습 신호 부재

**변경**:
- `reward.w_dist`: 0.5 → **1.0**
  - **이유**: 접근 보상 2배. 드론이 9~11m 정체 구간을 돌파하도록.
- `reward.k2_precision`: 0.5 → **0.2**
  - **이유**: precision 감쇠 완만하게. 10m에서 0.7→13.5 (학습 가능한 신호).
- `reward.k_drop_proximity`: 0.3 → **0.15**
  - **이유**: proximity 감쇠 완만하게. 10m에서 1.5→6.7.

**추가 변경 (Round 2 첫 시도 dbi74uif 분석 후)**:
- `environment.max_steps`: 500 → **800**
  - **이유**: Round 1 best가 420 step에 4.6m. 500은 접근 시간 부족.
- `reward.random_drop_start_step`: 150 → **600**
  - **이유**: step 150은 너무 이른 시점에 random drop 발동. 600 step 자유 접근 확보.
- NEW `environment.max_altitude` = **25.0**
  - **이유**: 고도 상한. 드론이 무한정 상승하며 시간 낭비 방지.
- NEW `reward.penalty_max_altitude` = **-30.0**
- NEW `environment.stagnation_window` = **200**
- NEW `environment.stagnation_min_progress` = **2.0**
- NEW `environment.stagnation_start_step` = **200**
  - **이유**: 200 step 동안 d_xy 2m 미감소 시 정체 판단. step 200 이후 체크.
- NEW `reward.penalty_stagnation` = **-15.0**
- WandB metric 22개 → **8개** 정리 (이름 변경 포함)
  - 상세: guides/wandb_metrics_guide.txt

**유지**:
- w_drop_base=100, drop_attempt_bonus=30, auto_drop_threshold=3.0
- random_drop_prob=0.005

**변경 전후 비교 (proximity + precision)**:
  15m:  0.4 →  8.2
  10m:  2.2 → 20.2
   8m:  4.5 → 29.2
   5m: 14.9 → 51.0
   3m: 34.5 → 74.0
   0m:  130 →  130

**Round 2 첫 시도 결과 (run dbi74uif, 150k)**:
  deterministic eval: d_xy 14~16m, 모두 random drop, 접근 실패
  원인: random_drop_start_step=150 → step 200에서 조기 drop → 접근 시간 부족

**결과**: (최종 학습 예정)

**결론 / 다음**: gradient 완만화 + 접근 시간 확보(max_steps 800, start_step 600)
  + 이탈 조기 차단(max_altitude, stagnation)으로 학습 효율 강화.

**추가 변경 (2026-05-30, 학습 진화 중)**:
- `environment.max_altitude` 제거 → drop 시점 고도 페널티로 대체
- NEW `reward.altitude_drop_threshold` = 15.0, `altitude_drop_w` = 1.0, `altitude_drop_k` = 0.15
  - **이유**: 비행 중 고도 강제할 필요 없음. drop 시 너무 높으면 수평 drift 큼.
- `environment.stagnation_*` 제거 → speed_gate가 이미 hover 보상 차단
- `truncate_reason 8종 wandb 카운터 추가` → 원인 추적
- `reset() pos_enu 초기화 추가` → n_steps=1 가짜 success 버그 fix

**Round 2 최종 결과 (run z05fx7g9, 150k)**:
  총 drop: 427건, 평균 19.09m, 중앙값 11.55m
  Best: 2.53m (Round 1 4.64m 대비 개선)
  Success (<5m): 16건 (Round 1 1건 대비 16배 증가)
  Deterministic eval: 3 epi 전부 crash, drop 0건

**발견된 문제**:
  1) Reset 버그 (n_steps=1 가짜 success 13건) — 해결
  2) Post-success regression — Issue #016, Round 3 처방

---

### #17. 2026-05-30 — **Round 3 (조합 C): PER + LR 1e-4 + Tau 0.002**

WandB run: `(예정)`
**Base**: #16 (Round 2 — z05fx7g9 결과 분석)

**문제 식별**:
  Issue #016 Post-success regression — real success 직후 5 epi 평균 19.72m,
  전체 평균 19.09m보다 나쁨. SAC sparse reward의 알려진 패턴.
  4가지 메커니즘:
    a) Critic overshoot
    b) Policy gradient over-correction
    c) Replay buffer 압도 (success 1/427)
    d) High entropy exploration

**변경**:
- `sac.learning_rate`: 3.0e-4 → **1.0e-4**
  - **이유**: 모든 update 점진적, critic overshoot 직접 완화
- `sac.tau`: 0.005 → **0.002**
  - **이유**: Target network 안정화, Q-value 발산 차단
- `Replay Buffer`: 표준 → **PrioritizedReplayBuffer (PER)**
  - alpha=0.6 (priority 강도)
  - beta=0.4 → 1.0 (importance sampling, schedule)
  - **이유**: TD-error 큰 success transition 자주 샘플링

**유지**:
  - 보상 구조 (Round 2)
  - max_steps 800, random_drop_start 600
  - Drop 시점 고도 페널티

**Pre-flight**:
  - Round 2 결과 백업 (archive/round2_z05fx7g9_2026-05-30/)
  - Reset 버그 fix 코드 베이스 사용

**Round 3 첫 시도 발산 (run q13hli0y, 30k 중단)**:
  step 17731 단일 epi reward -6.77e+9 발생 (≈ 드론 165m 고도에서 drop)
  원인: 지수 고도 페널티 무한대 폭주
  PER이 천문학적 priority 부여 → buffer 오염 → 학습 망가짐
  archive/round3_q13hli0y_FAILED_2026-05-30/ 보존

**Round 3 수정 (4가지 안전장치)**:
- `reward.alt_penalty_*` (NEW): Sigmoid 페널티 (지수 폐기)
  - alt_penalty_max=50.0, alt_penalty_mid=30.0, alt_penalty_k=0.15
  - penalty = -max * sigmoid(k * (alt - mid))
  - 출력 (-50, 0) 유계 → 폭주 불가
  - 거리별: 15m→-4.7, 20m→-11.7, 30m→-25, 50m→-47.5, 100m+→-50
  - **이유**: 음수 지수 4곳(proximity/precision/prediction/impact)은 본질
            적 유계라 안전. 양수 지수(고도)만 폭주 위험 → sigmoid로 교체.
- `environment.max_altitude` = 50.0 (NEW, 부활):
  - 비행 중 50m 초과 시 truncate
  - **이유**: 드론이 무한정 상승 차단 (sigmoid 페널티 외 추가 방어층)
- `reward.penalty_max_altitude` = -15.0 (NEW): timeout과 동일 페널티
  - **이유**: 0 페널티면 "climb-out" exploit 위험. -15는 가벼운 견제.
- `sac.per_priority_max` = 30.0 (NEW): PER priority 상한
  - priority = min(30, (|reward|+eps)^alpha)
  - **이유**: 이론 max reward 200 → priority 200^0.6 ≈ 24. cap 30은 충분.
- Hard cap [-200, +300] + warning (drone_drop_env.py step 끝):
  - reward 범위 밖이면 print warning + np.clip
  - **이유**: 방어적 안전망. 정상 학습엔 영향 없음. 발생 시 즉시 알림.

**유지**:
  - learning_rate=1e-4, tau=0.002, PER alpha=0.6 eps=0.1
  - 보상 구조 (Round 2)
  - max_steps 800, random_drop_start 600

**결과 (run lidq3ydu, 157k 크래시)**:
  - 총 drop 104건, 평균 22.42m, Best 4.32m, Success 8건 (7.7%)
  - PER + LR/Tau 효과: success 빈도 2배 가속 (Round 2: 16/150k vs Round 3: 8/100k)
  - Best episodes (107k, 108k, 124k): 장시간 비행 + 정밀 drop, reward 499~550
  - 100~125k 최우수 (avg 13.9m, success 3건)
  - Post-success regression 여전 (125~150k avg 35m)
  - Hover exploit 발생 (25~50k 45%, 100~125k 36%)
  - 4가지 안전장치 작동 (Hard cap 발동 없음, [WARN] 없음)

**크래시 원인**:
  - PX4 로그 20GB 누적 (.ulg 파일 2300+개)
  - Gazebo 응답 지연 → gz model --list 5초 timeout
  - reset() 중 예외 → 학습 abort
  - 마지막 체크포인트 95k (이후 60k 손실)

**결론 / 다음**:
  Round 3 PER + LR/Tau 처방 부분 검증 (success 2배 속도).
  Hover exploit 새로운 문제 식별 → Issue #017, Round 4 처방.
  PX4 로깅 비활성화로 인프라 안정성 확보.

---

### #18. 2026-05-31 — **Round 4 (A+C): Hover 차단**

WandB run: `vo1l9wl6` (학습 중)
**Base**: #17 (Round 3 — lidq3ydu 분석)

**문제 식별 (Issue #017)**:
  drop_error 13~16m 구간에 16건 (15.4%)
  14.87m = sqrt(11² + 10²) = spawn→target 거리 정확 매칭
  n_steps 600+ → 스폰에서 random_drop 대기 패턴
  원인: w_heading 0.7 × 600 step = +420 → hover가 안전한 수익원

**변경 (A + C 조합)**:
- `reward.w_heading`: 0.7 → **0.3** (Issue #017 A)
  - **이유**: hover의 주요 수입원 약화 (수익 57% 감소)
- NEW `reward.w_distance_penalty` = **0.03** (Issue #017 C)
  - **이유**: per-step penalty = -w * d_xy / 50
  - d_xy=15m → -0.009/step, 600 step → -5.4
  - 멀리 있을수록 비용 → hover 차단

**유지** (Round 3):
  - PER (alpha=0.6, eps=0.1, priority_max=30)
  - learning_rate=1e-4, tau=0.002
  - 4가지 안전장치 (sigmoid alt, max_alt truncate, PER cap, hard cap)
  - 보상 구조 (w_dist 1.0, k2 0.2, k_prox 0.15)
  - max_steps 800, random_drop_start 600

**인프라 변경**:
- PX4 로깅 비활성화: SDLOG_MODE 1 → -1
  - `/opt/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS`
  - **이유**: .ulg 누적 20GB → Gazebo timeout 크래시
  - 우리 RL 학습엔 .ulg 사용 안 함 → 손실 없음

**시뮬레이션 (300 step 예시)**:
  Hover 600 step at d_xy=15m: +183 (기존 +422, -57%)
  타겟 접근 400 step (15→5m): +179
  Success 500 step (15→3m): +232
  → success > hover ≈ 접근

**Round 4 결과**:
  vo1l9wl6 (14k 중단): reset 버그 부작용 — pos_enu z=0 → ground_contact 트리거
    Fix: drone_drop_env.py reset()에서 pos_enu = (0, 0, 5.0)
  4j46qwpk (146k 발산):
    - ent_coef 6.03 (정상 0.3~0.5의 12배)
    - critic_loss 230,000+ (정상 100~500의 500배)
    - ep_rew_mean -20, ep_len 감소 추세
    - 85 drops, 평균 25m, success 6건 (Round 3 8건/100k와 비슷)
    - 56~70k 구간 발산 시작 — 큰 양수(+187,+274) + 큰 음수(-184,-256) 교차

**발산 원인 분석 (정확한 메커니즘)**:
  1. w_heading 0.7→0.3 + distance_penalty 0.03 → per-step 보상 magnitude 감소
  2. Per-step / drop reward 비율: Round 3 1:300 → Round 4 1:700 (2배 sparse)
  3. Critic estimate variance 폭증
  4. Policy 집중 (한 가지 전략) → entropy ↓
  5. SAC auto-tuning "exploration 부족" 판단 → ent_coef ↑
  6. Bounded action space [-1,1] → entropy 못 올라감
  7. SAC 더 ent_coef ↑ → 양성 피드백 → 발산

**결론 / 다음**:
  Per-step 보상 density 변경은 SAC 발산 위험.
  Hover 차단은 다른 방법 필요 — Episode 종료 페널티로 시도 (Round 5).
  archive/round4_4j46qwpk_diverged_2026-05-31/

---

### #19. 2026-05-31 — **Round 5: Hover Terminal Penalty**

WandB run: `sdjytkpv` (학습 중)
**Base**: #18 (Round 4 발산 분석)

**문제 식별**:
  Round 4의 per-step hover 차단이 SAC auto-entropy 발산 트리거.
  Per-step 보상 density 보존하면서 hover 차단 필요.

**복원 (Round 4 변경 전면 되돌림)**:
- `reward.w_heading`: 0.3 → **0.7** (Round 3 수준)
- `reward.w_distance_penalty`: 0.03 → **0** (비활성)

**신규 (Episode 종료 페널티)**:
- NEW `reward.hover_speed_threshold` = **1.0** m/s
  - **이유**: 정지 판단 기준. 정상 비행 시 2~5 m/s 유지.
- NEW `reward.hover_consecutive_threshold` = **200** step
  - **이유**: episode max 800의 25% — 잠시 hover OK, 지속만 BAD
- NEW `reward.penalty_hover` = **-15.0**
  - **이유**: timeout과 동일 — 적당한 견제 (-50은 과함)

**메커니즘**:
  ```python
  # 매 step:
  speed_xy = sqrt(vx² + vy²)
  if speed_xy < 1.0:
      consecutive_still += 1
  else:
      consecutive_still = 0
  max_consecutive_still = max(...)

  # Episode 종료 시 (truncated AND not dropped):
  if max_consecutive_still > 200:
      reward -= 15
  ```

**핵심 장점**:
  - Per-step 보상 density 변화 없음 → SAC 안정성 유지
  - 1회 terminal 페널티 → 발산 모드 회피
  - Drop 시 제외 → 정밀 hover 정당화
  - 잠시 hover (50 step) OK → 학습 초반에도 무력화 안 함

**유지** (Round 3+4 안전장치):
  - PER (alpha=0.6, eps=0.1, priority_max=30)
  - learning_rate=1e-4, tau=0.002
  - Sigmoid alt penalty, max_alt truncate, hard cap
  - PX4 로깅 비활성

**결과**: (학습 중 — 300k)

**결론 / 다음**: ...

---

## 5. Entry 추가 template

```markdown
### #N. YYYY-MM-DD — <한 줄 제목>

WandB run: `<id>` (online/offline)
**Base**: <#N or branch>

**변경**:
- `<section>.<key>`: <old> → <new>
  - **이유**: ...

**추가** (NEW):
- `<section>.<key>` = <value>  (또는 코드 측 callback/메서드)
  - **이유**: ...

**제거** (REMOVED):
- `<section>.<key>`  (또는 폐기된 코드 patch)
  - **이유**: ...

**결과**:
- <metric>: <value>
- ...

**결론 / 다음**: ...
```

§2 (current snapshot) 와 §3 (요약 표) 도 함께 갱신.

---

끝.
