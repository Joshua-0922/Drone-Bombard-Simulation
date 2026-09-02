# RL Training Pipeline — Project Log

> **Branch:** `main` | **VM 프리엠션 후 컨텍스트 복구용 로그**

---

# 1. Current State

**2026-09-02 — 일반화 감사 완료. 두 축 통과, 한 축 실패.**

exp_028이 남긴 미측정 축 세 개를 닫았다 (arm당 600 짝지은 에피소드, DR 1.5).

**✅ 축 R — 미지 사거리 26–30 m (적합은 18–22 m):**

| n=600 | 성공%@1.0 | 성공%@0.5 | CEP50 | CEP90 | 배달률 |
|---|---|---|---|---|---|
| L0 | 73.00 | 60.67 | 0.299 | 0.617 | 73.83% |
| **L1-SL (tilt, EMA 0.3)** | 73.33 | **67.83** | **0.218 (−27.3%)** | 0.469 | 73.83% |
| 오라클 [천장] | 73.83 | 69.83 | 0.204 (−31.7%) | 0.409 | 74.17% |

오라클 회수율 **85.3%** — 훈련 범위(85.0%)와 동일. **기하 암기가 아니라 바람 추정이다.**
배달률이 세 arm 모두 73.83%로 같아, 잔차가 비행을 건드리지 않음이 재확인됐다.

**⛔ 축 P — 정책 전이 (seed 1 회귀기 → seed 2 정책): 순손실**

| n=600 | 성공%@1.0 | CEP50 | CEP90 | 평균오차 | 회수율 |
|---|---|---|---|---|---|
| L0 seed 2 | 96.00 | 0.274 | 0.542 | 0.305 | — |
| **전이 (seed 1에서 적합)** | **89.33** | 0.255 (−7.0%) | **0.830** | **0.361** | 22.9% |
| **재적합 (seed 2 데이터)** | 92.50 | 0.214 (−22.0%) | 0.562 | 0.294 | 72.3% |
| 오라클 [천장] | 94.33 | 0.191 (−30.2%) | 0.432 | 0.247 | 100% |

**obs→drift 사상은 물리가 아니라 정책의 성질이다.** tilt가 재는 것은 바람이 아니라
$\bar\theta \approx F_{wind}/K_{policy}$ 이고 $K$는 컨트롤러마다 다르다.
정보 문제가 아님의 증거: 오라클은 같은 정책에서 −30.2%, seed 2의 회귀 $R^2$는
오히려 높고(0.631), 재적합하면 꼬리가 복구된다. → **정책마다 재적합 필요 (Rule 39).**

**✅ 축 N — 라벨 수: 1,000회면 이득의 96%**

| 학습 ep | 100 | 200 | 500 | 1000 | 2334 |
|---|---|---|---|---|---|
| $R^2$ (obs+tilt) | 0.382 | 0.463 | 0.537 | 0.573 | 0.611 |
| $R^2$ (obs+**참바람**) | **0.982** | 0.991 | 0.995 | 0.997 | 0.998 |
| CEP50 Δ vs L0 | −5.5% | −14.8% | −23.8% | **−30.1%** | −31.4% |
| succ@1.0 (L0 93.83) | **87.33** | 89.33 | 90.00 | 92.50 | 93.50 |

**오라클과의 간극은 데이터가 아니라 정보다** — 참바람을 주면 100 ep에서 이미 0.982.
배증당 증가폭 0.081→0.032 감쇠, 점근선 0.65–0.70. **실기 예산 = 1,000회 투하.**

⚠️ **새 함정 (Rule 40):** 잘못 보정된 잔차는 **중앙값보다 꼬리를 먼저 망친다.**
n=100 arm은 CEP50 −5.5%(개선)인데 CEP90 +43%, succ@1.0 −6.5 pp — 전이 arm과 같은 서명.
**CEP50 단독으로 잔차 arm을 판정하지 말 것.**

**L1-RL 재평가:** 축 P는 RL의 실질적 장점(공동 적응 → 구조적으로 정책 결합 없음)을
만들어 준다. SL이 본선이라는 결론은 유지하되, 비교 절에 이 장점을 적어야 한다.

wandb: `sl_gen_unseenR` / `sl_gen_policy_transfer` / `sl_gen_label_count` (job_type=eval)

→ [[experiments/exp_029_l1_sl_generalization]] · [[research/residual_policy_coupling]] ·
[[research/residual_label_efficiency]] · Rule 39, 40

---

**2026-09-01 — 논문의 학습 행이 채워졌다. PPO 없이.**

지도학습 잔차(L1-SL): 동결 L0 + 오프라인 회귀만으로 **CEP50 −31.4%**(DR 1.5) /
**−32.9%**(DR 2.5), 특권 오라클 천장의 **85% / 78%** 회수. 보상 코드 미변경, L0 재학습 없음.

| DR 1.5, n=600 | 성공%@1.0 | 성공%@0.5 | CEP50 | CEP90 | 배달률 |
|---|---|---|---|---|---|
| L0 | 93.83 | 78.83 | 0.305 | 0.593 | 94.50% |
| **L1-SL (obs+tilt, EMA 0.3)** | 93.50 | **86.50** | **0.209 (−31.4%)** | **0.464** | 94.67% |
| 오라클 wind-only [천장] | 94.33 | 90.83 | 0.192 (−37.2%) | 0.392 | 94.67% |

관측 가능성 회귀 $R^2$ (반출 에피소드): 상수 0.000 · linear(obs) 0.127 ·
**MLP(obs) 0.436** · **MLP(obs+tilt 누적) 0.611** · MLP(obs+참바람) **0.998**.
→ 관측은 바람에 **장님이 아니고**, 정보는 **비선형**으로 들어 있다.

**tilt 채널에 L0 재학습은 필요 없다** — 지도 잔차는 정책과 분리된 네트워크라
자기 입력을 만든다(정책 26채널 / 잔차 36채널). RL 잔차라면 Fresh Start가 강제된다.

⚠️ **새 함정:** 결과 공간 잔차는 정확할 뿐 아니라 **시간적으로 매끄러워야** 한다.
릴리즈 게이트가 첫 교차 판정이라 예측 요동(0.073~0.129 m/step vs 참값 0.018)이
**계통 편향**으로 바뀌어 CEP를 2.5배 악화시킨다. EMA α=0.3으로 0.462 → 0.209.

**L1-RL의 위치 변경:** 필수 경로 → "RL이 지도학습을 넘어서는가" **ablation**.

→ [[experiments/exp_028_l1_sl_pilot]] · [[research/residual_observability]] ·
[[research/release_gate_jitter]] · Rule 37, 38

---

**2026-08-30 (오후 재측정 이후)** — 평가 120회 완주, 실패 0건.

## Table 1 (seen, DR 1.5, 팔당 n=600, 평가 시드 3개)

| 팔 | 배달률% | 배달시간 s | 착지오차 m | CEP50 | CEP90 | 투하 v | 고도 |
|---|---|---|---|---|---|---|---|
| T0 hover (3.5 m) | 74.8 | 8.56±1.25 | 0.445±0.318 | 0.359 | 0.907 | 0.31 | 3.52 |
| **T2@1.5 (3.5 m)** ★ | 76.2 | 8.09±1.52 | **0.330±0.219** | **0.287** | 0.623 | 1.42 | 3.53 |
| T3 참값 (6.0 m) | 75.8 | 6.51±1.27 | 0.417±0.287 | 0.349 | 0.840 | 2.81 | 6.04 |
| **L0 (제안)** | **94.0** | **5.80±1.14** | 0.337±0.208 | 0.299 | **0.596** | **3.54** | 3.53 |
| L0 + 완벽한 잔차 | 94.2 | 5.83±1.18 | **0.207±0.211** | **0.156** | **0.378** | 3.55 | 3.59 |

★ = baseline의 최적 구성 (투하 고도까지 튜닝)

**정확도는 동률, 배달 시간 −28%, 배달률 +17.8 pp, 투하 속도 2.5배.**

## 확정 사항

- **지표**: 헤드라인 = 배달시간 · 착지오차 · CEP50/CEP90 · 배달률.
  `success@1.0`은 부록(반경 곡선과 함께). 보상 `success_radius`는 1.0 유지 — 재학습 없음
- **DR 축**: CEP 격차 −0.027(DR 0) → +0.180(DR 2.5) **단조 증가**. DR=0에서는 모델 기반이 이긴다
- **잔차 상한선**: 바람만 아는 잔차 **CEP50 −31%**(DR 2.5 −35%). 전지적 −48%는 도달 불가
- **학습 시드**: 2개 (92.7 / 93.8%). seed 3은 60/1000 중단
- **L0 = throw**: 투하 3.54 m/s · 고도 3.53 m · $v_z$ −1.17 · 전진 3.32 m

## ⛔ 철회 5건 (2026-08-30 오후)

| 철회 | 왜 |
|---|---|
| 릴리즈 속도 1.39 m/s | `final_speed_xy`가 **착지 시점** 값. 실측 3.54 m/s (Rule 33) |
| "감속해서 던진다 / 투하속도 보상항 필요" | 감속은 투하 *후*. 막을 대상 없음 |
| "6개 팔 전 축 파레토 지배" | baseline 투하 고도 미튜닝. 고도만 낮춰도 T2 CEP50 0.457 → 0.287 |
| "오라클급 바람 강건성 도달" | 기하가 78%. 같은 정책 위 바닥은 0.0056 — 바람 오차 92% 잔존 (Rule 35) |
| "T3 55.5%" | 4 m/s 패스 점. 동일 조건에서 72.5% |

## 다음

1. **L1 파일럿 2팔** (동결 / 비동결). 중단 규칙: 500 iter까지 CEP50 −15% 미달 시 중단
2. Table 1에서 T1 또는 T2@3.0 제거 (100 Hz에서 동일 팔)
3. unseenR 추락 26.8% — 비행 제어의 사거리 외삽 실패 조사

# 2. Recent Progress

## 2026-09-02 — L1-SL 일반화 감사 (exp029)

- **주입 평가 39 run** (세 축 36 + 전이 확증 3), arm당 600 짝지은 에피소드,
  평가 시드 {3000,4000,5000} — 적합 시드 {1000,2000}과 서로소.
- **오프라인 적합 6종** — 라벨 수 곡선 5종(`--train_ep` 100…2000) + seed-2 자체 대조군.
- **코드:** `_fit_sl_residual.py --train_ep` / `_agg_sl_eval.py` arm 자동탐지 + `--wandb` /
  `_sl_gen.sh` 신설. paired eval에는 wandb 훅이 없어(비-paired 경로 전용)
  **집계 단계에 붙여 실험당 run 1개**로 처리.
- **결론 3건:** 기하 축 일반화 확인 · 정책 결합 발견(Rule 39) · 간극의 정체 = 정보(Rule 40).
- **DAgger 우선순위 최상위로 상향** — 축 P가 분포 이동의 파괴력을 실측으로 보여줌.

- **2026-09-01 (Isaac Lab, exp_028) — 잔차를 강화학습이 아니라 지도학습으로: PPO 없이 CEP50 −31.4%.** 08-30에 결정한 학습 목표($\delta^* = x_{real} - x_{nominal}$)를 실행. **Stage 1** 동결 L0를 추론만 굴려 `(관측 26, 참 드리프트 2)` 덤프(`play.py --dump_sl`, 7,008 ep / 42만 프레임, 라벨은 기존 `oracle_impact_residual(wind_only=True)` 재사용). **Stage 2** 오프라인 MLP 회귀, **분할은 에피소드 단위**(프레임 분할은 같은 바람 추출이 train/test에 걸쳐 라벨 누수). **Stage 3** 오라클과 **동일 경로**로 주입(`--sl_residual`), paired 200 ep × 36 run, 평가 시드 {3000,4000,5000}은 적합 시드 {1000,2000}과 서로소. **결과:** $R^2$ — linear(obs) 0.127 / MLP(obs) **0.436** / MLP(obs+tilt) **0.611** / MLP(참바람) **0.998**; CEP50 0.305 → **0.209 (−31.4%)**, DR 2.5에서 0.498 → **0.334 (−32.9%)**(회귀기는 DR 1.5로만 적합 — 외삽). **판정 3건:** ① 관측은 바람에 장님이 아니다(정보는 비선형으로 존재 — `--freeze_nominal`의 선형 판독이 왜 용량 부족이었는지와 정합). ② tilt 누적은 +17.5 pp인데 **L0 재학습 불필요** — 지도 잔차는 별도 네트워크라 정책이 보지 않는 입력을 먹는다. ③ **정확도만으로는 부족하다** — 게이트가 첫 교차 판정이라 예측 요동이 계통 편향("항상 일찍")이 되어 CEP를 2.5배 악화시킨다(참 드리프트 0.018 m/step vs 예측 0.073~0.129); EMA α=0.3으로 0.462 → 0.209, 단 정확도 부족은 못 메운다(obs 전용은 EMA에도 0.329 > L0 0.305). 부수: 릴리즈 프레임의 드리프트 RMS가 전 구간 대비 **0.643 → 0.341 m**로 반토막 — L0 릴리즈 기하의 프레임 단위 확인. `success@1.0`은 또 안 움직였고(93.83→93.50) `success@0.5`는 78.83→**86.50** — 지표 결정 3회차 확인. 함정 2건: `residual.enabled` 기본값이 True라 `--no_residual` 없이 평가하면 L0 성공률이 92.7→37.5%로 보임; rsl-rl은 관측을 TensorDict로 넘김. → [[experiments/exp_028_l1_sl_pilot]] / [[research/residual_observability]] / [[research/release_gate_jitter]] / Rule 37, 38
- **2026-08-01 (repo housekeeping) — `main`을 `isaac_jk`로 승격 + `Isaac-JS` 브랜치 정리.** 10개 브랜치 전수 계보 조사로 `main`이 07-03 시점(Isaac Lab skeleton)에 정체돼 있고 실제 작업은 전부 `isaac_jk`에 있음을 확인. `Isaac-JS`(제균 개인 브랜치, 07-02 이후 Isaac Lab 코드 없이 Gazebo/SAC 문서만 추가)의 고유 연구노트(v15 회귀 진단, 보상함수 리뷰, Rule 25/26)를 `isaac_jk`로 포팅 후 브랜치 삭제. `main`을 `isaac_jk`로 force-update(구 `main`은 태그 `archive/main-pre-isaac_jk-promotion`으로 보존 — junsang `_junsang` 노트·초기 `isaac_lab_tasks/` 스켈레톤 포함). push 용량초과 보고의 실제 원인도 규명: `Isaac-JS` 마지막 커밋의 `git add .`가 SAC replay buffer/영상 ~166MB를 실수로 포함시킨 것(브랜치 삭제로 해소, repo 전체 히스토리 bloat는 별도 과제). `Issac_JS`(junsang)는 미변경 — 세션 중 junsang의 신규 커밋(v20 task 등록)이 아직 `main` 미반영. → [[daily/daily_2026-08-01]]
- **2026-07-21 (Isaac Lab, exp_019) — 물리 페이로드 attach/detach: kinematic weld 구현 + hover-drop 검증 4/4 PASS.** 코드 전수 검토에서 릴리스 경로 결함 6종 발견(핵심: `_payload_attached`가 발화 시 False로 전환되지 않는 죽은 플래그 + 페이로드 자체가 물리적으로 부재). per-env RigidObject 페이로드 신설 — 부착=매 physics step pose+vel write(조인트 불가 제약 우회), 분리=CCIP 발화 후 release_delay(0.1 s) 카운트다운 만료 시 write 중단, 착탄=z≤0.10 m 래치로 측정 오차 기록. 검증: 추적 1.1 mm/분리 0 잔류/착탄 8/8/**측정 vs 해석적 |Δ| max 0.021 m**. 보상·종단 bit-identical, 기존 마커 env-0 버그도 수정. `play.py`에 payload_impact 통계 추가. → [[experiments/exp_019_physical_payload]] / [[research/physical_payload_attach]] / Rule 24
- **2026-07-03 (Isaac Lab migration) — README 컨테이너 진입 절차 수정 + `play.py` 4-tuple 버그 fix.** 사용자가 혼자 재현 시도 시 실패 원인 규명: README의 `docker pull drone-bombard-isaac:latest`가 가리키는 이미지는 로컬에도 GCP Artifact Registry(`isaac-lab` 저장소, 0 items)에도 **존재하지 않음** — pull 대상이 없었음. 실제로는 `isaac-lab-local:580` 이미지로 띄운 `isaac-verify` 컨테이너가 이미 dev VM에 떠 있었음. README §5에 "이미 떠 있는 컨테이너 재사용" 절 신설 + non-root exec 시 root 소유 캐시(`/isaac-sim/kit/cache` 등)로 인한 `PermissionError` 크래시 및 chown 해결법 문서화 + `PYTHONUNBUFFERED=1` 팁(미설정 시 `simulation_app.close()`의 하드 종료로 마지막 PASS/FAIL print 유실) + README 전체의 `./isaaclab.sh`(존재하지 않는 상대경로, `/workspace/drone-bombard`에서 cwd 불일치) → `/workspace/isaaclab/isaaclab.sh` 절대경로로 일괄 수정(14곳). 검증 중 `isaac_lab/play.py`의 `run_zero_actions`/`run_scripted`/`run_policy`가 `RslRlVecEnvWrapper`(rsl_rl 4-tuple `obs,rew,dones,extras` API) 사용 중임에도 5-tuple(`obs,rew,terminated,truncated,info`) unpacking을 시도해 **항상 `ValueError`로 즉시 크래시**하던 버그 발견·수정. 수정 후 `--zero-actions`는 실행은 되나 altitude drift 12m/100 step로 FAIL(`verify_one_episode.py`는 동일 env로 148-step 안정 호버 PASS) — wrapper 경로 자체의 미해결 회귀 가능성, 후속 조사 필요. → [[isaac_container_access]] (Claude memory)
- **2026-07-03 (Isaac Lab migration) — 실제 실행 검증 통과 (VERIFY: PASS).** 사용자 요청으로 이 dev 박스에 `isaac-sim:5.1.0` pull → Isaac Lab v2.3.2+rsl_rl 설치 → `verify_one_episode.py`(신규 무학습 하네스)로 `Isaac-DroneBombard-Direct-v0` **1 에피소드 실제 실행**. env 구성·USD 씬(드론)·질량 오버라이드(2.173kg)·reset(obs (1,14))·**148스텝 안정 호버**(고도 유지, 중력보상)·obs/reward/termination 유한(NaN 0)·stagnation guard 정상 발동. **실행으로만 잡히는 env/컨트롤러 버그 5종 수정**(핵심: rate loop 토크에 관성항 누락 → ~46× 과토크 → 즉시 스핀아웃; `τ=I·(k_rate·rate_err)`로 수정). + 이미지 자체 버그 2종(dangling `_structures.py` 심링크, core isaaclab 미설치) Dockerfile 반영. **렌더링/GUI는 driver 535<580으로 이 박스에서 불가** — 물리/CUDA 정상, 시각화는 L4 VM 필요(사용자: headless 검증 수용). 커밋 `f2f1b1a`. → [[experiments/exp_012_isaac_migration_phase2]] §6b / [[research/isaac_velocity_controller]]
- **2026-07-03 (Isaac Lab migration, `feat/isaac-env-migration` 브랜치):** **Phase 2 — env+PPO 코드 이식 완료.** 별도 워크트리(`git worktree add /opt/drone-bombard/isaac-worktree feat/isaac-env-migration` + `git merge jekyun`, merge `940c88b`)에서 진행 — jekyun의 라이브 v15 학습(tmux `rl_train`) 방해 없음. `isaac_lab/` 신설: `math_utils.py`(action rate-limit/LPF, pinhole vision projection, hold-buffer, ballistic/CCIP, 3-layer reward, overshoot/stagnation guard — 순수 torch, isaaclab 무의존) + `drone_bombard_env.py`(DirectRLEnv, 위 math_utils를 isaaclab lifecycle에 연결) + `agents/rsl_rl_ppo_cfg.py` + `train.py`/`play.py`/`yolo_eval.py`. v13/v15 obs(14)·action(4)·reward·termination 상수 전부 parity 이식(표: [[experiments/exp_012_isaac_migration_phase2]]). SAC→PPO(rsl_rl), target/spawn 랜덤화 신규(Gazebo는 고정 타겟), vision=학습 시 analytic pinhole(YOLO 캘리브레이션 노이즈)+평가 시 실제 YOLOv8 이원화, drop=액션 아닌 스크립트 CCIP 메트릭(태스크 스코프 v15와 동일). Phase-2 훅 4종(CCIP residual, release 상수 cfg화, obs superset 고정, domain-rand 스텁) 비활성 wiring — Phase 1 출력 불변. **검증:** `pytest isaac_lab/tests/test_math.py` **29/29 통과**(drone-bombard-harmonic 컨테이너, torch 2.4.1, isaaclab 미설치 — 파일 경로 직접 로드로 패키지 `__init__` 우회). `py_compile` 전체 통과. L4 Spot VM 미기동 → env 스모크·실제 학습 미실행(README에 정확한 커맨드 문서화). 부수 발견: Gazebo `hyperparams_v13.yaml`의 `limit_tilt:0.26`는 코드에서 미사용인 죽은 설정(실제 게이트는 `limit_inverted_tilt=1.047` 기본값) — 이식 안 함. Overshoot guard가 success_radius=0.8에서 도달 불가능한 것은 Rule 10의 의도된 설계임을 Gazebo 소스로 재확인(버그 아님) — 비종단 진단 카운터만 신설. 신규 **Rule 16**(시뮬레이터 이식 시 plant/reward parity는 상수뿐 아니라 타이밍+메커니즘까지 검증). → [[experiments/exp_012_isaac_migration_phase2]] / [[research/isaac_velocity_controller]] / [[research/rl_rules]] Rule 16
- **2026-07-05 (Gazebo/SAC 트랙, isaac_jk 분기 전 마지막 기록) — v15(310K) "X마커 미도달" 진단 (코드 변경 없음).** 사용자 관찰: wobble 교정판 모델이 이전 모델(v14)과 달리 X마커에 도달 못 함. 학습 이력 재구성 결과 v15는 잘못된 base config로 첫 크래시 후 수정, 이어서 **반복 reset-recursion abort**로 오토레쥼 서포바이저 추가, 0→310K에서 preempt — **그 이후 정식 eval이 한 번도 기록되지 않았음**을 확인. 원인 후보 2건(미확정): ① `vel_damp_radius=3.0m`가 `success_radius=0.8m`보다 넓어 근접-속도 댐핑(B)이 v14의 기존 final-approach stagnation 구간(0.5–1.2m)을 재타격했을 가능성 ② `run_train_supervised.sh`의 resume이 가중치만 복원하고 replay buffer는 매번 초기화 — 반복 abort가 있었다면 310K 스텝만큼의 연속 학습이 실제로는 없었을 가능성. **진단 중 GPU 드라이버 불일치(호스트 580 vs `drone-bombard-harmonic` 컨테이너 NVML 535, Isaac Lab 드라이버 업그레이드 여파) 발견 — evaluate.py 실행 자체가 reset-recursion으로 실패해 원인 확정이 차단됨.** 이 트랙은 인프라 정합 없이 여기서 멈춤(이후 이 워크트리는 Isaac Lab 전용). → [[daily/daily_2026-07-05_gazebo_v15_regression]] / [[experiments/exp_011_wobble_lpf_reward_damping]] / [[research/rl_rules]] Rule 25 / Rule 26
- **2026-06-23:** **v14(byxyaf4d) plateau stop + 195K eval = 65% + soft reset 장기검증 통과.** ep_rew_mean이 70K부터 ~120–135 평탄(125K step 정체) 확인 → **196.5K/500K(~39%)에서 SIGINT graceful stop**(`sac_drop_195000_steps.zip` 보존, sim 점유 해제). **clean 20-ep deterministic eval = 성공 65%(13/20)** — v13 80%(16/20) 대비 **회귀**. 실패 7 전부 **final-approach stagnation**(~0.5–0.8m 접근 후 0.50m gate 직전 정체; min 0.52/0.63/0.70/0.79/0.79/1.09/1.19, ep18은 2cm차). 항법·탐지 아닌 **종말 거리 좁히기 약점.** EKF 귀책 실패 0(eval중 health-gate EKF divergence 2회 전부 full-restart self-heal 후 성공). **Soft reset 장기검증 ✅:** stop 직전 env `attempts=3096 success=2826 skipped=118`(soft ~91%, no teleport/no PX4 restart) → exp_009의 미해결 질문(학습 정책 fallback율·EKF drift bounded) **해소, Rule 14 검증완료.** **회귀 원인=정책 미성숙**(39% budget, terminal-tightening은 막판 sharpen 스킬; reward plateau는 거친 정책의 평탄). 추가 가설: 10m 고도(v13 5m)로 최종 하강↑. **비디오:** `record_flight.py`+3-ep evaluate 동시 → 3/3 success(0.48–0.50m) 캡처 → `rl_eval_results/v14_195k_flight_annotated.mp4`(2.1MB, YOLO 박스) + `_raw.mp4`(3.7MB). headless `gz sim -s`라 onboard annotated가 산출물. **v14 commit 결정 대기**(soft reset 인프라 채택 vs 65%<80% baseline 채택). → [[experiments/exp_010_byxyaf4d_v14_195k_eval]] / Rule 14
- **2026-06-22 (오후):** **리셋 처리량 ~3.9× — soft reset로 teleport-EKF 병목 회피.** v14 fps≈2, ETA ~2.5일의 원인을 계측으로 확정: 에피소드마다 CRUISE timeout(~42s)+full restart(~22s)이고, 근본은 teleport+disarm 후 PX4 **EKF 추정기 재수렴** 대기(`ctrl_0.log: Delaying arm — pre_flight_checks_pass=False`). **fresh restart도 동일 timeout**(restart≈handoff). **EKF2_GPS_CHECK 0 A/B = 음성**(airframe가 이미 `COM_ARM_WO_GPS 1` → GPS는 게이트 아님; 실제 게이트=EKF validity/innovation 수렴, 바이패스 param 설계상 없음) → param/timeout 레버 고갈 규명. **해결 = soft reset**(`drone_drop_env.py _try_soft_reset`): 종료 시 flyable이면 disarm/teleport 없이 armed+airborne 유지하고 position setpoint로 출발점 복귀(controller 살려둬 offboard heartbeat 유지) → mission_manager FSM만 재시작 → 재핸드오프. EKF 미교란 → 재수렴 0. **프로토(9.2min/32회): throughput 0.93→3.61 handoffs/min(~3.9×), fps 2→9, reset 65s→11s, soft 성공 32/32(100%), EKF d_xy 안정 4.5–5.8m(발산 없음).** flyable 아니면 기존 teleport+restart fallback(downside bounded). full run `rl_yolo_v14_softreset`(byxyaf4d, fresh 500K, online) 기동 — 장기 EKF drift/fallback율 검증 중. **미커밋.** → [[experiments/exp_009_softreset_throughput]] / [[research/reset_throughput_bottleneck]] / Rule 14
- **2026-06-22:** **핸드오프 윈도우 확장 — 고도는 레버 아님, 탐지 게이트가 진짜 레버.** 사용자 요청("X마커가 늦게=거의 머리 위 탐지돼 RL 핸드오프 후 학습 윈도우가 짧다"). **시도 1(고도만 10 m):** `target_altitude` 5→10 + `start_drift_max` 5→10 정합. **실패** — clean 핸드오프 여전히 d_xy 2.7 m(베이스라인 동급), 3 ep 중 2가 순항-시작 spurious(conf=0.00, d_xy≈11 m, 넓어진 FoV의 X-like 지면 FP) → health gate 발동/abort. 원인: 마커 apparent size ∝ 1/고도(YOLO 늦게 lock) + `vision_callback` 200 px 필터가 핸드오프를 머리 위로 클립. **시도 2(10 m + 탐지 게이트):** `vision_callback`에 confidence 게이트(`min_detection_conf=0.5`; real 마커 conf 0.73–0.95 vs 지면 FP ≤0.45) + 공간 필터 200→300 px(`detection_pixel_radius`; real off-center 264–293 px 조기 accept). **dry-run PASS: 핸드오프 d_xy 2.7→5.0–5.2 m(윈도우 ~2배), spurious 0, EKF-drift 0, conf 0.93.** dry-run 격리(`--checkpoint-dir rl_dryrun_alt10` + offline)로 메인 체크포인트 보호. **코드 변경 미커밋**(사용자 직접 커밋 예정). 기하+탐지 변경이라 fresh 필수 아니나 핸드오프 3.5→5 m 변화로 정책 초반 재적응 예상. → [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13
- **2026-06-21:** **eval 발산 흡수 루프 근본 원인 규명 & 수정 3종 (health gate + YOLO 누수 + evaluate.py).** 06-20 발견한 EKF↔camera 발산 루프의 진짜 원인 = **YOLO `xmarker_detector` 누수**: `_start_infra`가 fresh-start마다 YOLO 노드를 죽이지 않고 새로 spawn → 누적(검증 시 3개) → 충돌하는 pixel_coords 발행 → spurious CRUISE→TRACKING(conf=0.00) + EKF↔camera 불일치. **fundamental EKF 버그 아님** (clean slate에선 handoff 0.9m 정상). **수정:** ① `drone_drop_env.py` reset() step 8b **health gate**(`d_xy_prev>start_drift_max(5.0)`면 full restart+progressive settle 후 retry, max_retries(6) 초과 시 loop 대신 abort), ② fresh-start kill 리스트에 `xmarker_detector` 추가(누수 차단), ③ `evaluate.py` 재작성(success_rate/step-to-reach/closest d_xy from obs[12,13]/outcome breakdown; 죽은 `info['drop_error_actual_m']` NaN 의존 제거). config `hyperparams_v13.yaml`에 `start_drift_*` 추가. **Dry-run PASS(clean slate, 3/3 SUCCESS, gate 0회, mean reward 162, report NaN 없음).** `--symlink-install`이라 src 편집 live, rebuild 불필요. → [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12
- **2026-06-20:** **v13 정책 평가 + 학습 stop.** iyhfy5ps가 157.7K(~32%)에서 **plateau**(ep_rew_mean ~100 80K부터 평탄, success ~82%, target_lost 0, fps≈0). eval을 위해 학습 **SIGTERM graceful stop**(preempt+70MB replay 보존, 재개 가능). deterministic eval(`sac_drop_preempt.zip`, 20-ep 요청/13 실행): **ep 1–3 전부 0.8m 성공(reward 126/114/132, step 41–72; 평균 124 > 학습 ~100) — 정책 양호.** **ep 4–13 전부 step1 EKF divergence(`d_xy≈11.9m`=home→target) → −15 truncation 흡수 루프** (연속 full-restart가 EKF 수렴 못 시킴; 카메라는 마커 봄=TRACKING OK이나 EKF position만 발산; 06-17 EKF 재수렴과 동일 뿌리, eval에서 누적 악화). **harness 결함 2종:** `evaluate.py`가 env 미emit 키 `info['drop_error_actual_m']` 의존 → miss-distance/CEP/drop-speed 전부 NaN; v13 env는 0.8m 성공원 종료(탄도 투하 미모델링) → CEP 비실재 → success-rate/step-to-reach로 평가해야. **다음:** 에피소드 시작 EKF↔카메라 health gate(drift면 retry) + evaluate.py 지표 교체 후 재평가. → [[experiments/exp_007_iyhfy5ps_v13_eval]] / [[research/eval_terminal_env_metrics]] / Rule 12
- **2026-06-17 (오후):** **v13 fresh 재시작(iyhfy5ps) + 인시던트.** 30K 재개를 시도했으나 armdiag dry-run이 **YAML 중복 `checkpoint_dir` 키**(격리 경로가 main에 덮임)로 메인 dir에서 실행되어 v13 30K 체크포인트 5개 삭제 + preempt를 599-step으로 덮음 → **30K 디스크 복구 불가**. 사용자 결정으로 **fresh 재시작**(arm_bail=20, 0→500K). 프로덕션 검증: 초기 윈도우 **bail 0, late-EKF 14.1/14.8/15.5s ×3 전부 회복**(구 10s면 30% bail). 재발방지: 파괴적 fresh-start 전 startup `Checkpoints:` 로그로 격리 검증. → [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]]
- **2026-06-17:** **arm_bail 처리량 병목 진단 & 수정.** v13(46y4xtiw)이 ~10h에 6%(29.9K)뿐, ETA ~6.5일 — 지배적 싱크가 `PX4 not armed after 10s` bail임을 확인. 컨트롤러에 `PREFLIGHT-PASS` dt 계측 추가 후 격리 dry-run(`hyperparams_v13_armdiag.yaml`, `arm_bail_timeout=25s`, offline). **결과: EKF 재수렴 bimodal — 0.0s(7/12) / 13–16s(5/12 ≈ 42%), 25s에서 bail 0 / SUCCESS 4.** v12의 10s 컷이 recoverable-with-time을 full-restart-only로 오판하고 복구 직전(3–6s 전) 단두대질했음. **Fix: `hyperparams_v13.yaml` arm_bail_timeout 10→20.** v13은 SIGTERM emergency save로 30K+리플레이 보존, 재개 가능. → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11
- **2026-06-15:** **Arming-rejection throughput fix.** `rl_yolo_v11_cam_fix`(k1uqgs8i) 분석 → 443 CRUISE 타임아웃의 근본 원인이 teleport 후 stale EKF arm 거부(28.2% NEVER ARMED, 전부 attempt 1/3)임을 규명. 수정 3종 적용: (#3) `/fmu/out/vehicle_command_ack` arm 거부 사유 로깅, (#2) `pre_flight_checks_pass` 게이팅, (#4) `arm_bail_timeout=10s` early-bail → 즉시 full infra restart. colcon build clean + dry-run(400 step, 0 타임아웃) 검증 후 fresh run `rl_yolo_v12_arm_fix`(500K) 기동. ⚠️ 정정: `cruise_poll_timeout`은 이미 20.0s(이전 "60s"는 fallback 기본값 오독). ⚠️ OPEN: YOLO target_lost_rate ~29% bimodal 미해결. → [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]]
- **2026-06-13:** v7 패치 적용 후 fresh run `rl_yolo_v7_drift_guard` (WandB: `7lhjy40o`) 시작. EKF drift guard (step1 d_xy>5m→truncate), proximity 4m→2.5m, penalty_target_lost -0.5→-0.1, stagnation_start_step 400→50.
- **2026-06-12:** Vision 기반 RL 학습 인프라 완성. EKF East 반전 버그 2종 수정. fresh run `rl_yolo` (WandB: `45l8vkw5`) 121K steps. **분석: target_lost_rate=1.0 원인 = EKF drift (dominant) + 카메라 FOV gap 3차진 이후 2.89m vs 시작 d_xy 3.5m).** run 폐기.
- **2026-04-16:** RTF dry-run 3종 완료 (RTF 1/2/4). **RTF=2 최적** (avg 59.5 fps, 61s/4Kstep). RTF=4는 Python 병목으로 역전. Exp 002 RTF=2로 결정.
- **2026-04-16:** WandB API key 영구 연결 (`/opt/drone-bombard/.wandb.env`, `--env-file` 방식). Docker image `drone-bombard-px4built:latest` — PX4 빌드 + 커스텀 airframes 4016-4019 포함.
- **2026-04-14:** Obsidian 연구 비서 시스템 초기화. `notes/` 구조 구축, CLAUDE.md + RL_Project_Log.md 간소화.
- **2026-03-22:** 보상 해킹 분석 → 4개 anti-milking 패치 적용 (학습 대기 중).
- **2026-03-20:** 선형 거리 보상 도입 (지수 포텐셜 교체), CRUISE retry, 3중 물리 폭발 방어.
- **2026-03-20:** Method A (1-World-4-Payload) 아키텍처 완성 및 dry-run 통과 (31 fps).
- **2026-03-19:** 자기관리 인프라 안정화 (z=0 스폰, COM_OF_LOSS_T=10s, fps=30-31).

---

# 3. Remaining Tasks (Next Steps)

## 2026-09-02 기준 우선순위

| # | 무엇 | 왜 | 비용 |
|---|---|---|---|
| **1** | **DAgger 1바퀴** (잔차 켠 채 재수집 → 재적합) | 축 P가 분포 이동의 파괴력 실증 → **최상위로 상향** | ~30분 |
| **2** | 조준 보상 잔차 포함 결함 수정 | **L1-RL 전 필수** | ~30분 |
| 3 | 적합 손실에 시간 평활 항 | EMA의 원인 처치 | ~20분 |
| 4 | seed 2 재수집(3,500 ep) 후 재적합 | "라벨 부족 vs 여유 부족" 구분 | ~10분 |
| 5 | 3-seed 각각 재적합 → 분산 보고 | Rule 39가 seed별 재적합을 요구 → 논문 표 변경 | ~40분 |
| 6 | OU(시변) 바람 ablation | tilt 누적의 전제 — 남은 마지막 한계 | ~1시간 |
| 7 | Table 1에서 T1 또는 T2@3.0 제거 · `research_architecture` §7 갱신 | 100 Hz 판정에서 동일해진 arm | ~20분 |

## 2026-09-01 이후 (exp_028 결과 반영)

| 순위 | 작업 | 상태 |
|---|---|---|
| 0 | 적합 손실에 **시간 평활 항** $\lVert\hat\delta_t-\hat\delta_{t-1}\rVert^2$ (EMA는 사후 처치) | ❌ |
| 0 | **조준 보상의 잔차 포함 결함 수정** (`task_env`에 exp_018 가드 복원) — L1-RL 전 필수 | ❌ |
| 1 | **L1-RL 파일럿** (frozen / unfrozen) — 이제 ablation. 잔차 채널 평활 페널티 포함 | ❌ |
| 1 | Table 1에서 T1 또는 T2@3.0 제거 (100 Hz 해상도에서 동일한 팔) | ❌ |
| 2 | `research_architecture.md` §7 갱신 | ❌ |
| 2 | unseen-R 추락률 26.8% 조사 | ❌ |
| 3 | (선택) L0 학습 seed 3 | ❌ |

## 2026-08-23 이후 착수 순서 (research_architecture v3 §11)

| 순위 | 작업 | 상태 |
|---|---|---|
| 0 | CCIP `vz` 수정 | ✅ 완료 2026-08-23 |
| 0 | **T3 오라클 재정의** (실 PhysX 드리프트 기준) — B4·Oracle gap·Abstract 의존 | ❌ |
| 1 | 페이로드 항력 `is_global=True` | ❌ |
| 1 | `_drag_coef` 팬텀 채널 정리 | ❌ |
| 1 | **T0~T3 Table 1 재측정** (vz 수정이 T2 horizon·T3 오라클에 영향) | ❌ |
| 2 | DR 축 정리(질량/Cd 병합, CoM 삭제) + **DR_SCALE 노브 분리** | ❌ |
| 2.5 | **actor 관측에서 wind/drag 제거 단독 실험** — 주 주장 #3의 전제 | ❌ |
| 3 | 관측 재설계: heading-invariant 프레임 + 비대칭 critic (함정 3종 주의) | ❌ |
| 4 | 주파수 변경 시 **per-second 보상 재스케일 표** (10→50 Hz면 per-step 5배, loiter는 ~25배) | ❌ |
| 4 | 고도 범위 vs 종료 조건 정합 (`min_altitude=3.0` / `release_alt_min=3.0` → 4 m 시작이면 창 1 m) | ❌ |
| 0 | CCIP `vz` 수정 / T3 오라클 재정의 / 항력 `is_global` / 팬텀 채널 | ✅ 완료 2026-08-27 (`59755e7`, `f0ef4b8`) |
| 2 | DR 축 정리 + DR_SCALE 노브 분리 | ✅ 완료 2026-08-27 (`f0ef4b8`) |
| 2.2 | DR_SCALE 스윕 유효성 게이트 | ✅ **통과** 2026-08-27 (exp_025) |
| 5 | `release_delay` 실 latency 플랜트 구현 | ✅ 완료 2026-08-27 |
| 5 | success_radius 1.0 → 0.5 m | ✅ 완료 2026-08-27 (exp_026) |
| 5 | 릴리즈 판정 10 → 100 Hz | ✅ 완료 2026-08-27 (exp_026) |
| **A** | **헤드라인 지표 교체 CEP50 → CEP90 / 성공률(0.5 m)** — 오라클 갭이 꼬리로 이동 | ❌ |
| **A** | **T1/T2 동일화를 Table 1에서 어떻게 제시할지 결정** (행 병합 vs 10 Hz 열 병기) | ❌ |
| **B** | **급기동 도입 여부 결정** — [[research/agility_ceiling]] §4 단계별 변경. `reveal_radius`와 속도는 반드시 세트 | ❌ |
| 3 | 관측 재설계: heading-invariant 프레임 + 비대칭 critic | ❌ (급기동 도입 시 선행 필요) |
| 2.5 | actor 관측에서 wind 제거 단독 실험 | ❌ |
| — | T0~T3 최종 Table 1 n≥500 | ❌ |


- [ ] **(repo housekeeping)** `Issac_JS`의 신규 커밋(v20 task/flag 등록, junsang이 세션 중 push)을 `main`/`isaac_jk`에 merge — 아직 미반영.
- [ ] **(repo housekeeping, 선택)** git 히스토리 다이어트 — SAC replay buffer·YOLO epoch 체크포인트·mp4가 여러 브랜치 히스토리에 누적(pack 320MB). `git filter-repo` 필요, 파괴적 작업이라 사용자 확인 후 진행.
- [ ] **(repo housekeeping, 선택)** `donghyeok`/`junsang` 브랜치(둘 다 어디에도 머지 안 된 고아 브랜치) 처리 방침을 팀에 확인.
- [ ] **(Gazebo/SAC 트랙, 보류 — isaac_jk는 이 트랙을 더 이상 진행하지 않음)** v15 회귀 원인 미확정 상태로 인프라 차단(GPU 드라이버 535/580 불일치)됨. 재개하려면: 드라이버/컨테이너 정합 → `evaluate.py` 재실행 → `vel_damp_radius` 축소 또는 `w_vel→0` 적용 여부 결정. → [[daily/daily_2026-07-05_gazebo_v15_regression]] / Rule 25 / Rule 26
- [x] **(Isaac Lab, exp_019)** 물리 페이로드 attach/detach — kinematic weld 구현·검증 완료(4/4 PASS). → [[experiments/exp_019_physical_payload]]
- [ ] **(Isaac Lab, exp_019 후속 — 별도 지시 대기)** ⑤에피소드를 페이로드 착탄까지 연장(발화-종단 → 착탄-종단 + 측정 착탄오차 터미널 보상; 종단 의미론 변경 = fresh 학습) ⑥CCIP vz 항 복원(`t=(vz+√(vz²+2gH))/g`) ②Phase-2 DR(drag/wind) 힘을 물리 페이로드 낙하에도 적용(해석↔물리 정합) ④per-env `_ctrl_mass` 텐서화(분리 후 팬텀 0.1 kg). → [[research/physical_payload_attach]] §트레이드오프
- [x] **(Isaac Lab, exp_015)** Phase별 순차 커리큘럼 **코드 구현 완료** — action 4→6 residual, phase 노브+파생 플래그, 릴리스 이벤트+DR 착탄오차 터미널 보상, Gauss-Markov 이동타겟+lead, `train.py --phases` 오케스트레이터. `py_compile` 12파일 통과. → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20
- [x] **(Isaac Lab, exp_015)** 본 학습 완주 — `--phases 1,2,3 --phase_iterations 600,500,500 --num_envs 2048`(baseline, ~65 min, ORCH_EXIT=0). Phase 1 완전 수렴(success 1.00), P2/P3는 방향성 신호(drop↓/lead best 0.071m)만·명중 미형성. → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]]
- [ ] **(Isaac Lab, exp_015 후속)** P2/P3 명중 능력 확보: (a) exp_018 `release_terminal` 구조를 Phase 2·3에 적용(warm-start=exp018 B0 검토) 또는 (b) P2/P3 iteration 대폭 증량. PhaseCfg(residual_scale/release_tolerance/w_impact/w_lead) 튜닝. → [[research/curriculum_phase_convergence]]
- [ ] **(Isaac Lab migration, 병행 트랙)** L4 Spot VM 기동(`infra/deploy.sh` 빌드+push, `infra/startup.sh` 실행) → Cartpole 스모크 → `Isaac-DroneBombard-Direct-v0` env 스모크(2-iter) → `play.py --zero-actions/--scripted` 물리 검증. → [[experiments/exp_012_isaac_migration_phase2]]
- [ ] **(Isaac Lab migration)** PX4 속도-스텝응답 Gazebo 캡처 세션(`vel_logger_v2.py` 신규, 7-포인트) → Isaac 컨트롤러 게인 검정. 현재 미검정(구조 일치, 게인 초기값). → [[research/isaac_velocity_controller]]
- [ ] **(Isaac Lab migration)** `yolo_eval.py --calibrate` 첫 실행 → vision 캘리브레이션 v1(현재 v0=스펙 추정).
- [ ] **(Isaac Lab migration, 게이트 조건부)** `feat/isaac-env-migration`(이 워크트리)에서 `ros2_ws/`/`gazebo_models`/PX4 파일 정리 — **jekyun(라이브 SAC)는 대상 아님, 절대 미삭제.** 위 2개 항목(env 스모크 통과 + PX4 스텝응답 캡처) 완료 전까지 보류(사용자 확인, 2026-07-03). → [[experiments/exp_012_isaac_migration_phase2]] §8
- [x] **Vision 기반 RL 인프라 완성** — YOLO + SAC 시각 서보잉 파이프라인 구축
- [x] **EKF East 반전 버그 수정** — proximity target + RL env reward target 좌표 수정
- [x] **WandB `45l8vkw5` 100+ 에피소드 분석** → target_lost=1.0, EKF drift 확인 → 폐기
- [x] **EKF drift 방어 로직** — step 1에서 d_xy>5m이면 즉시 truncate (ekf_drift)
- [x] **fps 개선 — CRUISE 타임아웃 근본 원인 수정** — arm 게이팅(#2) + early-bail(#4). v12에서 효과 검증 중 → [[research/cruise_timeout_arming]]
- [x] **v12/v13 arm 처리량 재진단** — `ARM REJECTED` 0회(게이팅 작동), 실제 병목은 EKF 재수렴이 10s bail 초과(13–16s). **arm_bail 10→20s 적용** → [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / Rule 11
- [x] **v13 재시작** — 30K 체크포인트 인시던트로 소실 → **fresh 재시작**(iyhfy5ps, arm_bail=20). 검증: bail 0, late-EKF 회복 확인.
- [x] **핸드오프 윈도우 확장** — 고도↑(레버 아님) 기각, **탐지 게이트 수정**(conf 0.5 + 공간필터 200→300 px)으로 핸드오프 2.7→5.0 m(~2배), spurious 0. dry-run PASS, **미커밋** → [[experiments/exp_008_dryrun_alt10_handoff_window]] / Rule 13
- [x] **리셋 처리량 병목 규명 + 수정** — 원인=teleport 후 EKF 재수렴(param 불가). **soft reset로 ~3.9×** 프로토 검증 → [[experiments/exp_009_softreset_throughput]] / Rule 14
- [x] **soft reset 장기 검증** — byxyaf4d 3096 resets에서 soft 성공 ~91%, EKF bounded, no teleport/no PX4 restart. **Rule 14 검증완료.** → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- [ ] **⚠️ v14 정책 회귀(65%<80%) 해소** — 195K eval 실패 전부 final-approach stagnation(미성숙 가설). 옵션: ① soft reset 켠 채 500K까지 재개/연장(미성숙 검증) ② 10m→5m 고도 A/B(종말 거리 가설) ③ 종말 보상 shaping 강화. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- [ ] **⚠️ v14 commit 결정 (사용자)** — soft reset/탐지게이트 인프라는 검증완료지만 정책 65%<80%. "v14를 validated baseline으로" 채택 여부 결정 필요. 윈도우 확장 + 탐지게이트 + soft reset 코드 일괄 미커밋 상태.
- [ ] **v13(iyhfy5ps) 추세 점검** — 첫 롤아웃 후 success_rate 발생 + ep_len/env/ep_reward 추세 + 전체 bail율(구 ~21/h 대비)
- [ ] **(장기) teleport EKF 재수렴 단축** — 13–16s 재수렴 자체 줄이기(명시적 EKF reset 등). 타임아웃은 증상 완화일 뿐
- [ ] **⚠️ YOLO target_lost_rate ~29% bimodal 해결** — per-step 트리거가 전부/전무로 분리(악화 0.24→0.35). obs[9-11] zeroed + `-10` 페널티. 미해결 (이번 세션 범위 밖)
- [ ] **PX4 로그 /dev/null 리다이렉트** — `/tmp/px4_{i}.log` 100+ MB 증가 방지

---

# 4. Training History

> **전체 히스토리:** `notes/experiments/training_history.md`
> **개별 실험 노트:** `notes/experiments/exp_NNN_*.md`

최근 주요 runs:

| 날짜 | Run ID | Steps | 요약 |
|------|--------|-------|------|
| 2026-07-21 | **exp019 물리 페이로드 attach/detach (검증 전용, 병행 트랙)** | 0 (8 envs hover-drop 검증, isaac-verify) | **kinematic weld 구현 + 4/4 PASS.** 부착 추적 1.1 mm, 분리 0/8 잔류, 착탄 8/8, 측정 vs 해석적 CCIP \|Δ\| mean 0.012/max 0.021 m. 보상·종단 bit-identical. Rule 24. → [[experiments/exp_019_physical_payload]] |
| 2026-07-13 | **exp015 이어학습(2차) P2/P3 ext (Isaac PPO, 병행 트랙)** | P2: +2000 iters (1098→3097) · P3: +2000 iters (3097→5096), 2048 envs, ~3h | **iter 예산 확대 검증 — 0.8m 돌파 ❌.** P2 ext: drop 2.87m(정체), release 0.33→0.01, success 0. P3 ext: drop 5.31m(회귀), reward 74.5, success 0. Rule 20f. → [[experiments/exp_015_phased_curriculum]] §8 |
| 2026-07-12 | **exp015 실학습 (baseline, Isaac PPO, tensorboard, 병행 트랙)** | 2048 envs × 600/500/500 iters (~104M steps, ~65 min) | **Phase 1→2→3 커리큘럼 첫 완주(ORCH_EXIT=0). Phase 1만 완전 수렴.** plain `--phases 1,2,3`. P1 success 0.48→1.00·reward→107(exp_014 재현); P2 reward −0.8→94.7·drop 4.66→2.91m↓·release peak 0.98/tail 0.33·success~0; P3 reward→102·lead best 0.071m·success~0. warm-start 무손실(Rule 20e). reward 우상향=proximity 지배 — P2/P3 명중 미형성(추가 학습 또는 exp_018 종단구조 필요). ckpt/로그/그래프 host 영속화. → [[experiments/exp_015_phased_curriculum]] §7 / [[research/curriculum_phase_convergence]] |
| 2026-03-20 | 8otphxy8 | 114K | 선형 거리 보상 + CRUISE retry. 마지막 정상 베이스라인. |
| 2026-03-22 | — | — | 보상 패치 적용 (학습 없음). Fresh start 대기 중. |
| 2026-04-16 | mtx7ud6o/x8jq9fsy/u8w3xn0w | 5500×3 | RTF 1/2/4 dry-run. RTF=2 최적 (59.5 fps). |
| 2026-06-12 | esmtny0a | 33K | Vision SAC. proximity 버그로 128ep stagnation. 폐기. |
| 2026-06-12 | 45l8vkw5 | 121K | rl_yolo. target_lost=1.0 전구간. 원인: EKF drift + FOV gap. 폐기. |
| 2026-06-13 | 7lhjy40o | 진행 중 | rl_yolo_v7_drift_guard. EKF drift guard + proximity 2.5m + penalty_lost=-0.1. |
| 2026-06-14 | k1uqgs8i | ~42K | rl_yolo_v11_cam_fix. 학습 개선(env/ep_reward 20→54, 404 successes)이나 443 CRUISE 타임아웃으로 중단. |
| 2026-06-15 | rl_yolo_v12_arm_fix | 진행 중 | Arming-rejection throughput fix (arm 게이팅 + early-bail). dry-run 0 타임아웃 검증 후 fresh 기동. |
| 2026-06-17 | 46y4xtiw | ~30K (중단) | rl_yolo_v13_terminal_reward. ~10h에 6%뿐(fps≈0.83) — arm_bail 병목 진단 위해 graceful stop. 재개 대기. |
| 2026-06-17 | xgzum51v (offline) | 1000 (dry-run) | v13_armdiag. EKF 재수렴 bimodal(0s/13–16s) 계측 → arm_bail 10→20s 수정. Rule 11. ⚠️ 이 dry-run이 YAML 중복 키로 v13 30K 체크포인트 파괴. |
| 2026-06-17 | iyhfy5ps | 진행 중 (fresh 0→500K) | rl_yolo_v13_terminal_reward **fresh 재시작** (arm_bail=20). 30K 인시던트 후. 검증: bail 0, late-EKF 14–15.5s ×3 회복. → [[errors/err_20260617_dryrun_clobbered_v13_checkpoints]] |
| 2026-06-22 | dryrun_alt10 (uqy7lmny/_gated, offline) | 1500×2 (dry-run) | **핸드오프 윈도우↑.** 고도만 10 m=실패(레버 아님; 마커 작아짐 + 200 px 필터 캡). 10 m+탐지 게이트(conf 0.5 + 200→300 px)=성공: 핸드오프 2.7→5.0 m(~2배), spurious 0. **미커밋.** → [[experiments/exp_008_dryrun_alt10_handoff_window]] / [[research/detection_gate_vs_altitude]] / Rule 13 |
| 2026-06-22 | EKF A/B + softreset proto (offline) | 1500+5000 | **리셋 처리량 ~3.9×.** 원인=teleport 후 EKF 재수렴(`pre_flight_checks_pass=False`). EKF2_GPS_CHECK 0 A/B=음성(COM_ARM_WO_GPS). **soft reset(teleport 회피)=성공**: 0.93→3.61 handoffs/min, fps 2→9, reset 65s→11s, soft 100%, EKF 안정. → [[experiments/exp_009_softreset_throughput]] / Rule 14 |
| 2026-06-22 | rl_yolo_v14_softreset (byxyaf4d) | 진행 중 (fresh 0→500K) | **full run — soft reset ON 장기 검증.** 10m + 탐지게이트 + soft reset 일괄, online. EKF drift bounded?/fallback율?/실제 ETA(~15h 예상) 확인 후 커밋. → [[experiments/exp_009_softreset_throughput]] |
| 2026-06-23 | rl_yolo_v14_softreset (byxyaf4d, stop @196.5K) | 196.5K/500K(~39%), reward plateau | **plateau stop + 195K eval = 65%(13/20).** v13 80% 대비 회귀(실패 전부 final-approach stagnation). EKF 귀책 0. **Soft reset 장기검증 ✅**(3096 resets, soft ~91%, EKF bounded) → Rule 14 검증완료. 비디오 3/3 success 캡처. 회귀=미성숙(39% budget). commit 결정 대기. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]] |
| 2026-07-01 | rl_yolo_v15_bc_stable | 진행 중 (fresh 0→300K) | **wobble 교정(LPF+B+C) 적용 fresh run.** jerk RMS 2.92→1.61(−45%) A/B 확정 후 기동. → [[experiments/exp_011_wobble_lpf_reward_damping]] / Rule 15 |
| 2026-07-03 | isaac_migration_phase2 (`feat/isaac-env-migration`, 병행 트랙) | 0 (코드만) | **Isaac Lab env+PPO 이식.** v13/v15 parity, `pytest test_math.py` 29/29 통과, L4 VM 미기동. jekyun SAC 학습과 별개 브랜치/워크트리. → [[experiments/exp_012_isaac_migration_phase2]] |
| 2026-07-03 | exp013_v1_baseline (Isaac PPO, 병행 트랙, 중단 @iter 106) | ~7M steps | **비전 사멸 버그 발견·중단.** `rew_vision`≡0.0000 → `_update_vision` env-origin 프레임 혼용(2048-env grid에서 타겟 항상 프레임 밖). 수정+수치검증(visible 0%→63%). → [[errors/err_20260703_vision_env_origin_frame]] |
| 2026-07-03 | **exp013_v2_visionfix (wcjklw7a, Isaac PPO, 병행 트랙)** | 65.5M steps (1000 iters 완주) | **첫 완주 + deterministic 200-ep eval = 36%.** plateau @iter 700, d_xy_min 1.4m 정체. 실패: max_alt 33%(상승 farming, Rule 17)+crash 27%. farmer(+225)>finisher(+121) 불균형(Rule 18a), noise_std 0.8→3.92 폭주(Rule 18b). **사후 --zero-actions FAIL(11.9m): 리셋 속도킥 활성 — run 오염, max_alt 1차 용의자.** 다음=exp_014(0순위 킥 수정 → conf 거리감쇠+success 300+entropy 0, fresh). → [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] / [[research/isaac_ppo_tuning_recommendations]] |
| 2026-07-05 | **exp014 A2 (v3qk07pg) + A0′ (azoc1xp0), Isaac PPO, 병행 트랙** | 각 26.2M steps (400 iters) | **plant 수정 + 비전 거리감쇠 → deterministic 200-ep eval = 100.0% (202/202), d_xy_min 0.665m.** 수정: ①킥→스폰타임 MassAPI authoring(게이트 11.9m FAIL→0.2m PASS) ②로터 스핀 리셋 재주입 제거 ③**inertia 대반전**: `set_inertias`는 전파되고 있었음 — exp_013은 rate loop ~1300× 저토크 plant(Rule 19, 구 정책 plant-overfit로 무효). A2: R_alt=0.0000(climb 창발→기각 시그니처), noise_std 0.80 안정(폭주 없음). A0′ 대조: R_alt 0.0365 → 지배 요인=plant 일관성, 감쇠=꼬리 제거+YOLO parity(유지). 실 YOLO 캘리브레이션은 이미지 annotator 버그로 차단(하네스 수리 완료). reward_success·entropy 불변(다음 페이즈). → [[experiments/exp_014_A2_visionrange]] / [[research/isaac_inertia_ctrl_mismatch]] |
| 2026-07-05 | isaac_phased_curriculum (`feat/isaac-env-migration`, 병행 트랙) | 0 (코드만, 미학습) | **Phase별 순차 커리큘럼 구현.** 이미지 3단계(접근/nominal → CCIP+Residual/정지 → 이동타겟) 완전 구현: action 4→6(δ residual), phase 노브+파생 플래그, 릴리스 이벤트(nominal CCIP+δ 트리거 → 실제 DR 낙하 착탄오차 터미널 보상), drag/wind DR, Gauss-Markov 이동타겟+lead, `train.py --phases 1,2,3` 서브프로세스 오케스트레이터(6-dim 고정 → warm-start 무손실). Phase 1=exp_014 baseline 동작 동일. **`py_compile` 12파일 통과; `pytest`(+8 신규)·본 학습은 L4/컨테이너 대기.** → [[experiments/exp_015_phased_curriculum]] / [[research/phased_curriculum]] / Rule 20 |
| 2026-07-05 | **exp016 CCIP 릴리스 referee 재평가 (eval-only, A2 ckpt, 병행 트랙)** | 0 (200-ep deterministic eval) | **디커플링 규명: 4.59m = 지표 의미론 버그(릴리스 트리거 부재, 성공-종단 속도 캐리).** CCIP referee(≤0.2m) 수정 후: 발화 시 0.137m, release_rate 6%/11.5%(10/100Hz), aim_err_min med 0.755m ≈ d_xy_min(cross-track 지배). 구 지표 4.649m 재현. 보상/종단 bit-identical. Rule 21. → [[experiments/exp_016_ccip_release_reeval]] / [[research/ccip_release_decoupling]] |
| 2026-07-06 | **exp017 Stage A — 밀집 CCIP 조준 보상 (750gpldr/6z0gpnhy/fv5qqmtz, Isaac PPO, 병행 트랙)** | 3 runs (P1 400 + v1 400 + v2 600 iters, 2048 envs) | **보상-변경-단독은 release_rate 못 올림 — 판정 (b), Rule 22.** det 200-ep: 기준선 2.5% → v1(w=1) 5.5%(aim 0.889m·speed 2.72, 방향 실재·p≈0.13) → v2(w=2/knee 1.0) 3.5% 회귀. P1 기준선 학습 내 12→3.7% 단조 하락(근접 최적화가 릴리스 능력 파괴). 원인=γ-할인 완주 보너스·CCIP 노이즈 증폭(×1.53s)·성공 조기 종단. 5-lens 사전 검증, farm 0. ckpt 3종 분리 보존(+호스트 백업) — 차기 warm-start=stageA(v1). → [[experiments/exp_017_stageA_aim_reward]] / [[research/ccip_aim_reward_stageA]] |
| 2026-07-06 | **exp018 Stage B — 릴리스-종단 이벤트 (xt0hrr1c/0ns10yso/4vaodj0o/kk06wsbx, Isaac PPO, 병행 트랙)** | 4 runs × 400 iters (전부 v1 warm-start) | **릴리스-종단 구조 → det release_rate 100.00%, drop err 0.125 m (Rule 23).** 종단 교체 단독(B0)으로 5.5%→100%(학습 내 23→99.6% 단조 상승 — Stage A 하락 반전, Rule 22a 인과 확정). aim 보상 노브 불감(w 0/1.5 100%, knee 0.75만 98.5%) — 자동 발화 referee가 노이즈를 +100 샘플러로 전환. done-flag alias 버그 사전 수정(eval success 0% 위험). 호버-드롭 수렴(종단 속도 0.11 m/s). **Stage C warm-start = exp018_B0_final.pt.** → [[experiments/exp_018_release_terminal]] / [[research/release_terminal_stageB]] |
| 2026-07-23 | **exp020 물리 페이로드 부착 첫 학습 (o5jn9xzk train / vryuc6mu eval, Isaac PPO, 병행 트랙)** | 400 iters (2048 envs, warm-start=exp018_B0, 보상 bit-match) | **물리 페이로드(kinematic weld) 학습 비용 = 0 — det 200-ep success/release 100.00%, drop err 0.169 m.** `physical_payload=True`가 유일한 델타, release_rate 첫 롤아웃부터 100%(재학습 과도기 없음). σ 드리프트 1.41→1.71 모니터 대상. `play.py --wandb` eval-figure 파이프라인 신설. 컨테이너 빈 WANDB_API_KEY 함정(`--env-file` 필수). ckpt 호스트 `/opt/drone-bombard/checkpoints/exp020/`. → [[experiments/exp_020_o5jn9xzk_payload_training]] |
| 2026-07-30 | **exp021 v19 + 이동 타겟 CV/CT/CA warm-start 학습 (a6saa42b/29jqq1lu/ntumqwoz, Isaac PPO, 병행 트랙)** | 3 runs × 1000 iters (2048 envs, warm-start=준상 v19 precise 사본) | **이동타겟 모션(CV/CT/CA)을 v19에 obs-보존 포팅 → 3종 학습 완주.** 종반 창: cv release 0.67–0.90/drop 0.39–0.90 m, ct 0.43–0.80/0.24–1.24 m, ca 0.40–0.78/0.32–1.96 m(reward 음수 잔존 — 가속 타겟 최난). 난이도 cv<ct<ca. det eval 후속. ckpt `/opt/drone-bombard/checkpoints/exp021/`. → [[experiments/exp_021_v19_moving_target]] |
| 2026-07-30 | **exp021 det 200-ep eval 3종 (1nvvuogg/prdqujah/gdow3vfg, eval)** | 0 (deterministic eval only) | **cv success 44.5%/release 81.5%/drop med 0.775 m · ct 33.8%/82.6%/0.783 m · ca 16.5%/63.5%/1.063 m.** 릴리스 이월·명중은 리드 부재로 경계 몰림(released-miss>success). 개선 1순위=리드 개입(KF obs 포팅/target-vel obs/w_lead). play.py --wandb NameError(impacted) 수정, eval seed 미고정 표본 변동(32↔44.5%) 확인. → [[experiments/exp_021_v19_moving_target]] §3b |
| 2026-09-02 | **exp029 L1-SL 일반화 감사 — 미지 사거리 · 정책 전이 · 라벨 수 (주입 39 run + 오프라인 적합 6종, 학습 없음)** | 0 (추론 롤아웃 + 오프라인 회귀; arm당 600 짝지은 ep, 평가 시드 3000/4000/5000) | **✅ 미지 사거리 26–30 m: CEP50 0.299→0.218(−27.3%), 오라클 회수율 85.3% = 훈련 범위(85.0%)와 동일 — 기하 암기 아님. ⛔ 정책 전이(seed1→seed2): CEP50 −7.0%인데 succ@1.0 96.0→89.3%, CEP90 0.542→0.830 = 순손실; seed2 재적합 시 −22.0%·CEP90 0.562로 복구 → obs→drift는 정책의 성질($\bar\theta \approx F_{wind}/K_{policy}$), 정책마다 재적합 필요 (Rule 39). ✅ 라벨 수: 100/200/500/1000/2334 ep → CEP50 −5.5/−14.8/−23.8/−30.1/−31.4%, 1,000회면 이득의 96% = 실기 예산. 참바람 입력은 100 ep에서 이미 $R^2$ 0.982 → 오라클 간극은 데이터가 아니라 정보. ⚠️ 오보정 잔차는 중앙값보다 꼬리를 먼저 망침(n=100: CEP90 +43%, succ@1.0 −6.5 pp) — CEP50 단독 판정 금지 (Rule 40).** wandb `sl_gen_unseenR`/`sl_gen_policy_transfer`/`sl_gen_label_count`. → [[experiments/exp_029_l1_sl_generalization]] / [[research/residual_policy_coupling]] / [[research/residual_label_efficiency]] |
