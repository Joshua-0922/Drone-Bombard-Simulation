---
date: 2026-07-03
updated: 2026-09-25
tags: [research, architecture, isaac-lab, ppo, rsl_rl, code-structure]
status: active
type: research
---

# 시스템 구조 — `isaac_lab/` 파일별 역할 (2026-09-25 기준)

> **한 줄.** 단일 프로세스 GPU 병렬 Isaac Lab `DirectRLEnv` 하나가 시뮬레이터·제어기·심판을 전부 맡고, 그 위에서
> PPO(rsl_rl)로 비행 정책 L0를 학습한 뒤, 동결한 L0의 비행 기록으로 착탄점 잔차 GRU를 오프라인 지도학습해 주입한다.
> 어느 함수가 몇 번째 줄인지는 [[research/code_map]], 실행 명령은 [[sessions/commands]], 방법 설명은 [[research/l1_sl_pipeline]].
> 2026-07 이식 당시의 구조·Gazebo 대비표는 이 문서의 git 이력(커밋 14db22b 이전)과 [[experiments/legacy/exp_012_isaac_migration_phase2]]에 있다.

## 저장소 레이아웃

```
Drone-Bombard-Simulation/  (branch main)
├── isaac_lab/                   ← 현행 코드 전부 (아래)
├── notes/                       ← Obsidian 연구 기록 (research·experiments·daily·sessions·errors, 구식은 각 폴더 legacy/)
├── graphify-out/                ← 코드·노트 지식 그래프 (gitignore, `graphify update .`)
├── drone_drop_system/docker/    ← Isaac Sim 5.1 + Isaac Lab 2.3 + rsl_rl 이미지 (컨테이너 isaac-verify의 출처)
├── infra/                       ← GCP L4 VM 배포 스크립트
├── ros2_ws/, gazebo_models/     ← 구 Gazebo/PX4/ROS2 스택. 미참조·미삭제 (실기 PX4 정합 시 참고용)
├── drone_bombard_best.pt, yolo_workspace/  ← YOLO 가중치·학습 산출물 (비전 절에서 사용 예정)
└── RL_Project_Log.md, CLAUDE.md
```

실행은 컨테이너 `isaac-verify:/tmp/rebuild`(repo `isaac_lab/`의 사본)에서 하며, 편집 후 `docker cp`로 맞춘다.

## `isaac_lab/` 파일 역할

### 환경 (drone_bombard/)
| 파일 | 역할 |
|---|---|
| `__init__.py` | gym 등록: `Isaac-DroneBombard-Direct-v0`(기저), **`Isaac-DroneBombard-Task-v0`(현행 과제)** |
| `drone_bombard_env.py` | 기저 `DirectRLEnv`: 씬·기체 물리·캐스케이드 속도 제어기(PX4 게인 매핑, [[research/isaac_velocity_controller]])·**모델 오차 랜덤화**(`ModelErrCfg`: 바람 정상/OU/현실, 탄도계수, 릴리즈 지연, `scale` 노브)·페이로드 낙하 물리·오라클 잔차 |
| `task_env.py` | **현행 과제** `DroneBombardTaskEnv(Cfg)`: 관측 26, 행동 7(`[0:4]` 속도·요, `[4]` 투하, `[5:7]` 착탄점 잔차), CCIP 예측(`_ccip`), 첫 교차 릴리즈 게이트(`_resolve_release`), 보상(`TaskRewardCfg`, [[research/reward_design]]), 핸드오프·릴리즈 봉투·인지 cfg, 잔차 EMA·누적 관측 스위치 |
| `math_utils.py` | isaaclab 무의존 순수 torch 함수: 탄도 적분(`integrate_payload_impact`, 정답 드리프트의 근원), CCIP 공칭 예측, 잔차 적용, 보상 항, 게이트, 가드, 핀홀 투영, (이동 표적·KF는 논문 범위 밖) |
| `residual_actor.py` | 잔차 RL용 동결 L0 + 별도 잔차망 — **논문 미채택**(Rule 44), exp_032·033 재현용 |
| `v11_env.py` | 팀원 트랙의 구 환경 — 미사용 |
| `mdp/domain_rand.py` | 바람 샘플러(`sample_wind_capped`) 등 랜덤화 유틸 |
| `agents/rsl_rl_ppo_cfg.py` | PPO 하이퍼파라미터(actor/critic [256,256] ELU, 96 steps/env, lr 3e-4 adaptive, entropy 0.005) |

### 학습·평가 진입점
| 파일 | 역할 |
|---|---|
| `train.py` | rsl_rl `OnPolicyRunner`로 L0 학습(`--task_env --no_residual --dr_scale 1.5`). 잔차 RL 플래그(`--residual_net` 등)는 미채택 경로 |
| `play.py` | 평가·수집 허브: `--paired_eval`(팔 비교, seed 고정 시나리오), `--dump_sl`(GRU 학습 데이터 덤프), `--sl_residual/--sl_ema`(GRU 주입, `_SLResidual`·`_ResidualEMA`), 오라클 팔, 바람·DR 스위치, sanity 모드(`--zero-actions/--scripted/--step-response/--wind-test/--drop-test`) |
| `baseline_drop.py` | 규칙 팔 T0(정지 투하)·T2(정속 통과 + 공식 최소점 릴리즈) |
| `_fit_sl_seq.py` | **본 방법 학습기**: GRU 26→64→2, MSE, TorchScript 내보내기. `--label realised`·`--nll`는 미채택 변형 |
| `_fit_sl_residual.py` | 구 방법(수제 누적 특징 + MLP 38) — ablation "학습 필터 vs 수제 특징" |
| `eval_harness.py`, `_agg_table1.py`, `_agg_sl_eval.py` | 평가 지표(CEP50/90, succ@0.5, 배달률·시간, 투하 속도)와 표 집계(시드셋 assert) |
| `_fig_final.py` | 논문 그림 3장 |
| `yolo_eval.py`, `record_episode.py`, `verify_one_episode.py`, `select_best_checkpoint.py` | 실 YOLO 평가(비전 절 예정), 녹화, 1 에피소드 검증, 체크포인트 선택 |

### 일괄 실행 스크립트 (`_*.sh`)
`_gru_eval.sh`(τ 사다리) · `_gust_eval.sh`(현실 바람 A·B) · `_grus_final.sh`(DR 스윕·미지 사거리) · `_ema_sweep.sh`(EMA α) · `_gi_headline.sh`·`_gi_eval.sh`·`_t_reseed.sh`·`_dr_axis.sh`(기존 팔·T0/T2 재시드) · `_sl_gen.sh`(일반화 감사) · `_ou_test.sh`·`_ou_orc.sh`(OU) · `_l1rl_pilot.sh`(잔차 RL, 미채택)

### 진단·프로브 (`_diag_*.py`, `_probe_*.py`, `_corr.py`, `_r2_groups.py`, `_fit_falloff.py`, `_extract_traincurve.py`, `_adhoc_badatt.py`, `_test_payload_drop.py`)
과거 버그·가설 검증용 일회성 스크립트. 결과는 해당 실험 노트에 있고, 재실행 대상이 아니다.

### 테스트 (`tests/`)
`test_math.py`(순수 torch, 탄도·게이트·보상), `test_residual_actor.py` 등 — 컨테이너에서 `isaaclab.sh -p -m pytest`, 98/98.

## 데이터 흐름 (policy step 10 Hz, 물리 100 Hz)

```
관측 26 (표적 상대위치·CCIP 오차·착탄 오차 크기 / 고도·속도·자세·각속도·낙하시간·남은 시간 / 직전 명령 / 탐지 플래그)
  │  [주입 시] GRU-S 한 스텝 → δ̂ → EMA 0.3 → 행동 채널 5:7
  ▼
행동 7 = [vx vy vz yaw_rate | drop | δx δy]
  ├─ 속도 명령 → rate-limit → 캐스케이드 속도→자세→토크 제어기 ×10 물리 스텝 (바람은 기체 항력에 작용)
  ├─ _ccip: 예측 착탄점 = 공칭 CCIP(pos, vel, alt, 공칭 탄도계수·지연) + 2 m × δ
  │        → 관측의 조준 오차 채널(L0 조향) 과 릴리즈 게이트(첫 교차) 양쪽에 들어감
  ├─ 릴리즈: 게이트 통과 시 페이로드 분리 → 실제 바람·탄도계수·지연으로 낙하 → 착지 오차로 채점
  └─ 보상: 착지 오차(종단) + 들고 있는 시간 비용 + 접근·조준 셰이핑 ([[research/reward_design]])
```

## 학습 조건 요약 (본 표의 전제)
DR `scale` 1.5(바람 축당 N(0, 1.5·1.5) m/s, 상한 7.5) · 판정 100 Hz · 표적 참값 · 사거리 18–22 m · 성공 반경 0.5 m · L0 seed 1 · 평가 seed 3000/4000/5000 × 200.
