# Drone Payload Drop — 바람 속 정밀 투하 (Isaac Lab)

드론이 표적 근처로 비행하며 화물을 던져 떨어뜨릴 때, **바람 때문에 생기는 착지 오차를 바람 센서 없이 줄이는** 연구다.
비행은 강화학습 정책이 맡고, 조준 보정은 과거 비행 기록에서 지도학습한 작은 신경망이 맡는다.
시뮬레이션(Isaac Sim / Isaac Lab)에서 방법과 결과를 확정했고, 다음 단계는 실기 적용(sim2real)이다.

> 상태: 2026-11-28 · 브랜치 `main` · 연구 기록은 `notes/`(Obsidian vault), 진행 요약은 `RL_Project_Log.md`

---

## 1. 방법 한눈에

```
관측 이력 ──► [잔차 신경망] ──► 착지 오차 예측 δ̂ ──► 예측 착탄점 = 공식 예측 + δ̂
                                                        │
비행 정책 L0 (PPO, 동결) ◄── 조준 오차 관측 ◄────────────┘
                                                        │
                           릴리즈 게이트: 보정된 착탄점이 표적을 처음 지나는 순간 투하
```

| 구성 | 내용 |
|---|---|
| **L0 비행 정책** | PPO(rsl_rl), 2048 병렬 환경, 1000 iteration. 속도 명령과 투하 신호를 낸다. 기체 모델은 PX4 속도·자세·각속도 PID, 모터 지연, 통신·검출 지연을 포함한다 |
| **잔차 신경망 (GRU-S)** | 관측 26 → GRU 은닉 64 → 보정 2(x, y). 정답 = "동결 L0가 보정 없이 날았을 때 실제 착지점 − 표적". L0 비행 약 2,000회로 오프라인 학습(수 분) |
| **모델 오차** | 바람(비행마다 다른 평균풍 0–6 m/s + Dryden 돌풍) · 화물 공기저항 계수 · 투하 지연 · 질량 등을 함께 랜덤화, 세기 노브 `dr_scale` 1.0 |

출력 평활(EMA)은 2026-11-28에 방법에서 제외했다. 새 정답에서는 평활 유무가 결과를 바꾸지 않는다.

## 2. 주요 결과 (시뮬레이션, 같은 시나리오 paired 비교, L0 seed 3개 합산 1800 에피소드)

| 조건 — CEP50 / CEP90 [m] / 성공률@0.5 m | 정지 투하 T0 | 규칙 투하 T2 (1.0 m/s) | L0 (보정 없음) | **본 방법** |
|---|---|---|---|---|
| DR 1.0, 18–22 m | 0.489 / 0.953 / 48.3% | 0.401 / 0.971 / 56.5% | 0.345 / 0.897 / 66.4% | **0.253 / 0.546 / 84.6%** |
| DR 2.0 (미학습 세기) | 0.802 / 2.730 / 22.8% | 0.900 / 2.763 / 28.8% | 0.626 / 2.453 / 39.5% | **0.401 / 1.602 / 53.8%** |
| 미학습 거리 26–30 m | 0.494 / 0.917 / 43.3% | 0.390 / 0.901 / 52.3% | 0.348 / 0.842 / 67.3% | **0.243 / 0.570 / 82.2%** |
| 현실 돌풍 B (30%, τ 3 s) — CEP50 | — | — | 0.408 | **0.290** |

- 배달 시간은 본 방법 6.15 s, T2 8.58 s, T0 8.09 s (DR 1.0).
- 평균풍 없이 낙하 시간(약 1 s)보다 빨리 변하는 돌풍만 남긴 스트레스 조건에서는 보정이 손해다(+7 ~ +12%).
- 상세: `notes/experiments/exp_052`, `exp_054`, `exp_055`.

## 3. 저장소 구조

```
isaac_lab/                     현행 코드 전부
├── drone_bombard/
│   ├── task_env.py            현행 과제 환경 (관측 26, 행동 7, CCIP 예측, 릴리즈 게이트, 보상)
│   ├── drone_bombard_env.py   기저 환경 (기체 물리, 속도 제어기, 바람·모델 오차 랜덤화, 화물 낙하)
│   ├── math_utils.py          순수 torch 수식 (탄도 적분, CCIP, 보상, 게이트) — 단위테스트 대상
│   └── agents/rsl_rl_ppo_cfg.py
├── train.py                   L0 PPO 학습
├── play.py                    평가·데이터 덤프·잔차 주입 허브 (--paired_eval, --dump_sl, --sl_residual)
├── _fit_sl_seq.py             잔차 GRU 학습기 (본 방법)
├── baseline_drop.py           규칙 기반 비교 팔 (정지 투하, 규칙 릴리즈)
├── _*.sh, _agg_table1.py, _fig_final.py   일괄 평가·표·그림
└── tests/                     단위테스트
notes/                         연구 기록 (Obsidian): 00_index.md 대시보드, 00_toc.md 전체 목차
drone_drop_system/docker/      Isaac Sim 5.1 + Isaac Lab 2.3 + rsl_rl 이미지
infra/                         GCP L4 VM 배포
ros2_ws/, gazebo_models/       구 Gazebo/PX4/ROS2 스택 (현재 미사용, 실기 PX4 정합 시 참고)
REVIEW_GUI.md, checkpoints/v19 팀원 트랙(v19)의 GUI 재생·warm-start 안내 (현행 파이프라인 아님)
```

파일별 역할은 `notes/research/isaac_lab_architecture.md`, 함수·줄 위치는 `notes/research/code_map.md`.

## 4. 실행

코드는 repo `isaac_lab/`에서 편집하고, 컨테이너 `isaac-verify`의 `/tmp/rebuild`에서 실행한다.

```bash
docker start isaac-verify
docker cp isaac_lab/. isaac-verify:/tmp/rebuild/          # 편집 후 매번
docker exec -it isaac-verify bash
IL=/workspace/isaaclab/isaaclab.sh; cd /tmp/rebuild
CK=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt   # 본 L0 체크포인트

# 단위테스트
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $IL -p -m pytest tests -p no:cacheprovider -q

# L0 학습 (약 1시간)
$IL -p train.py --task_env --no_residual --dr_scale 1.5 --num_envs 2048 --max_iterations 1000 --seed 1 --headless

# 잔차: 덤프 → 학습 → 주입 평가
$IL -p play.py --policy $CK --no_residual --headless --num_envs 256 --episodes 1500 \
    --marker_dist 18 22 --dr_scale 1.5 --seed 1000 --dump_sl /tmp/sl/v2_s1a.npz
$IL -p _fit_sl_seq.py /tmp/sl/v2_s1a.npz /tmp/sl/v2_s1b.npz --label miss --export /tmp/sl/res_gru_S.pt
$IL -p play.py --policy $CK --paired_eval --headless --episodes 200 --num_envs 200 \
    --marker_dist 18 22 --dr_scale 1.5 --seed 3000 --sl_residual /tmp/sl/res_gru_S.pt
```

전체 명령(바람 조건, 비교 팔, 표·그림, 백그라운드 실행): `notes/sessions/commands.md`

## 5. 환경

- GPU 드라이버 ≥ 580 (Isaac Sim 5.1 RTX 렌더러). 헤드리스 학습·평가는 GUI 없이 동작한다.
- 이미지 빌드: `drone_drop_system/docker/Dockerfile`. 로컬에서는 기존 컨테이너 `isaac-verify`(이미지 `isaac-lab-local:580`)를 재사용한다.
- 컨테이너 안 파이썬은 `isaaclab.sh -p`. 호스트 python3에는 torch가 없다.
- 학습·평가 산출물은 컨테이너 `/tmp`에 있다. wandb 키는 호스트 `/opt/drone-bombard/.wandb.env`를 `--env-file`로 넘긴다.

## 6. 문서 안내

| 무엇 | 어디 |
|---|---|
| 논문 개요 (쉬운 용어) | `notes/research/paper_outline_v6.md` |
| 방법 5단계 | `notes/research/l1_sl_pipeline.md` |
| 최종 표·그림 | `notes/research/final_tables_v6.md` |
| 연구 아키텍처 (확정 사항 요약) | `notes/research/research_architecture.md` |
| 실험 규칙 Rule 1~46 | `notes/research/rl_rules.md` |
| 실험 이력 | `notes/experiments/training_history.md` |
| 전체 노트 목차 | `notes/00_toc.md` |

과거 이력(Gazebo/PX4/ROS2 + SAC 시기, Isaac 이식, 커리큘럼 실험)은 `notes/research/legacy/`, `notes/experiments/legacy/`와 git 이력에 있다.
