# Fundamentals 학습 인덱스

> 시작일: 2026-06-27
> 목적: v10 design 전에 환경/제어/MDP/알고리즘/학습 메커니즘을 바닥부터 정리.
> 진행 방식: 옵션 A — 카테고리별 순차 깊이 학습 (top-down).

---

## 카테고리 진도표

| # | 파일 | 주제 | 상태 |
|---|---|---|---|
| 1 | [01_environment.md](01_environment.md) | 환경 (Sim Stack — Gazebo/PX4/DDS/ROS2) | ✅ 완료 (Session 1.1~1.4) |
| 2 | [02_drone_control.md](02_drone_control.md) | 드론 제어 (PX4 control hierarchy, 좌표계, mixer) | ✅ 완료 (Session 2.1~2.5) |
| 3 | [03_mdp_env_code.md](03_mdp_env_code.md) | MDP & DroneDropEnv 해부 | ✅ 완료 (Session 3.1~3.8) |
| 4 | [04_sac_per.md](04_sac_per.md) | SAC + PER 알고리즘 | ✅ 완료 (Session 4.1~4.7) |
| 5 | [05_learning_mechanism.md](05_learning_mechanism.md) | 학습 메커니즘 (exploration / replay / forgetting / curriculum) | ✅ 완료 (Session 5.1~5.8) |

## 5 카테고리 통합 정리 — 한 페이지 요약

### Layer 별 인사이트

| 영역 | 핵심 결론 |
|---|---|
| **환경** | 4 프로세스 (Gazebo / PX4 / Agent / ROS2) 협력. RTF=1, UXRCE_DDS_SYNCT=0, ODE physics, camera off — 안정 4 조건이 world/airframe 에 박혀 있음. |
| **드론 제어** | 5단 cascade (pos→vel→att→rate→mixer). 우리는 [1] 또는 [2] 진입. 좌표계 ENU/NED/FRD 혼용 — 부호 변환이 silent bug. |
| **MDP** | obs 17-d (물리+비전+payload+target+CCIP). action 5-d 중 drop 은 dead. reward 4 Layer 의 Layer 4 가 한 ep reward 의 ~80%. |
| **SAC + PER** | sparse + large terminal reward 가 표준 SAC 가정 위반 → 우리 4가지 customization (Damped α + Hard cap + Target Q clip + Huber). |
| **학습 메커니즘** | off-policy fine-tune 의 옵션 3 (weights+buffer load) 가 catastrophic forgetting 의 직격탄. v9a 의 실패 메커니즘. |

### v10 design 으로 가져갈 4 가지 question

**Q1. Reward 구조** — Layer 4 의 ±200 sparse signal 을 dense 로 분산할 것인가? (예: drop window 진입 시점 + drop quality 분리)

**Q2. Action space** — action[4] (drop) 를 살릴 것인가, 더 deep 하게 hard-code 할 것인가? (다른 옵션: drop timing 을 discrete signal 로 분리)

**Q3. Fine-tune 정책** — 옵션 1 (fresh) 만 허용할 것인가, 옵션 2/3 의 안전한 사용법을 정립할 것인가? (예: buffer 일부 prune + critic 재초기화)

**Q4. Curriculum 구조** — 2 단계 모드 분리의 정확한 의미? (시간축 / 정책축 / 커리큘럼축 / 상태머신축 중 어느 쪽)

## 학습 원칙

- 카테고리당 여러 세션. 매 세션 끝에 self-check 질문 (1.1~1.4 답안 inline 포함).
- 우리 코드/세팅과 반드시 연결 (이론만 안 다룸).
- 막히면 다음 카테고리로 넘어가지 않음.
- v10 design 은 5 카테고리 완주 후에 시작. ← **지금 여기**
