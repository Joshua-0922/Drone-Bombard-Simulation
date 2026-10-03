---
date: 2026-10-03
tags: [research, sim2real, hardware, px4, experiment-design]
status: active
type: research
---

> 2026-10-03 다중 에이전트 조사(하드웨어·선행연구·펌웨어 1차 조사 → 상호 결과 공유·수정 + 스케일 판단 → 종합 → 비판 검토 → 최종본)의 산출물입니다.
> 관련: [[research/sim2real_strategy]] · [[research/sim2real_hardware_spec]] · [[research/sim2real_scale_wind_retrain]] · [[research/sim2real_firmware_stack]] · [[research/sim2real_strategy_review]] · [[research/sim2real_gap]] · [[research/perception_integration_survey]] · [[00_index]]

# sim2real (2) 선행연구의 하드웨어 실험 구조 — 기체·펌웨어·sim2real 기법·반복 횟수·바람 생성

조사 범위는 2020년 이후 논문입니다. 주제는 멀티로터 sim2real, 드론 투하·투척, 그리고 4족·휴머노이드 로봇의 던지기로 한정했습니다. 출처는 WebSearch와 WebFetch로 확인했고, 원문에서 확인하지 못한 항목은 "미확인"으로 적었습니다. 게재처가 확인되지 않은 프리프린트는 표에 **[프리프린트]**로 표시했고 참고용으로만 씁니다.

---

## 0. 교차 검토로 바뀐 점

| # | 바뀐 내용 | 계기 | 근거 |
|---|---|---|---|
| 1 | **Learning to Throw의 저수준 제어기를 확정했습니다.** 원문 표현은 "identified **Betaflight** CTBR controller"입니다. FC 하드웨어 이름, 정책 실행 위치, 모캡 이름은 원문에 없어 미확인으로 둡니다. 페이로드는 테니스공입니다. | (4)는 펌웨어 이름을 미확인이라고 적었습니다. 원문을 다시 확인해 해소했습니다. | [arXiv 2606.27603](https://arxiv.org/html/2606.27603) |
| 2 | **AeroThrow의 NMPC 출력은 집단추력 + 바디 각속도(CTBR)이고, 제어 할당(control allocation)을 거쳐 로터로 갑니다.** FC는 NxtPX4v2이며, PX4 계열이라는 것은 이름으로 미루어 본 추정이고 원문에 명시되어 있지 않습니다. 릴리즈 지연은 **보고되어 있지 않습니다.** | (4)의 제어기 비교 작업 | [arXiv 2507.13903](https://arxiv.org/html/2507.13903v1) |
| 3 | **Neural-Fly 기체를 2.6 kg에서 2.53 kg으로 고쳤습니다.** 구성은 Pixhawk 4 + PX4 + Raspberry Pi 4, T/W 2.2입니다. 다만 출처는 arXiv PDF 보충자료의 검색 요약입니다. PDF가 커서 원문을 직접 열지 못했으므로 2차 확인으로 분류합니다. 이 논문은 **우리 시뮬 기체(2.07 kg, T/W 2.0)와 가장 가까운 실기 사례**이고, **PX4 + RPi급 탑재 컴퓨터**로 학습 제어기를 돌렸습니다. 그래서 (1)의 RPi 5 권고와 (4)의 PX4 선택을 함께 뒷받침합니다. | (1)의 컴패니언 결론, (4)의 펌웨어 결론 | [arXiv PDF 2205.06908](https://arxiv.org/pdf/2205.06908), [Caltech](https://www.caltech.edu/about/news/rapid-adaptation-of-deep-learning-teaches-drones-to-survive-any-weather) |
| 4 | **DCE의 속도 제어기 랜덤화를 원문으로 확인했습니다.** 원문은 "parameters of the velocity controller are randomized to vary the step-response time constant … by ±10%"입니다. 시정수를 **어떻게 동정했는지는 원문에 없습니다.** 따라서 우리 계단응답 동정 절차는 (4)의 7개 운용점 계획을 따릅니다. | (4) 방안 C | [ar5iv 2402.03947](https://ar5iv.labs.arxiv.org/html/2402.03947) |
| 5 | **P1(sim 재정렬)을 (4)의 방안 B로 통일했습니다.** 실기 PX4는 기본값에서 속도 루프가 PID(xy I 0.4, D 0.2 / z I 2.0)입니다. 시뮬은 순수 P입니다. 선행연구 가운데 오토파일럿 적분을 끈 사례는 없습니다. Ishihara는 PX4 PID를 그대로 두고 잔차를 더했고, DCE는 오토파일럿 응답을 랜덤화했습니다. 그래서 **방안 A(PX4 I=0)는 첫 비행과 ablation용으로만 쓰고, 바람 본실험은 방안 B(시뮬에 PID를 넣고 미세조정)로 갑니다.** 적분을 끄면 바람 정상오차를 지우는 능력이 사라지므로, 바람 실험의 주 구성으로 쓰기에는 부적절합니다. | (4) §1.3–1.4 | 위 각 논문 |
| 6 | **실기 질량을 다르게 다룹니다.** 1차 결과에서는 "축소 시나리오"만 언급했습니다. (1)의 결론에 따라 실기는 X500 V2급(1.3–1.5 kg 추정, T/W 약 3.5 추정)이 됩니다. 시뮬의 2.07 kg, T/W 2.0과 다릅니다. SimpleFlight는 "정확히 잴 수 있는 파라미터에 DR을 주면 오히려 해롭다"고 보고했습니다. 이를 따라 **DR 범위를 넓혀 실기를 덮는 방식이 아니라, 실측값으로 시뮬 기체를 바꾸고 L0를 미세조정한 뒤 GRU를 재적합하는 방식**을 권장합니다. | (1) §3·결론, (4) §6 | [SimpleFlight](https://arxiv.org/html/2412.11764v4) |
| 7 | **바람 층화 구간을 바꿨습니다.** (1)이 권고한 운용 상한(평균풍 5 m/s 이하)을 반영해 "2 미만 / 2–4 / 4 이상"을 **"2 미만 / 2–4 / 4–5(상한)"**으로 바꿨습니다. 선행 실기의 바람 크기도 이 범위 안에 있습니다. Extreme Adaptation은 3.5 m/s, Ishihara 실내는 7 m/s 미만입니다. | (1) §2.4 | 해당 논문 |
| 8 | **반복 횟수를 비행 횟수로 환산했습니다.** 페이로드가 하나이므로 한 소티(이륙에서 착륙까지 한 번의 비행)에 투하는 한 번입니다. 따라서 투하 100회는 소티 100회입니다. 배터리 교체 주기는 실측해야 하므로 미확인입니다. | (1)의 기체 구성 | 추론 |
| 9 | **릴리즈 지연 측정을 우리 기여로 다룹니다.** 선행 드론 투척 두 편 모두 지연 측정 방법을 보고하지 않았습니다. Learning to Throw는 "identified 0.11 s"라고만 적었고, AeroThrow는 언급이 없습니다. 그래서 (1)·(4)가 제안한 고속촬영 30회 이상과 ulog 가속 계단 교차검증을 프로토콜에 넣고, 이를 선행보다 엄밀한 점으로 적습니다. | (1) §5, (4) §5 | 위 두 논문 |
| 10 | **sim-to-sim 통과 기준을 정리했습니다.** (4)는 "Gazebo의 CEP50이 Isaac 대비 1.5배 이내"를 제안했습니다. 선행연구에 정량 통과 기준을 둔 사례는 **찾지 못했습니다.** RESC와 Narrow Gaps는 정성적인 성공만 보고합니다. 따라서 1.5배는 우리 자체 기준이라고 명시합니다. 참고로 Gazebo x500 모델(본체 2.0 kg)은 Isaac 동역학과 일부러 다른 동역학 역할도 겸합니다. Narrow Gaps가 같은 방식을 썼습니다. | (4) §4.3 | [Narrow Gaps](https://ar5iv.labs.arxiv.org/html/2302.11233) |

추가 검색으로 투하 정확도를 바람 조건별로 실측한 해외 학술 논문(2020년 이후, IROS급 또는 해외 저널)은 여전히 찾지 못했습니다. FPV 드론에서 바람을 고려한 투하 알고리즘이 우크라이나 대학 저장소에 하나 있습니다([KAI](https://er.kai.edu.ua/handle/KAI/70011)). 게재처와 실험 내용은 미확인이고, 조사 기준(해외 주요 학술지·학회) 밖이라 제외했습니다.

---

## A. 논문별 정리

### A-1. 드론 투척·투하

| 항목 | **Learning to Throw** (Zhai, Raimondi, Ren, Geles, Armati, Xing, Scaramuzza, 2026) [프리프린트] | **AeroThrow** (Li, Chen, Lin, Ye, Lyu, RA-L 2025) | **Parabolic Airdrop Trajectory Planning for Multirotor UAVs** (IEEE Access 2022) |
|---|---|---|---|
| URL | https://arxiv.org/html/2606.27603 | https://arxiv.org/html/2507.13903v1 (RA-L 게재는 2606.27603 참고문헌으로 확인) | https://doaj.org/article/3672c733fa0d4c8da739aaada38b58dd |
| 기체 | 0.21 kg, T/W 6.8, 팔 19.4 cm. 페이로드는 케이블에 매단 테니스공 | 쿼드 + 델타 팔(Dynamixel XL430 3개), **약 1.59 kg**(6S 포함), 전자석 | 미확인 |
| 펌웨어·저수준 | **Betaflight CTBR**(실측 동정 후 시뮬에 재현). FC 하드웨어는 미확인 | **NxtPX4v2** FC. NMPC(ACADO/qpOASES)가 **CTBR**을 출력하고 제어 할당으로 로터에 전달 | 미확인 |
| 정책 주기·위치 | CTBR + 릴리즈 래치 50 Hz. 실행 위치 미확인 | **Jetson Orin NX 16GB** 탑재, ROS Noetic | 미확인 |
| 상태추정 | 미확인 | **NOKOV 모캡 + IMU EKF** | 미확인 |
| 시뮬레이터 | **Isaac Lab**. 기체는 Warp 해석 모델, 로프는 PhysX 500 Hz | 시뮬 검증 있음, 상세 미확인 | 시뮬 반복성 시험 |
| sim-to-sim | 없음(Isaac에서 실기로 직행) | 없음 | 미확인 |
| sim2real 기법 | **SysID**: 모터 1차 지연, 전압 의존 추력·토크 맵, Betaflight 제어기. **DR**: 질량 ±5%, 케이블 길이 ±6%, 행동 잡음 0.025, 관측 잡음 0.01. **릴리즈 지연 0.11 s**(동정값, 측정 방법 미보고) | 계층적 외란 보상, 릴리즈 타이밍에 둔감한 궤적, NMPC 예측으로 릴리즈 시점 온라인 재평가. 지연 측정은 미보고 | 미확인 |
| 실험 변수 | 표적 1.5 / 2.0 / 2.5 m(2.5 m는 OOD), 상태 기반 vs 비전 정책, 시뮬 구성요소 ablation | 궤적 3종(v_r 4.3 / 2.3 / 2.6 m/s), 릴리즈 트리거 방식, 미지 페이로드(200 g 철 큐브) | 실내·실외, 반복성, 장애물 회피 |
| 반복 | **표적당 10회**, ablation은 조건당 5회 | **방법당 약 10회**(Fig. 8). 총 횟수는 미기재 | 검색 요약에 "56회 중 39회 성공"이 있으나 원문 미확인 |
| 착지 계측 | 미보고 | **바닥 판지에 착지 흔적 기록** | 미확인 |
| 정확도 | 2.0 m 표적 **0.105 ± 0.099 m**, 1.5 m 0.082, 2.5 m 0.133. MPC 대비 착지오차 −50% | 궤적별 MEAN/MAX **3.3–83.8 cm** | 검색 요약에 평균 0.1885 m가 있으나 원문 미확인 |
| 시뮬 대비 | **Table V**: 공력을 빼면 0.342, 모터 모델을 빼면 0.254, 저수준 제어기를 이상화하면 **0.386**, 강체 케이블이면 0.306 m. 하나만 빠져도 2.4–3.7배 악화 | 직접 비교 없음 | 미확인 |
| 실기 이후 | zero-shot. 한계로 정지 표적, 무항력 탄도, 고정 페이로드를 명시 | 미확인 | 미확인 |
| 바람 | 없음 | 없음 | 미확인 |

### A-2. 드론 RL·학습 제어의 sim2real

| 논문 | 기체 / 펌웨어 | 상위 출력 / 실행 위치 | 상태추정 | 학습 sim → 중간 단계 | sim2real 기법 | 실험 변수·반복 | 결과 |
|---|---|---|---|---|---|---|---|
| **Swift**, *Nature* 2023 [PMC](https://pmc.ncbi.nlm.nih.gov/articles/10468397) | 870 g, T/W 4.1, Betaflight | CTBR 약 125 Hz, Jetson TX2 탑재 | T265 VIO + 게이트 검출 KF | 자체 sim → 실기 | 실비행 약 50 s를 모캡 참값으로 기록해 **잔차 모델**(지각은 GP, 동역학은 k-NN)을 만들고, sim에 넣어 **정책 미세조정** | 챔피언 3명과 9/7/9 레이스 | 5/9, 4/7, 6/9 승 |
| **SimpleFlight**, RA-L 2025 [arXiv](https://arxiv.org/html/2412.11764v4) | Crazyflie 2.1, 보조로 PX4 + Orin 기체 | CTBR 100 Hz, 지상 PC → 무선 | OptiTrack 100 Hz | OmniDrones(Isaac) → 실기 | **SysID(질량·관성·k_f·모터 시정수) + 선택적 DR**. 민감한 추력계수만 ±10–30%. 잘 잴 수 있는 파라미터를 DR하면 해롭다 | 궤적별 3–10회 | 8자 0.028 m, zero-shot |
| **Neural-Fly**, *Sci. Robotics* 2022 [arXiv](https://arxiv.org/abs/2205.06908) | **2.53 kg, T/W 2.2, Pixhawk 4 / PX4 + RPi 4** (보충자료 검색 요약 기준, 2차 확인) | RPi 4에서 적응 제어 | OptiTrack | 실비행 데이터 12분(정적 풍속 여러 조건)으로 학습 | 메타학습 표현 + 온라인 적응 | CAST 풍동(1,296개 팬, 최대 12.1 m/s), 학습 풍속은 시험 최대의 절반, 실외 병행 | 추적오차 2.5–4배 감소 |
| **DATT**, CoRL 2023 [arXiv](https://arxiv.org/html/2310.09053v3) | Crazyflie 2.1(40 g) | CTBR 50 Hz, 지상 PC | OptiTrack 50 Hz | 자체 sim → 실기 | 외력 DR ±3.5 m/s², L1 외란 추정기 | 선풍기 3대 + 판지판(풍속 미보고). 궤적 10개 × 2회 = **조건당 20회** | 적응 NMPC 대비 우위 |
| **Extreme Adaptation**, T-RO 2025 [arXiv](https://arxiv.org/html/2409.12949v2) | 985 g / 267 g, PixRacer | **모터 속도 500 Hz**, RB5 탑재 | 모캡은 평가용 | Flightmare → 실기 | **크기 스케일 DR**(0.226–0.95 kg) + 적응 모듈 | 편심 페이로드 200 g, 모터 80% 제한, 자연풍 최대 3.5 m/s. **방법·과제당 3회** | sim과 real 오차의 상관 r = 0.652 (p = 0.002) |
| **DCE**, ICRA 2024 [ar5iv](https://ar5iv.labs.arxiv.org/html/2402.03947) | **1.2 kg**, PixRacer **ArduPilot** | **속도 + 요레이트 10 Hz**, Orin NX 탑재(15 ms) | T265 + IMU | Aerial Gym(Isaac Gym) → 실기 | **속도 제어기 계단응답 시정수 ±10% 랜덤화**(동정 방법 미기재), 외력·카메라 자세 랜덤 | 실험 3건, 반복 수 미기재 | 정성적 성공 |
| **Aerial Gym**, RA-L 2025 [arXiv](https://arxiv.org/html/2503.01471v1) | VOXL2 Mini + PX4 / ArduPilot | 모터 추력 / 속도 | Qualisys / ROVIO | Isaac Gym → 실기 | DR 상세 미확인 | 시드 5개 | 위치오차 0.09 m |
| **NN mode for PX4** [프리프린트] [arXiv](https://arxiv.org/html/2505.00432) | 1.2 kg, Pixracer Pro | **FC 위 PX4 모듈**(TFLite, 93 µs) | Qualisys | Aerial Gym → 실기 | 질량은 저울, 관성은 CAD, 추력계수는 호버에서 동정 | 1회 | 모터 시정수 오추정으로 명령 분산 차이 |
| **RESC**, RA-L 2025 [arXiv](https://arxiv.org/html/2408.00275) | 1.64 kg, **PX4 rate + allocator** | CTBR 50 Hz | NOKOV | **Gym → Gazebo + PX4 SITL → 실기** | 질량·관성·추력맵 DR, **행동 지연 큐** | 미확인 | 실기 성공(정성) |
| **Narrow Gaps**, RA-L 2023 [ar5iv](https://ar5iv.labs.arxiv.org/html/2302.11233) | 1.1 kg, T/W 3.5, Pixracer(PX4), Manifold 2-C | 학습 20 Hz / 실기 50 Hz | 탑재 D455 | 자체 sim → **Gazebo 9 + PX4 v1.11 SITL**(일부러 다른 질량 0.9 kg) → 실기 | 동역학·관측 잡음 DR | **87회** | 87.36% |
| **Residual RL wind**, Ishihara 2023 [프리프린트] [ar5iv](https://ar5iv.labs.arxiv.org/html/2308.01648) | 5.12 kg(페이로드 포함), **PX4 캐스케이드 PID 유지** | PID 출력에 **가산 잔차** 10 Hz | 미확인 | **Gazebo + PX4에서 학습** | 질량·양력 50–150%에 강건 | 실내 블로어 돌풍(7 m/s 미만), 실외 팬 약 13 m/s | 위치편차 −41% / −35% |
| **E2E-Fly** 2026 [프리프린트] [arXiv](https://arxiv.org/html/2604.12916) | 750 / 470 g, Betaflight | CTBR 30 Hz | 모캡 | VisFly → AirSim → HIL → 실기 | **계단응답으로 통신지연 정렬**(지상 약 90 ms, 탑재 30 ms 미만) | 6개 과제 | 상태 기반 과제는 zero-shot |
| **Learning to Fly in Seconds**, RA-L 2024 [arXiv](https://arxiv.org/html/2311.13081v2) | Crazyflie | RPM, MCU 탑재 | 미확인 | 자체 sim → 실기 | 커리큘럼, 비대칭 AC | 미확인 | 경쟁력 있다고 보고 |

### A-3. 4족·휴머노이드 로봇의 던지기

| 논문 | 로봇 / 시뮬 | sim2real | 실험 | 계측 | 결과 |
|---|---|---|---|---|---|
| **Ma, Liu, Qu, Hutter 2025** [프리프린트] [arXiv](https://arxiv.org/html/2506.16986v3) | ANYmal + DynaArm, legged_gym | DR, 액추에이터 네트워크, 물체 질량 랜덤, 고주파 잔차 정책 | **표적 4곳 × 10회 = 40회**(4 m, 6 m). 인간 비교 125회, 실외 5–7 m | **AprilTag** | 4 m 0.429 m, 6 m 0.276 m(nominal 0.710 / 0.685). "시뮬이 실기보다 상당히 좋다" |
| **Munn et al. 2024–25** [프리프린트] [arXiv](https://arxiv.org/html/2410.05681v2) | 휴머노이드, ANYmal + Kinova, Isaac Lab | DR, **릴리즈 지연 100 ms 구현** | 4 m 표적 5회 | **모캡으로 공 궤적 추적** | 실기 0.80 m(시뮬 0.73 m). 센서 정렬 오차를 원인으로 지목 |

---

## B. 질문별 정리

### (a) 우리 실기 구성과 선행 기체 비교 (교차 검토 신규)

| 기준 | 우리 시뮬 | (1) 권장 실기 (X500 V2) | 가장 가까운 선행 |
|---|---|---|---|
| 질량 | 2.07 + 0.1 kg | 1.3–1.5 kg(추정) | Neural-Fly 2.53 kg, RESC 1.64, AeroThrow 1.59, DCE 1.2, Narrow Gaps 1.1 |
| T/W | 2.0 | 약 3.5(추정) | Neural-Fly 2.2, Narrow Gaps 3.5 |
| 펌웨어·명령 수준 | 순수 P 캐스케이드, 속도 명령 | PX4 offboard 속도 (PID) | DCE(ArduPilot 속도 10 Hz), Ishihara(PX4 PID + 잔차), Neural-Fly(PX4) |
| 컴패니언 | – | RPi 5 | Neural-Fly RPi 4. DCE와 AeroThrow의 Orin NX는 깊이 영상 또는 NMPC 부하 때문에 쓴 것 |

판정은 다음과 같습니다.
- 1–2.5 kg대 PX4/ArduPilot 기체에서 속도 수준 학습 정책을 돌린 선행이 여럿 있으므로, (1)·(4)의 구성은 선행 범위 안에 있습니다.
- 선행에서 CTBR(Betaflight)은 T/W 4 이상의 경량·고기동 기체에서만 쓰였습니다. (4)가 Betaflight를 기각한 판단과 맞습니다.

### (b) 실기에서 바람을 만들고 측정한 방법

| 방식 | 사례 | 풍속 측정 |
|---|---|---|
| 팬 어레이 풍동 | Neural-Fly(최대 12.1 m/s) | 풍동 설정값 |
| 선풍기 | DATT | 미보고 |
| 블로어·대형 팬 | Ishihara(7 m/s 미만, 약 13 m/s) | 수치만 보고, 풍속계 사양 미보고 |
| 자연풍 | Extreme Adaptation(최대 3.5 m/s) | 수치만 보고 |
| 바람 없음 | 투척·투하 실기 전부(Learning to Throw, AeroThrow, Ma, Munn) | – |

투하 정확도를 바람 조건별로 실측한 선행은 찾지 못했습니다. 따라서 **실외 자연풍 + 풍속계 층화**가 우리 실기 기여가 됩니다. 풍속계를 정책 입력이 아닌 기록용으로 쓰는 것은 (1)의 권고와 같습니다.

### (c) 반복 횟수
- 투척·투하 정확도 논문은 **조건당 10회**가 표준입니다. ablation은 5회입니다.
- 제어 추종 논문은 3–20회입니다.
- 통과·성공률형 논문은 87회 수준입니다.

### (d) 착지 계측과 지표
- 착지 계측 방법은 판지 흔적(AeroThrow), AprilTag(Ma), 모캡 궤적(Munn)이 있습니다. Learning to Throw는 보고하지 않았습니다.
- 지표는 평균 ± 표준편차가 주를 이루고, 최대오차를 같이 내기도 합니다. CEP를 쓴 로봇 논문은 없습니다.
- (1)이 제시한 측정오차 2–3 cm 이하 요구는 우리 CEP50 0.204 m를 기준으로 한 경험칙이며, 선행 근거는 없습니다.

### (e) sim-to-sim 단계

| 경로 | 논문 |
|---|---|
| Isaac 계열에서 실기로 직행(저수준 실측 동정으로 대체) | Learning to Throw, SimpleFlight, DCE, Aerial Gym, NN-PX4 |
| 자체 sim → Gazebo/PX4 SITL → 실기 | RESC, Narrow Gaps |
| Gazebo/PX4에서 학습 | Ishihara |
| sim → AirSim → HIL → 실기 | E2E-Fly |
| 실측 잔차 모델로 sim 보정 후 재학습 | Swift |

Gazebo SITL 단계는 **인터페이스·지연·좌표계 검증용**이었습니다. 우리처럼 PX4 offboard를 쓰는 경우에는 (4)의 판단대로 이 단계가 특히 유용합니다. 선행 근거는 RESC와 Narrow Gaps입니다.

### (f) 릴리즈 지연

| 논문 | 지연 | 측정 방법 |
|---|---|---|
| Learning to Throw | 0.11 s | "identified"라고만 적음. 방법 미보고 |
| AeroThrow | 미보고 | – |
| Munn | 100 ms 구현 | 미보고 |
| 우리(시뮬) | 0.22 ± 0.05 s 가정 | 실측 예정. 고속촬영 30회 이상 + ulog 가속 계단 교차검증, (1)·(4) 제안 |

---

## C. 공통 패턴과 우리에게 주는 함의

1. **정확도를 좌우하는 것은 저수준 체인의 실측 동정입니다.** Learning to Throw에서 제어기를 이상화하면 3.7배 나빠졌습니다. SimpleFlight도 SysID를 먼저 하고 DR은 선택적으로만 줍니다. 우리에게는 다음 두 가지가 결정적입니다. (4)가 확인한 **PX4 PID와 시뮬 순수 P의 차이**, 그리고 **X500(1.3–1.5 kg, T/W 약 3.5)과 시뮬 기체(2.07 kg, T/W 2.0)의 차이**입니다.
2. **명령 수준이 맞습니다.** 우리 L0(10 Hz 속도 + 요레이트)는 DCE와 같은 구조입니다. 해법도 같습니다. 오토파일럿 속도 루프의 응답을 동정하고 ±10%로 랜덤화합니다.
3. **잔차를 기존 오토파일럿 위에 얹는 구조에는 선례가 있습니다.** Ishihara는 PX4 PID를 유지한 채 잔차를 더했습니다. 우리 GRU 잔차는 착탄점 공간에 있어 위치가 다르지만, PX4 PID를 끄지 않는다는 점은 같습니다.
4. **실행 위치는 탑재 컴퓨터입니다.** RPi급(Neural-Fly)이면 충분합니다. 정책이 작기 때문입니다((1) §4.1).
5. **지연은 명시적으로 모델링합니다.** 릴리즈 지연, 통신 지연 계단응답 정렬(E2E-Fly), 행동 지연 큐(RESC)가 그 예입니다. (4)의 관측·행동 지연 DR(0–1 step)이 여기에 해당합니다.
6. **실기 이후 단계는 대부분 zero-shot 보고와 한계 서술입니다.** 재보정 후 재학습한 사례는 Swift 하나입니다.
7. **투척 실기는 모두 실내 1.5–6 m, 무풍입니다.** 우리 체제(18–22 m 진입, 3–8 m 투하 고도, 자연풍)는 선행과 겹치지 않습니다. 이 점이 기여이자 위험 요소입니다. 통제 불가능한 풍황, 측위 오차, 넓은 비행 구역이 위험입니다.

---

## D. 우리 논문에 권장하는 실험 프로토콜 (교차 검토 반영)

| 단계 | 내용 | 근거 / 담당 연계 |
|---|---|---|
| **P0 실측 동정** | 질량(저울), 관성(CAD 또는 진자), 호버 추력, 배터리 전압별 추력. **PX4 속도 루프 계단응답** 7개 운용점((4) 계획: 전진 {1,2,4}, 측방 {1.5,3}, 수직 {±1,±3}, 대각). 관측→행동 지연(`timestamp_sample`). **릴리즈 지연은 고속촬영 30회 이상으로 평균과 표준편차.** 페이로드 k/m는 낙하 시험으로 잼 | Learning to Throw Table V, SimpleFlight, DCE. (1) §5, (4) §2.2·§5 |
| **P1 sim 재정렬** | ① Isaac 기체를 실측 X500 값(질량·관성·T/W)으로 교체. ② `_run_velocity_controller`에 PX4형 I·D, anti-windup, z 우선 포화를 넣음. ③ 시정수 ±10% DR과 지연 DR 0–1 step. ④ 기울기 35°는 시뮬과 PX4 양쪽에서 맞춤(`MPC_TILTMAX_AIR=35`). ⑤ L0는 기존 체크포인트에서 미세조정하고 GRU는 오프라인 재적합. DR 범위를 넓혀 질량 차이를 덮는 방식은 쓰지 않음 | (4) 방안 B, SimpleFlight(선택적 DR) |
| **P2 sim-to-sim** | 실기와 **같은 rclpy 노드**로 Gazebo Harmonic + PX4 v1.15 SITL에서 폐루프 투하. 검증 항목은 좌표계, offboard 인계, 하강 제한(`MPC_Z_VEL_MAX_DN=3`), 서보 명령 체인, 안전 로직. 통과 기준은 Isaac 대비 CEP50 1.5배 이내이며 **선행에 없는 우리 자체 기준**임. 방안 A(PX4 I=0)와 방안 B를 비교하는 ablation도 이 단계에서 실시 | RESC, Narrow Gaps, (4) §4 |
| **P3 실기 무풍·저풍 기준선** | 지상 부호 시험과 섀도 비행((4) §3)을 먼저 함. 이후 평균풍 2 m/s 미만인 날 원 스케일로 투하. 시나리오 스케일 결정은 (3)이 하며, 실외 RTK에서는 원 스케일이 가능하다는 것이 (1)의 결론. **표적은 RTK로 측량한 좌표**, 정책은 RPi 5에서 실행 | 선행 표준, (1) §6 |
| **P4 바람 조건** | 실외 자연풍. 투하 고도 근처 마스트에 2D 초음파 풍속계를 둠(기록용). 사후에 **2 미만 / 2–4 / 4–5 m/s**로 층화. 운용 상한은 평균풍 5 m/s | (1) §2.4. 투하 × 바람 실측 선행 없음 |
| **반복·비교** | 팔: L0(`--no_residual`에 해당), L0 + GRU-S + EMA, T2 규칙, T0 정지 투하. **팔당 풍속 구간당 10회 이상**(선행 표준), 주 비교(L0 대 L0+GRU-S)는 20회 이상. **ABAB 교대**로 풍황 드리프트를 블록화(우리 paired 원칙). 1투하 = 1소티 | Learning to Throw, AeroThrow, Ma |
| **계측** | 착지점 측정 오차 2–3 cm 이하. RTK 측량봉 또는 고정 하방 카메라 + 격자/AprilTag. 첫 접지점 규칙을 미리 정함. 지표는 CEP50 / CEP90 / 성공률@0.5 m와 평균 ± 표준편차·최대를 함께 보고(선행 비교용) | AeroThrow, Ma, (1) §5 |
| **sim 대비 보고** | 비행마다 측정 풍속(평균과 돌풍 시계열)을 Isaac에 재생해 예측 착탄점을 만들고, 실측과의 차이와 피어슨 r을 보고 | Extreme Adaptation |
| **실기 이후** | 차이가 크면 비행 로그(RTK와 ulog)로 잔차 동역학을 만들어 sim에 넣고 재학습(Swift 방식). 한계로 실외 풍장의 비균일성, 표적 참값 사용(트랙 B 미통합), 단일 기체를 명시 | Swift |

**권장 횟수(초안):** 4개 팔 × 3개 풍속 구간 × 10회 = 120회입니다. 여기에 주 비교 보강으로 2개 팔 × 3개 구간 × 10회 = 60회를 더하면 약 **180소티**입니다. 4–5 m/s 구간은 그런 날이 드물어 채우기 어려울 수 있습니다. 그 경우 2개 구간 × 4개 팔 × 10회 + 보강 = 약 100소티로 줄입니다. 검정력 계산은 실측 분산을 확보한 뒤에 하므로 이 숫자는 확정이 아닙니다. 하루에 가능한 소티 수는 배터리와 페이로드 재장전 시간에 달려 있어 미확인입니다.

---

## E. 다른 담당에게 넘기는 사항

- **(3)에 넘김**
  - 질량과 T/W 차이는 DR 범위를 넓혀서가 아니라 실측값으로 재정렬해 해결합니다(L0 미세조정 + GRU 재적합). 근거는 SimpleFlight입니다.
  - 실외 RTK라면 원 스케일 유지가 선행과 충돌하지 않습니다. 선행 실기는 모두 실내라 스케일에 관한 선례는 없습니다.
  - 바람 운용 상한은 5 m/s입니다.
- **(4)와 일치**
  - PX4 v1.15 offboard 속도 명령, Gazebo SITL 단계, 방안 B 본선과 방안 A ablation, RPi에서 numpy 추론을 씁니다.
  - 선행 근거는 DCE, Ishihara, RESC, Narrow Gaps, Neural-Fly입니다.
- **(1)과 일치**
  - X500 V2 + RPi 5 구성입니다. Neural-Fly(PX4 + RPi 4, 2.53 kg)가 가장 가까운 선례입니다.

---

## 출처
- https://arxiv.org/html/2606.27603
- https://arxiv.org/html/2507.13903v1
- https://doaj.org/article/3672c733fa0d4c8da739aaada38b58dd
- https://pmc.ncbi.nlm.nih.gov/articles/10468397
- https://arxiv.org/html/2412.11764v4
- https://arxiv.org/abs/2205.06908 · https://arxiv.org/pdf/2205.06908 · https://www.caltech.edu/about/news/rapid-adaptation-of-deep-learning-teaches-drones-to-survive-any-weather
- https://arxiv.org/html/2310.09053v3
- https://arxiv.org/html/2409.12949v2
- https://ar5iv.labs.arxiv.org/html/2402.03947
- https://arxiv.org/html/2503.01471v1
- https://arxiv.org/html/2505.00432
- https://arxiv.org/html/2408.00275
- https://ar5iv.labs.arxiv.org/html/2302.11233
- https://ar5iv.labs.arxiv.org/html/2308.01648
- https://arxiv.org/html/2604.12916
- https://arxiv.org/html/2311.13081v2
- https://arxiv.org/html/2506.16986v3
- https://arxiv.org/html/2410.05681v2
- (제외 대상, 참고) https://er.kai.edu.ua/handle/KAI/70011

참고한 내부 노트(읽기만 함):
- /opt/drone-bombard/Drone-Bombard-Simulation/notes/research/related_work_survey.md
- /opt/drone-bombard/Drone-Bombard-Simulation/notes/research/sim2real_gap.md