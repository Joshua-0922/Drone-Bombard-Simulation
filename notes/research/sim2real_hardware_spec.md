---
date: 2026-10-03
tags: [research, sim2real, hardware, px4, experiment-design]
status: active
type: research
---

> 2026-10-03 다중 에이전트 조사(하드웨어·선행연구·펌웨어 1차 조사 → 상호 결과 공유·수정 + 스케일 판단 → 종합 → 비판 검토 → 최종본)의 산출물입니다.
> 관련: [[research/sim2real_strategy]] · [[research/sim2real_prior_hw_experiments]] · [[research/sim2real_scale_wind_retrain]] · [[research/sim2real_firmware_stack]] · [[research/sim2real_strategy_review]] · [[research/sim2real_gap]] · [[research/perception_integration_survey]] · [[00_index]]

# (1) 하드웨어 사양 조사, 교차 검토 반영본: 트랙 A(표적 참값) 실기 구성

## 교차 검토로 바뀐 점

| # | 1차 결론 | 바뀐 결론 | 바뀐 이유 |
|---|---|---|---|
| 1 | 투하 기구를 PX4 `PD_GRIPPER_*`(payload_deliverer)와 `MAV_CMD_DO_GRIPPER`로 구동 | **`VEHICLE_CMD_DO_SET_ACTUATOR`(187)로 구동하고, 출력 함수를 "Peripheral via Actuator Set n"으로 둔 PWM 서보**를 1순위로 둡니다. gripper 방식은 커스텀 빌드가 필요한 대안입니다 | (4)가 PX4 v1.15.4 소스를 확인했습니다. `CONFIG_MODULES_PAYLOAD_DELIVERER=y`는 sitl 보드에만 있고, fmu-v6c/v6x 기본 빌드에는 없습니다. Actuator Set 방식은 기본 빌드에서 동작하고, 명령이 ulog 시간축에 남아 지연을 잴 수 있습니다 |
| 2 | 실기 T/W 약 3.5~3.8을 시뮬레이션의 2.0에 맞출지는 미정. PX4 스로틀 상한도 후보 | **PX4에서 추력을 제한해 T/W를 맞추지 않습니다.** 실측한 T/W·질량·관성을 Isaac에 넣고 L0를 미세조정합니다(4의 방안 B). `MPC_THR_MAX`는 안전용 상한으로만 씁니다 | (4)의 결론에 따르면 PX4 PID가 순수 P와 달라 L0 미세조정과 GRU 재적합은 어차피 필요합니다. `MPC_THR_MAX`는 정규화 스로틀(기본 0.9)로 z 성분을 먼저 포화시키는 값이라([PX4 controller diagrams](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)) 비선형 추력 곡선에서는 T/W 2.0을 정확히 재현하지 못합니다. 선행연구도 실측 동정 후 sim을 맞추는 것이 표준입니다(2의 C-1). Extreme Adaptation의 "모터 80% 제한"은 외란 시험 조건이었고 정합 수단이 아니었습니다 |
| 3 | 실기가 시뮬레이션 기체(2.07 kg)보다 가벼우므로 재동정 필요(근거는 일반론) | 결론은 같고 근거가 구체화됐습니다. **GRU는 바람을 자세 평균($mg\tan\theta = F_{wind}$)으로 추정하므로 $k_d/m$이 바뀌면 입력 분포가 바뀝니다.** 2 kg 미만 기체라면 (4)의 분류상 "L0 미세조정 + GRU 재적합"에 해당합니다. **2.07 kg에 가깝게 맞추려 밸러스트를 다는 방안은 기각합니다** | 어차피 재학습이 필요하므로 밸러스트는 이득이 없고 법적 여유만 줄입니다. 선행연구 기체 중 PX4 계열은 1.1~1.64 kg(AeroThrow 1.59, RESC 1.64, DCE·NN-PX4 1.2, Narrow Gaps 1.1)이라 X500급 1.3~1.5 kg이 주류입니다 |
| 4 | 고도는 RTK에만 의존 | **하방 거리센서(Benewake TFmini-S, 5 g)를 추가**합니다. 기록과 교차검증에 쓰고, EKF 융합은 선택입니다 | (4)가 관측 5번과 18번(표적 기준 고도)의 기준면 문제와 baro 드리프트를 지적하며 (1)에 조율을 요청했습니다 |
| 5 | 컴패니언은 RPi 5(정책 연산 근거) | **RPi 5를 유지합니다.** 근거가 두 가지 늘었습니다. (a) (4)는 정책을 순수 numpy(.npz)로 이식하므로 torch가 필요 없습니다. (b) Neural-Fly가 Pixhawk와 Raspberry Pi 4 조합으로 탑재 적응제어를 실증했습니다 | 2의 A-2, 4의 §2.3 |
| 6 | 실험 장소는 실내·실외 병렬 옵션 | **본 실증은 실외 원 스케일(RTK)로 합니다. 실내는 P0 동정(릴리즈 지연, 스텝 응답)용**입니다 | (2)에 따르면 투척·투하 실기는 전부 무풍 실내였고, 바람 조건별 투하 정확도 실측은 선행에 없습니다. 실외 바람이 기여점이 됩니다 |
| 7 | 반복 횟수와 소모품 수량 미정 | (2)의 초안(약 80~100회 투하)을 기준으로 **페이로드 10개 이상, 배터리 6개 이상(추정)**을 둡니다 | 2의 D |
| 8 | 서보 지연 기준 없음 | 선행 구현 지연(드론 0.11 s, 휴머노이드 100 ms)을 참고합니다. **평균보다 표준편차가 작은 서보**를 고르고 벤치 20~30회 측정 후 채택합니다 | 2의 A-1·A-3, 4의 §5 |
| 9 | PX4 파라미터는 범위 밖 | 하드웨어 설정 체크리스트에 `MPC_TILTMAX_AIR=35`와 `MPC_Z_VEL_MAX_DN=3`을 넣습니다. 하강 제한 기본값 1.5 m/s가 정책의 vz 명령을 자릅니다 | 4의 §1.2 |

다음 결론은 바뀌지 않았습니다. 기체는 X500 V2와 Pixhawk 6C(PX4)이고, 컴패니언은 RPi 5입니다. 법적으로 2 kg 이하이고, Jetson Nano(구형)는 비권장입니다. 제어기는 PX4 offboard 속도 setpoint이고, (2)·(4)와 모순이 없습니다. CTBR/Betaflight 계열은 고 T/W 소형기(0.21~0.87 kg, T/W 4~6.8)용입니다. 우리 행동 공간(속도와 요레이트, 10 Hz)은 DCE(1.2 kg, ArduPilot 속도 루프)와 같은 계열입니다.

---

## 요약

- **권장 구성 1안:**
  - **기체:** Holybro X500 V2 PX4 Dev Kit(Pixhawk 6C, PX4 v1.15.x)
  - **컴패니언:** Raspberry Pi 5(Ubuntu 22.04, ROS 2 Humble, uXRCE-DDS)
  - **측위:** RTK F9P와 하방 TFmini-S
  - **투하:** PWM 서보 래치(DO_SET_ACTUATOR)
  - **페이로드:** 0.1 kg 원통
  - **풍속 측정:** 지상 마스트에 2D 초음파 풍속계
  - **이륙중량 예상:** 1.3~1.5 kg(실측 필요)
- **라즈베리파이 가능 여부:** 트랙 A에서는 RPi 5(또는 RPi 4)로 충분합니다. 정책은 약 0.1 M MAC/step이고 numpy만으로 추론합니다. 트랙 B의 YOLO까지 기체 위에서 돌리려면 Orin Nano(Super)나 RPi 5 + Hailo가 필요합니다.
- **Jetson Nano(구형):** 동작은 하지만 비권장입니다. Ubuntu 18.04라 Humble을 Docker로만 쓸 수 있고, 단종됐습니다.
- **(3)·(5)에 넘기는 핵심:**
  - 실기는 시뮬레이션 기체의 복제가 아닙니다. 시뮬레이션 기체는 2.17 kg이라 4종 범위 밖입니다.
  - 따라서 질량·관성·추력곡선·PX4 속도 응답을 동정한 뒤 L0 미세조정과 GRU 재적합을 해야 합니다.
  - 실외 운용 상한은 평균풍 5 m/s 정도로 둡니다.

---

## 1. 법적 제약: 4종 범위

| 항목 | 내용 | 출처 |
|---|---|---|
| 4종 무인동력비행장치 | 최대이륙중량 **250 g 초과 ~ 2 kg 이하** | [한국교통안전공단 조종자 증명](https://main.kotsa.or.kr/portal/contents.do?menuCode=02020200), [국토부 정책Q&A](https://www.molit.go.kr/USR/policyTarget/dtl.jsp?idx=584) |
| 취득 방법 | TS배움터 온라인 교육 이수 | [TS배움터](https://edu.kotsa.or.kr/user/Main.do), [서울시 미디어허브](https://mediahub.seoul.go.kr/archives/2005610) |
| 기체 신고 | 최대이륙중량 2 kg 초과 기체(또는 사업용)는 신고 의무 | [KOTSA 기체신고](https://main.kotsa.or.kr/portal/contents.do?menuCode=02020100), [korea.kr](https://www.korea.kr/news/policyNewsView.do?newsId=148869304) |
| 자작 기체의 "최대이륙중량" 판정 | 비공식 해설은 실제 장착 구성의 비행 중량으로 판단하라고 합니다. **공식 해석은 미확인**입니다 | [비공식 해설](https://www.nepla.ai/wiki/it-정보-방송통신/로봇-드론/-일문일답-드론-구매-후-신고를-해야-하는-드론은-gy380o1e8rxk) |

> **주의:** X500 V2 공식 문서의 페이로드 표기는 "~1 kg (4S 5000 mAh, 70% 스로틀)"입니다([Holybro docs](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2)). 프레임 610 g + 배터리 + 1 kg이면 "제조사 최대 운반능력" 기준으로는 2 kg을 넘을 수 있습니다. 실험 전에 KOTSA(4종 문의 031-645-2102)에 서면으로 질의하고 실측 중량 기록을 남기기를 권합니다. 밸러스트로 중량을 2 kg에 붙이는 방안은 이 이유로도 기각합니다.

---

## 2. 필요한 추력 계산

시뮬레이션 값은 `drone_bombard_env.py`에서 가져왔습니다.
- `drone_mass` 2.07, `payload_mass` 0.1
- `thrust_to_weight_unloaded` 2.0, `tilt_clamp_deg` 35
- `wind_drag_k` 0.06 N/(m/s)²

### 2.1 시뮬레이션 기체가 실제로 가진 값
- 총추력 $T_{max}=2.0\times2.07\times9.81=40.6\,\mathrm{N}$ (4.14 kgf, 모터당 1.04 kgf)
- 페이로드를 실은 상태의 T/W는 $4.14/2.17=$ **1.91**입니다.

### 2.2 정상 비행에 필요한 기울기와 T/W

$$\tan\theta = \frac{k_d v_{rel}^2}{mg},\qquad T/W=\frac{1}{\cos\theta}$$

| 상대 기류 $v_{rel}$ | 의미 | 항력 | 기울기 θ (m=2.17 / 1.4 kg) | 유지 T/W |
|---|---|---|---|---|
| 3.5 m/s | 평균 투하 속도, 무풍 | 0.73 N | 2.0° / 3.0° | 1.00 |
| 6 m/s | 최대 순항, 무풍 | 2.16 N | 5.8° / 9.0° | 1.01 |
| 9 m/s | 순항 6 + 역풍 2σ | 4.86 N | 12.9° / 19.5° | 1.03~1.06 |
| 13.5 m/s | 순항 6 + 바람 상한 7.5 | 10.9 N | 27.2° / 38.5°(**35° 초과**) | 1.12~1.28 |

$k_d$=0.06은 시뮬레이션 기체 값입니다. 1.4 kg 열은 같은 $k_d$를 가정한 보수적 계산입니다. X500의 실제 $k_d$는 **미확인**이므로 비행 로그로 동정합니다. 가벼운 기체는 같은 바람에서 더 많이 기울어집니다. 이것이 GRU 재적합이 필요한 직접적 이유입니다(교차 검토 3번).

### 2.3 기울기 상한 35°에서의 수직 여유

| 수직 가속 여유 $a_z$ | 필요 T/W $=(1+a_z/g)/\cos35^\circ$ |
|---|---|
| 0 | 1.22 |
| 0.3 g | 1.59 |
| 0.5 g | 1.83 |

### 2.4 결론
- **최소 T/W는 약 1.6, 권장은 2.0 이상입니다.** X500 V2는 약 3.5 이상으로 추정되어 충분합니다.
- **T/W를 시뮬레이션에 맞추려고 실기 추력을 깎지 않습니다.** 반대로 Isaac의 `thrust_to_weight_unloaded`를 실측값으로 바꿉니다.
- PX4 offboard 경로에서는 가속 제한이 기울기와 추력으로만 걸립니다(4의 §1.2). T/W가 높으면 실기 응답이 시뮬레이션보다 공격적일 수 있습니다. 그래서 스텝 응답 동정(P0)이 필수입니다.
- 실외 운용 상한은 **평균풍 5 m/s 이하**를 권합니다. 1.4 kg 기체는 순항 6 m/s에 바람 7.5 m/s가 더해지면 35°를 넘기 때문입니다. 이 상한은 계산에 근거한 권고이며 실험 근거는 없습니다.

---

## 3. 기체 후보 비교

| 기체 | 무게 | 추력 / T/W | 페이로드 여유 | FC / 스택 | 가격 | 판정 |
|---|---|---|---|---|---|---|
| **Holybro X500 V2 PX4 Dev Kit** | 프레임 610 g, 휠베이스 500 mm ([Holybro docs](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2)) | 2216 KV920 + 1045 프롭. Holybro 공식 추력표는 **미확인**. 동급 T-Motor AIR2216II-KV920+T1045@4S는 모터당 1332 g ([T-Motor](https://store.tmotor.com/goods.php?id=1220)) → 총 약 5.3 kgf, 1.4 kg 기준 T/W 약 3.8(추정) | ~1 kg(5000 mAh, 70%) | **Pixhawk 6C, PM02 V3, M8N GPS, SiK 텔레메트리, 20A ESC 포함**(같은 출처). PX4·Gazebo `x500` 모델과 같은 계열이고, 예전 SITL airframe `4015_gz_x500_bombard`를 재사용할 수 있습니다 | $507.99 ([Holybro Store](https://shop.holybro.com/x500-kit_p1180.html)) | **1안** |
| Holybro S500 V2 | 프레임 782 g ([mybotshop](https://www.mybotshop.de/Holybro-S500-V2-Kit-ARF_3)) | X500과 같은 모터·프롭 | 1.5 kg(배터리 제외) | Pixhawk 계열 | 미확인 | X500보다 170 g 무거워 비권장 |
| ModalAI Starling 2 Max | 566 g ([ModalAI](https://docs.modalai.com/starling-2-max-datasheet/)) | 미확인 | 500 g | VOXL 2(PX4 + ROS 2 내장) ([PX4](https://docs.px4.io/main/en/flight_controller/modalai_voxl_2.html)) | 미확인 | 대안 A2: 4종 적합성이 확실하고 실내 VIO가 장점. 질량이 시뮬레이션의 약 1/4이라 재학습 부담이 큽니다 |
| Holybro QAV250 | 456 g ([Holybro](https://holybro.com/products/qav250-kit)) | 미확인 | 미확인 | Pixhawk 6C mini | 미확인 | 탑재 공간 부족, 비권장 |
| Crazyflie, 소형 Betaflight 기체 | 0.03~0.87 kg | T/W 4~6.8 | 0.1 kg 불가 또는 빠듯함 | CTBR 전제 | – | 행동 공간(속도 명령)이 맞지 않아 비권장(4의 §1.5) |

**선행연구 기체와의 정합(2에서 인용):**
- PX4 계열 기체는 다음과 같습니다. 모두 X500 V2와 같은 급입니다.
  - AeroThrow: 1.59 kg, NxtPX4v2, Orin NX
  - RESC: 1.64 kg, PX4 rate
  - DCE: 1.2 kg, 속도 + 요레이트 10 Hz
  - NN-PX4: 1.2 kg
  - Narrow Gaps: 1.1 kg, T/W 3.5
- Neural-Fly는 2.6 kg, T/W 2.2, Pixhawk 4와 RPi 4 조합으로, 시뮬레이션 기체와 가장 가깝습니다. 다만 4종 범위 밖입니다.

### X500 V2 이륙중량 예산 (추정, 반드시 실측)

| 부품 | 질량 | 근거 |
|---|---|---|
| 프레임·모터·ESC·프롭 | 610 g | 공식 |
| Pixhawk 6C, PM02, 텔레메트리, 배선 | 약 100 g | 추정 |
| GPS를 H-RTK F9P로 교체 | 58 g | [Holybro](https://holybro.com/products/dronecan-h-rtk-f9p-rover) |
| TFmini-S 하방 거리센서 | 5 g | [데이터시트](https://cdn.sparkfun.com/assets/1/3/a/3/c/TFmini-S_LiDAR_Module_Datasheet.pdf) |
| 4S LiPo 3000~5000 mAh | 약 300~500 g | 추정, 미확인 |
| RPi 5와 마운트·전원 | 약 100 g | 추정 |
| 서보 래치 | 약 30 g | 추정 |
| 페이로드 | 100 g | 설계값 |
| **합계** | **약 1.3~1.5 kg** | 2 kg까지 0.5 kg 이상 여유 |

이 기체의 페이로드 비율은 0.1/1.4 ≈ 7%로 시뮬레이션의 4.6%보다 큽니다. 투하 순간 상향 가속 계단은 약 $0.1g/1.3 \approx 0.75\,\mathrm{m/s^2}$(추정)입니다. ulog로 릴리즈 시각을 교차검증하기에는 오히려 유리합니다(4의 §5-2). 같은 질량 변화를 Isaac에도 반영해야 합니다.

---

## 4. 컴패니언 컴퓨터

### 4.1 연산 요구
- 정책은 L0 MLP(26→256→256→7, 약 74 k MAC)와 GRU(26→64→2, 약 17 k MAC)입니다. 합계 약 0.1 M MAC/step, 10 Hz면 약 1 M MAC/s입니다.
- 배포 스택((4) 기준)은 단일 rclpy 노드입니다. 10 Hz 정책, 20 Hz setpoint와 LPF, **100 Hz 투하 게이트**를 돌리고, MicroXRCEAgent와 rosbag2 기록을 함께 씁니다. 추론은 numpy로 하므로 torch가 필요 없습니다.
- 따라서 병목은 추론이 아니라 **100 Hz 타이머의 지터와 odometry 수신 지연**입니다. 이 값은 RPi 위에서 실측해야 합니다(미확인).

### 4.2 보드별 비교

| 보드 | 정책만 (트랙 A) | 정책 + YOLOv8n(640) | ROS 2 Humble / uXRCE-DDS | 무게 / 전력 | 판정 |
|---|---|---|---|---|---|
| **Raspberry Pi 5** | 여유 | CPU NCNN 85 ms(약 12 FPS, 추론만, [커뮤니티 보고](https://huggingface.co/akswelh/Test_model1/blob/main/docs/en/guides/raspberry-pi.md)). 10 Hz 경계선 | Ubuntu 22.04 + Humble 네이티브, PX4 공식 가이드(TELEM2, `UXRCE_DDS_CFG=102`, 921600) ([PX4](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi)) | 미확인 / 최대 8.8 W ([raspberry.tips](https://raspberry.tips/en/?p=8794)) | **트랙 A 1안** |
| Raspberry Pi 4 | 여유 | 미확인(RPi 5보다 느림) | 위와 같음. Neural-Fly 탑재 선례 ([Caltech](https://www.caltech.edu/about/news/rapid-adaptation-of-deep-learning-teaches-drones-to-survive-any-weather)) | 미확인 | 이미 보유하고 있다면 대체 가능 |
| RPi 5 + AI HAT+(Hailo-8L) | 여유 | YOLOv8s 60~80 FPS(제3자 보고, v8n 미확인) ([raspberry.tips](https://raspberry.tips/en/raspberrypi-tutorials/raspberry-pi-ai-hat-plus-ai-acceleration)). Hailo 형식으로 다시 컴파일해야 함 | 위와 같음 | HAT 평균 약 6 W ([Sixfab](https://sixfab.com/?p=202764)) | 트랙 B 통합 시 저전력 대안 |
| **Jetson Orin Nano(Super)** | 여유 | TRT FP16 100 FPS ([Qengineering](https://github.com/Qengineering/YoloV8-TensorRT-Jetson_Nano)) / 120 FPS ([jetson-ai-lab](https://jetson-ai-lab.com/tutorial_ultralytics.html)) | JetPack 6(22.04), Humble 네이티브 | 모듈 약 31 g + 캐리어(미확인) / 7~25 W ([NVIDIA](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules)) | 트랙 B 통합 시 1순위. AeroThrow·DCE의 Orin NX와 같은 계열 |
| Jetson Nano(구형) | 여유 | TRT FP16 19 FPS ([Qengineering](https://github.com/Qengineering/YoloV8-TensorRT-Jetson_Nano)) | Ubuntu 18.04라 Humble은 Docker로만 사용 ([NVIDIA 포럼](https://forums.developer.nvidia.com/t/ros2-humble-desktop-for-jetson-nano/236942)) | 미확인 | **비권장**(단종, 구형 OS) |

### 4.3 결론
- **트랙 A는 RPi 5로 충분합니다.** 연결은 TELEM2 직결 시리얼 921600입니다. uXRCE-DDS에는 인증이 없으므로 무선 DDS를 쓰지 않습니다(4의 §2.5).
- **트랙 B를 통합할 때는 Orin Nano(Super)를 씁니다.** 무게 예산을 다시 계산해야 하지만, 2 kg까지 여유 0.5 kg 안에 들어갈 것으로 예상합니다(캐리어 무게 미확인).
- 시험 항목: RPi 5에서 100 Hz 게이트 타이머의 지터, `vehicle_odometry` 수신률, `timestamp_sample`과 수신 시각의 차이를 실측합니다. 측정값은 (4) 방안 B의 지연 DR에 넣습니다.

---

## 5. 추가로 필요한 장비

| 항목 | 권장안 | 근거·주의 | 가격대 |
|---|---|---|---|
| **투하 기구** | **PWM 서보 래치(핀 빼기 또는 후크) → Pixhawk AUX 출력. 출력 함수 "Peripheral via Actuator Set 1", `VEHICLE_CMD_DO_SET_ACTUATOR`(187)로 구동** ([PX4 Generic Actuator Control](https://docs.px4.io/main/en/payloads/generic_actuator_control.html)). failsafe·disarmed PWM은 "닫힘". 대안: `DO_GRIPPER`(커스텀 빌드 필요 ([PX4 Gripper](https://docs.px4.io/main/en/peripherals/gripper.html))), 전자영구자석(OpenGrab EPM v3 97 g, 단종 ([ArduPilot](https://ardupilot.org/copter/docs/common-electro-permanent-magnet-V3.html))) | **지연의 표준편차가 오차원입니다.** 평균은 예측기가 보상합니다. 시뮬레이션은 0.22 ± 0.05 s를 가정하고, 선행은 0.11 s(Learning to Throw)와 100 ms(Munn)입니다. 서보 후보 2~3종을 벤치에서 20~30회씩 측정합니다. Actuator Set 2에 LED를 동시에 켜고 240 fps 이상으로 촬영합니다. **표준편차가 작은 것**을 채택하고 평균과 표준편차를 Isaac에 반영합니다 | 서보는 소액(미확인) |
| **페이로드** | 시뮬레이션과 같은 원통(r 0.05 m, h 0.06 m, 0.1 kg). 3D 프린트 외피에 쇠구슬이나 모래를 채움. **10개 이상, 질량 ±1 g** | 낙하 시험으로 탄도계수 $k/m$을 동정해 공칭 예측기에 넣습니다(4의 §6). 선명한 색, 바운스 억제 | 소액 |
| **하방 거리센서** | **Benewake TFmini-S**(5 g, 0.1~12 m @90% 반사율, 0.1~7 m @10%, ±6 cm @≤6 m, 100 Hz, 70 klux) ([데이터시트](https://cdn.sparkfun.com/assets/1/3/a/3/c/TFmini-S_LiDAR_Module_Datasheet.pdf)) | 투하 창 3~8 m를 덮습니다. 순항 8~12 m는 지면 반사율에 따라 일부만 덮습니다. 1차 고도 출처는 RTK이고, 거리센서는 기록과 교차검증용입니다. EKF 융합은 선택입니다. TF-Luna(5 g)는 최대 8 m라 여유가 부족합니다 ([ArduPilot](https://ardupilot.org/copter/docs/common-benewake-tf02-lidar.html)). 투하 경로 지면은 평탄해야 합니다 | 미확인 |
| **측위(실외)** | H-RTK F9P 로버(58 g, 0.01 m + 1 ppm) + 베이스 또는 NTRIP ([Holybro](https://holybro.com/products/dronecan-h-rtk-f9p-rover)). 키트의 M8N을 교체 | RTK 수직 정밀도와 국내 NTRIP 조건은 미확인. 표적 좌표 측량에도 같은 장비를 씁니다 | Rover $296.99 |
| **측위(실내, P0 단계)** | 모션캡처를 EKF2 외부 비전으로 융합 ([PX4](https://docs.px4.io/main/zh/ros/external_position_estimation.html)) | 선행 투척 실기는 전부 모캡이었습니다(2의 C-6). 우리는 동정용으로만 씁니다 | 시설 대여 |
| **착탄점 계측** | (1) 표적 주변 격자 매트 또는 판지와 지상 하방 고정 카메라 + AprilTag 호모그래피, (2) RTK 측량봉 | 측정 오차 목표는 **2~3 cm 이하**(CEP50 0.204 m의 약 1/10, 경험칙)입니다. 선행: AeroThrow는 판지 흔적, Ma/Hutter는 AprilTag. **첫 접지점** 기준과 바운스 처리 규칙을 미리 정합니다 | 소~중 |
| **풍속계** | 2D 초음파식(예: Gill WindSonic, ±2%@12 m/s, 0.25~4 Hz) ([Campbell](https://s.campbellsci.com/documents/us/product-brochures/b_windsonic4-qd.pdf))를 투하 고도(3~8 m) 근처 마스트에 설치하고 시각을 동기화 | 정책 입력이 아니라 **조건 층화와 sim 재생용**입니다(2의 D, P4). 투하 조건별 바람 실측은 선행에 없어 우리 기여점입니다. 돌풍 τ 3~10 s에 4 Hz면 충분합니다 | 미확인 |
| **질량·관성 동정** | 저울(±1 g), 관성은 CAD 또는 이중 진자(bifilar) 측정 | NN-PX4는 저울과 CAD, SimpleFlight는 SysID를 먼저 했습니다. 정확히 잴 수 있는 값은 DR하지 말고 측정값을 씁니다(SimpleFlight) | 소액 |
| **추력곡선** | 호버 로그(`MPC_THR_HOVER`, HTE)로 추력계수를 동정. 추력 스탠드는 선택 | NN-PX4는 호버에서 동정했습니다 | 스탠드 가격 미확인 |
| **배터리** | 4S LiPo 3000~5000 mAh, **6개 이상(추정)** | 투하 80~100회 기준. 전압 강하가 추력을 바꾸므로 시작 전압 상한을 고정하고 기록합니다(sim2real_gap §3.3). 투하 1회당 소요 시간에 따른 정확한 수량은 미확인 | 미확인 |
| **안전** | RC 킬 스위치, RC 투하 허가 스위치(인터록), PX4 지오펜스, `COM_OF_LOSS_T`, 초기 테더 시험 | (4)의 §2.5와 같습니다. 비행 승인·장소는 드론원스톱에서 확인합니다 | – |

### PX4 설정 체크리스트 ((4)와 합의)

| 파라미터 | 값 | 이유 |
|---|---|---|
| `MPC_TILTMAX_AIR` | 35 | 시뮬레이션 기울기 상한과 맞춥니다(기본 45) |
| `MPC_Z_VEL_MAX_DN` | 3.0 | 기본 1.5 m/s가 정책의 vz 명령을 자릅니다 |
| `MPC_XY_VEL_I_ACC` / `MPC_Z_VEL_I_ACC` | 첫 비행은 0 / 0.2(방안 A), 본선은 기본값 + sim 재학습(방안 B) | 4의 §1.4 |
| `MPC_THR_MAX` | 기본 0.9 유지(안전 상한) | T/W 정합에는 쓰지 않습니다 |
| `UXRCE_DDS_CFG` | 102(TELEM2), 921600 | PX4 RPi 가이드 |
| 서보 출력 | Peripheral via Actuator Set 1, disarmed·failsafe = 닫힘 | §5 투하 기구 |

---

## 6. 실험 장소 (역할 분담으로 수정)

| 단계 | 장소 | 측위 | 목적 |
|---|---|---|---|
| P0 동정 | 실내(모캡 홀) 또는 무풍 실외 | 모캡 약 1 cm / RTK | 릴리즈 지연(벤치), 호버 추력, PX4 속도 루프 7점 스텝 응답, 관측 부호 시험, 섀도 비행 |
| P3 무풍·저풍 기준 | **실외 원 스케일** | RTK + TFmini-S | 평균풍 2 m/s 미만에서 팔(L0, L0+GRU-S+EMA, T2, T0) 비교, 10회 이상 |
| P4 바람 조건 | **실외 원 스케일** | 같음 + 풍속계 마스트 | 자연풍(평균풍 5 m/s 이하)을 사후 층화하고 ABAB 교대 비행 |

일반 실내 홀에는 원 시나리오(순항 8~12 m, 접근 18~22 m)가 들어가지 않습니다. 실내는 동정용으로만 쓰므로 **스케일 축소는 필요 없다는 쪽으로 기웁니다.** 최종 결정은 (3)에서 합니다.

---

## 권장 구성 1안 (트랙 A, 실외 원 스케일)

| 구분 | 구성 |
|---|---|
| 기체 | **Holybro X500 V2 PX4 Dev Kit**(Pixhawk 6C, PX4 v1.15.x). 이륙중량 1.3~1.5 kg(추정), T/W 약 3.5 이상(추정) |
| 컴패니언 | **Raspberry Pi 5**, Ubuntu 22.04 + Humble, MicroXRCEAgent, TELEM2 직결, numpy 추론 |
| 측위·고도 | H-RTK F9P(+베이스 또는 NTRIP) + TFmini-S |
| 투하 | PWM 서보 래치, DO_SET_ACTUATOR, RC 투하 허가 인터록 |
| 페이로드 | 0.1 kg 원통 10개 이상 |
| 계측 | 초음파 풍속계 마스트, 착탄 카메라 + AprilTag 격자(또는 RTK 측량), 240 fps 이상 카메라(릴리즈 지연) |
| 사전 조치 | KOTSA 서면 질의, 실측 중량 기록, 질량·관성·추력·스텝 응답·릴리즈 지연 동정 → Isaac 반영 → L0 미세조정 + GRU 재적합 |

### 대안

| 대안 | 언제 쓰나 |
|---|---|
| A2: ModalAI Starling 2 Max(566 g) | 법적 적합성을 확실히 해야 할 때, 또는 실내 VIO 실험. 재학습 부담이 큽니다 |
| A3: X500 V2 + Orin Nano(Super) | 트랙 B의 YOLO를 기체 위에서 함께 돌릴 때 |
| A4: X500 V2 + RPi 5 + Hailo AI HAT+ | YOLO를 저전력으로 탑재하는 절충안 |
| 비권장 | Jetson Nano(구형), QAV250, Crazyflie·Betaflight 소형기, 밸러스트로 2.07 kg에 맞추는 방안 |

### (3)·(5)로 넘기는 사항
1. 실기는 2.07 kg/T/W 2.0 기체의 복제가 아닙니다. **실측 파라미터로 Isaac을 재설정하고 L0 미세조정과 GRU 재적합**을 합니다. 밸러스트 정합은 기각합니다.
2. 1.4 kg 기체는 같은 바람에서 기울기가 1.4~1.5배 큽니다. 따라서 **실외 평균풍 상한은 5 m/s**로 두고, 시뮬레이션 바람 분포도 실기 운용 범위에 맞춰 재검토합니다.
3. 원 스케일 실외 실험이 가능합니다. 실내는 동정용입니다.
4. 릴리즈 지연의 평균과 표준편차, 탄도계수, 지연 예산은 벤치에서 측정한 뒤 확정합니다.

**확인하지 못한 항목:**
- Holybro 2216 KV920의 공식 추력표와 X500의 실제 $k_d$
- RPi 4·VOXL 2의 YOLO 수치, Jetson Nano 무게·전력, Orin Nano 캐리어 무게
- Starling 2 Max 가격, RTK 수직 정밀도, 국내 NTRIP 조건
- 최대이륙중량의 공식 해석
- 배터리당 투하 횟수, RPi 5에서의 100 Hz 타이머 지터

**참고한 내부 파일:**
- /opt/drone-bombard/Drone-Bombard-Simulation/isaac_lab/drone_bombard/drone_bombard_env.py
- /opt/drone-bombard/Drone-Bombard-Simulation/notes/research/sim2real_gap.md
- /opt/drone-bombard/Drone-Bombard-Simulation/notes/research/residual_observability.md((4) 인용)

**추가 출처(교차 검토 단계):**
- [Holybro X500 V2 docs](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2)
- [PX4 Controller Diagrams (MPC_THR_MAX)](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)
- [TFmini-S datasheet](https://cdn.sparkfun.com/assets/1/3/a/3/c/TFmini-S_LiDAR_Module_Datasheet.pdf)
- [ArduPilot Benewake 문서](https://ardupilot.org/copter/docs/common-benewake-tf02-lidar.html)
- [PX4 Generic Actuator Control](https://docs.px4.io/main/en/payloads/generic_actuator_control.html)
- [PX4 Gripper](https://docs.px4.io/main/en/peripherals/gripper.html)
- [Caltech Neural-Fly 보도](https://www.caltech.edu/about/news/rapid-adaptation-of-deep-learning-teaches-drones-to-survive-any-weather)