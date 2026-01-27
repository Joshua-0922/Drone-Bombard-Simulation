1️⃣ mission_manager (Commander)
시스템의 지휘관 역할을 하는 최상위 제어 노드입니다. FSM(Finite State Machine)을 기반으로 미션 단계를 관리합니다.

역할:

CRUISE: 정해진 경로(웨이포인트)로 순항 명령 생성 (/drone/cmd/position).

INTERCEPT: Vision 노드에서 표적 감지 신호 수신 시, 순항을 중단하고 추적 모드로 전환.

TRACK: RL Navigator(또는 유도 알고리즘)의 속도 명령을 드론에게 중계 (/drone/cmd/velocity).

DROP: Drop Calculator의 신호를 받아 투하 장치 작동.

Input: /vision/target_pixel, /payload/drop_cmd, /fmu/out/vehicle_local_position

Output: /drone/cmd/position, /drone/cmd/velocity, /mission/state

2️⃣ drone_controller (Pilot)
PX4 Autopilot과 직접 통신하며 드론의 비행 제어를 담당하는 노드입니다.

역할:

Protocol Bridge: ROS 2(ENU 좌표계) 명령을 PX4(NED 좌표계)로 변환.

Mode Handling: Offboard 모드 진입, Arming(시동), Failsafe 처리.

Control Interface: 위치 제어(Position)와 속도 제어(Velocity) 모드를 상황에 따라 스위칭.

Input: /drone/cmd/position, /drone/cmd/velocity

Output: /fmu/in/trajectory_setpoint, /fmu/in/offboard_control_mode, /fmu/in/vehicle_command

3️⃣ vision_detection (Eyes)
하방 카메라 영상을 분석하여 목표물('X' 마커)을 실시간으로 탐지합니다.

역할:

YOLOv8 Inference: 사전 학습된 모델을 사용하여 객체 탐지.

Pixel Error Calculation: 이미지 중심과 타겟 중심 간의 차이(u, v error) 계산 (유도 비행의 핵심 데이터).

Estimation: 기압계/GPS 고도 정보를 결합하여 타겟의 상대적 3D 위치 추정.

Input: /camera/image_raw, /fmu/out/vehicle_local_position

Output: /vision/target_pixel, /vision/bbox, /vision/annotated_image

4️⃣ rl_navigator (Guidance System)
표적 추적 시 드론의 움직임을 결정하는 유도 조종 알고리즘입니다. (Visual Servoing / RL)

역할:

Bank-to-Turn: 타겟이 화면 중심에 오도록 드론의 Roll(기울기)과 Pitch(전진) 속도를 계산.

Proportional Navigation: 움직이는 타겟이나 바람의 영향을 고려하여 최적의 요격 경로 생성.

Input: /vision/target_pixel (Pixel Error)

Output: /drone/cmd/velocity (Twist msg)

5️⃣ drop_calculator (Ballistics)
물리학 기반의 정밀 투하 타이밍 계산기입니다.

역할:

드론의 **현재 속도(Ground Speed)**와 **고도(Altitude)**를 실시간 모니터링.

낙하 궤적을 예측하여 타겟이 '투하 가능 영역(Drop Zone)'에 들어왔을 때 트리거 신호 전송.

Input: /fmu/out/vehicle_odometry, /vision/target_position

Output: /payload/drop_cmd