---
date: 2026-10-11
tags: [error, px4, offboard, drone_controller, yaw, ros2]
status: resolved
type: error
---

# drone_controller가 PX4에 yaw를 0으로 보내 기수가 북쪽에 고정되고 yaw 속도 명령이 무시되었다

관련: [[experiments/exp_043_gazebo_px4_sim2sim]] · [[00_index]]

**증상.** Isaac 정책을 Gazebo + PX4 SITL에서 돌리자 handoff 직후 정책이 좌측 +3 m/s로 포화되어 표적을 지나쳤다. 관측을 Isaac과 비교하니 yaw가 90° 달랐다. 기체가 접근 방위(동쪽) 대신 북쪽을 보고 있었다.

**원인 (둘).**
1. `publish_velocity_setpoint`가 `TrajectorySetpoint.yaw`를 설정하지 않아 메시지 기본값 0.0(북쪽)이 나갔다. PX4는 yaw가 유한값이면 그 heading을 목표로 삼고 `yawspeed`는 피드포워드로만 쓴다. 그래서 RL의 yaw 속도 명령이 사실상 무시되고 기수가 북쪽에 고정되었다. **Gazebo 시절 학습·평가의 yaw 명령도 같은 이유로 버려지고 있었다.**
2. `publish_position_setpoint`는 yaw=NaN이라 이륙 뒤 기수가 정해지지 않았다.

**수정.** 속도 모드 `msg.yaw = NaN`(yawspeed 사용). 위치 모드는 새 토픽 `/drone/cmd/yaw`(std_msgs/Float32, ENU, 동쪽에서 반시계)로 heading 지정, 미지정 시 기존처럼 NaN. `drop_mission_node`가 이륙·순항 중 접근 방위로 고정한다.

**교훈.** PX4 `TrajectorySetpoint`는 쓰지 않는 필드를 반드시 NaN으로 채운다(기본값 0은 "0으로 가라"는 명령이다). 정책 배포 전에는 관측 벡터를 학습 시뮬과 채널 단위로 비교한다.
