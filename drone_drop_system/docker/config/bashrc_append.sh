# ROS 2 환경 설정 (워크스페이스 우선)
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
    source /workspace/ros2_ws/install/setup.bash
else
    source /opt/ros/humble/setup.bash
fi


alias setup= "source /workspace/ros2_ws/install/setup.bash"

alias roshumble="source /opt/ros/humble/setup.bash"

# 데몬 자동 리셋 (혹시 모를 캐시 문제 방지)
alias ros_reset="ros2 daemon stop && ros2 daemon start"

# 터미널 프롬프트 예쁘게 (선택사항)
export PS1='\[\033[01;32m\](ROS2 Docker)\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\]\$ '