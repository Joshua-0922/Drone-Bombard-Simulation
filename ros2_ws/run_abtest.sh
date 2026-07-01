#!/bin/bash
# run_abtest.sh <cond> <config_rel_path>
# Runs a 2-episode eval of the v14 195K model while logging the velocity
# setpoint PX4 receives (wobble metric) and recording onboard video.
# NOTE: no `set -u` — ROS setup.bash references unbound vars and would abort.
COND="$1"
CFG="$2"
DUR=110

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds
cd /workspace/ros2_ws

bash start_infra_clean.sh
rm -f rl_eval_results/flight_raw.mp4 rl_eval_results/flight_annotated.mp4

ros2 run rl_navigation evaluate \
  --model rl_checkpoints/sac_drop_195000_steps.zip \
  --episodes 2 --config "$CFG" \
  --output-dir "rl_abtest_${COND}" > "/tmp/abtest_${COND}.log" 2>&1 &
EVAL_PID=$!

# wait (max 90s) for infra
for i in $(seq 1 90); do
  grep -q 'PX4 ready' "/tmp/abtest_${COND}.log" 2>/dev/null && break
  sleep 1
done
echo "PX4 ready -> loggers ($COND)"
python3 vel_logger.py "$DUR" "/tmp/vel_${COND}.csv" > "/tmp/vel_${COND}.out" 2>&1 &
VEL_PID=$!
python3 record_flight.py "$DUR" > "/tmp/rec_${COND}.out" 2>&1 &
REC_PID=$!

wait "$EVAL_PID"
echo "eval done ($COND) -> stopping loggers"
kill -INT "$VEL_PID" "$REC_PID" 2>/dev/null
sleep 3
mv -f rl_eval_results/flight_raw.mp4 "rl_eval_results/abtest_${COND}_flight_raw.mp4" 2>/dev/null
mv -f rl_eval_results/flight_annotated.mp4 "rl_eval_results/abtest_${COND}_flight_annotated.mp4" 2>/dev/null
echo "ABTEST_${COND}_COMPLETE rows=$(wc -l < /tmp/vel_${COND}.csv 2>/dev/null)"
