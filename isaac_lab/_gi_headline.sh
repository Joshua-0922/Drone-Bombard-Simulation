#!/bin/bash
# Promote the gain-invariant regressor to the headline arm. Table 1 and figure 4
# currently report the OLD tilt regressor, which gi beats on every metric
# (CEP90 0.464 -> 0.411, succ@0.5 86.50 -> 89.67%), so the paper's method and
# its numbers disagree. DR 1.5 already exists in /tmp/sl_GI; this fills the rest.
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
cd /tmp/rebuild
SEEDS="3000 4000 5000"
run() { local f=$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
GI="--sl_residual /tmp/sl/res_gi.pt --sl_ema 0.3 --arm_name L1SL_gi_ema"
B="--policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200"

# Table 1, DR 2.5 column
for S in $SEEDS; do
  run /tmp/sl_GI/SLgiE_dr2.5_s$S $IL -p play.py $B --marker_dist 18 22 --dr_scale 2.5 --seed $S $GI
done
# Figure 4, the three missing DR points
for D in 0 0.5 1.0; do
  for S in $SEEDS; do
    run /tmp/dr_axis/SLgiE_dr${D}_s$S $IL -p play.py $B --marker_dist 18 22 --dr_scale $D --seed $S $GI
  done
done
# Unseen range row
for S in $SEEDS; do
  run /tmp/sl_R/SLgiE_dr1.5_s$S $IL -p play.py $B --marker_dist 26 30 --dr_scale 1.5 --seed $S $GI
done
echo "GI HEADLINE DONE $(date +%T)"
