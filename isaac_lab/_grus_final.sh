#!/bin/bash
# Final-table evals for the adopted method (GRU-S): DR-scale sweep + unseen range, seeds {3000,4000,5000}.
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
cd /tmp/rebuild
GS="--sl_residual /tmp/sl/res_gru_S.pt --sl_ema 0.3 --arm_name L1SL_gruS"
B="--policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200"
run() { local f=$1.json; shift; [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for D in 0 0.5 1.0 2.5; do for S in 3000 4000 5000; do
  run /tmp/dr_axis/GRUS_dr${D}_s$S $IL -p play.py $B --marker_dist 18 22 --dr_scale $D --seed $S $GS
done; done
for S in 3000 4000 5000; do
  run /tmp/sl_R/GRUS_dr1.5_s$S $IL -p play.py $B --marker_dist 26 30 --dr_scale 1.5 --seed $S $GS
done
echo "GRUS FINAL DONE $(date +%T)"
