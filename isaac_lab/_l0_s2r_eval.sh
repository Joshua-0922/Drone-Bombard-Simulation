#!/bin/bash
# L0 (PX4 plant + measured sim2real constants, exp_046) paired evals:
# DR sweep {0, 0.5, 1.0, 1.5} at 18-22 m + unseen range 26-30 m at the training scale 1.0,
# eval seeds {3000, 4000, 5000} x 200, for every checkpoint given as  tag=path  arguments.
# usage: bash _l0_s2r_eval.sh s1=/tmp/l0_s2r/logs/.../model_final.pt [s2=...]
set -u
IL=/workspace/isaaclab/isaaclab.sh
OUT=${OUT:-/tmp/l0_s2r/eval}
mkdir -p $OUT
cd /tmp/rebuild
run() { local f=$1.json; shift; [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for arg in "$@"; do
  T=${arg%%=*}; CK=${arg#*=}
  B="--policy $CK --no_residual --paired_eval --headless --episodes 200 --num_envs 200 --arm_name L0_$T"
  for D in 0 0.5 1.0 1.5; do for S in 3000 4000 5000; do
    run $OUT/L0_${T}_dr${D}_s$S $IL -p play.py $B --marker_dist 18 22 --dr_scale $D --seed $S
  done; done
  for S in 3000 4000 5000; do
    run $OUT/L0_${T}_R26-30_dr1.0_s$S $IL -p play.py $B --marker_dist 26 30 --dr_scale 1.0 --seed $S
  done
done
echo "L0 S2R EVAL DONE $(date +%T)"
