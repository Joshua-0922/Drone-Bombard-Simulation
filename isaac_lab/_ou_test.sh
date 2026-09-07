#!/bin/bash
# Time-varying (OU) wind: a pure TEST, no training and no refitting.
# Everything is frozen -- the L0 policy, both regressors, the injection path --
# and only the wind process changes. tau is the correlation time in seconds;
# the stationary distribution is held at the trained one, so this isolates time
# variation rather than wind strength.
#
# Why these tau: delivery takes ~5.8 s and the payload falls for ~1 s.
#   10 s  slower than an episode -> should look like the constant case
#    3 s  comparable to an episode -> the accumulator's window
#    1 s  comparable to the FALL   -> the oracle itself must start breaking,
#                                     since it integrates the CURRENT wind as
#                                     if it held for the whole fall
#  0.3 s  gusts
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
OUT=/tmp/ou; mkdir -p $OUT
cd /tmp/rebuild
run() { local f=$OUT/$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }

for T in 10 3 1 0.3; do
  for S in 3000 4000 5000; do
    B="--policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 \
       --marker_dist 18 22 --dr_scale 1.5 --seed $S --wind_tau $T"
    run "L0_tau${T}_s${S}"      $IL -p play.py $B --no_residual --arm_name L0
    run "SLtiltE_tau${T}_s${S}" $IL -p play.py $B --sl_residual /tmp/sl/res_tilt.pt --sl_ema 0.3 --arm_name L1SL_tilt
    run "SLgiE_tau${T}_s${S}"   $IL -p play.py $B --sl_residual /tmp/sl/res_gi.pt   --sl_ema 0.3 --arm_name L1SL_gi
    run "ORCwind_tau${T}_s${S}" $IL -p play.py $B --oracle_residual_wind_only --arm_name oracle_wind_only
  done
done
echo "OU TEST DONE $(date +%T)"
