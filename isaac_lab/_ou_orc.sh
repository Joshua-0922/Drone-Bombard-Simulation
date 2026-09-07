#!/bin/bash
# The oracle arm is injected RAW everywhere else, which is right under constant
# wind (its output is already smooth) but not under OU: at tau=10 the raw oracle
# LOST to no-residual (CEP50 0.315 vs 0.272, CEP90 1.233 vs 0.578). A ceiling
# that loses is not a ceiling. Re-run it with the same EMA the learned arms get,
# so the comparison isolates information rather than smoothness.
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
cd /tmp/rebuild
run() { local f=/tmp/ou/$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for T in 10 3 1 0.3; do
  for S in 3000 4000 5000; do
    run "ORCwindE_tau${T}_s${S}" $IL -p play.py --policy $CK1 --paired_eval --headless \
        --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale 1.5 --seed $S \
        --wind_tau $T --oracle_residual_wind_only --oracle_ema 0.3 --arm_name oracle_wind_ema
  done
done
# Control: the constant-wind oracle with EMA, to show the smoothing itself is
# not what changes the ceiling (it should land on the published 0.192).
for S in 3000 4000 5000; do
  run "ORCwindE_tau0_s${S}" $IL -p play.py --policy $CK1 --paired_eval --headless \
      --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale 1.5 --seed $S \
      --oracle_residual_wind_only --oracle_ema 0.3 --arm_name oracle_wind_ema
done
echo "OU ORACLE DONE $(date +%T)"
