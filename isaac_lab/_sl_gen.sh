#!/bin/bash
# L1-SL generalization batch -- the three axes the main sweep left untested.
#
#   R : unseen target range 26-30 m (trained/fitted on 18-22)  -> is it wind, or memorized geometry?
#   P : the seed-1 regressor injected into the seed-2 policy   -> is the map physics, or policy-specific?
#   N : regressors fitted on 100..2000 episodes                -> is the oracle gap data, or information?
#
# All arms paired, 3 eval seeds x 200 episodes, DR 1.5, EMA 0.3 on every
# injected residual (raw injection is broken by first-crossing bias).
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
CK2=/tmp/l0_s2/logs/drone_bombard_ppo/2026-08-30_09-57-12_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
cd /tmp/rebuild
SEEDS="3000 4000 5000"

run() { local f=$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }

# --- R: unseen range -------------------------------------------------------
mkdir -p /tmp/sl_R
BR="--policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 --marker_dist 26 30 --dr_scale 1.5"
for S in $SEEDS; do
  run /tmp/sl_R/L0_dr1.5_s$S       $IL -p play.py $BR --no_residual --seed $S --arm_name L0
  run /tmp/sl_R/SLtiltE_dr1.5_s$S  $IL -p play.py $BR --sl_residual /tmp/sl/res_tilt.pt --sl_ema 0.3 --seed $S --arm_name L1SL_tilt_ema
  run /tmp/sl_R/ORCwind_dr1.5_s$S  $IL -p play.py $BR --oracle_residual_wind_only --seed $S --arm_name oracle_wind_only
done

# --- P: policy transfer (regressor fitted on the seed-1 policy) -------------
mkdir -p /tmp/sl_P
BP="--policy $CK2 --paired_eval --headless --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale 1.5"
for S in $SEEDS; do
  run /tmp/sl_P/L0_dr1.5_s$S       $IL -p play.py $BP --no_residual --seed $S --arm_name L0_s2
  run /tmp/sl_P/SLtiltE_dr1.5_s$S  $IL -p play.py $BP --sl_residual /tmp/sl/res_tilt.pt --sl_ema 0.3 --seed $S --arm_name L1SL_tilt_ema_xfer
  run /tmp/sl_P/ORCwind_dr1.5_s$S  $IL -p play.py $BP --oracle_residual_wind_only --seed $S --arm_name oracle_wind_only
done

# --- N: label-count curve --------------------------------------------------
# L0 and the oracle ceiling are config-identical to the main sweep, so reuse
# those episodes instead of re-flying them; N=2334 is the main sweep's SLtiltE.
mkdir -p /tmp/sl_N
cp -n /tmp/sleval/L0_dr1.5_s*.json /tmp/sleval/ORCwind_dr1.5_s*.json /tmp/sl_N/ 2>/dev/null
for f in /tmp/sleval/SLtiltE_dr1.5_s*.json; do cp -n "$f" "/tmp/sl_N/n2334_dr1.5_s${f##*_s}"; done
BN="--policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale 1.5"
for N in 100 200 500 1000 2000; do
  for S in $SEEDS; do
    run /tmp/sl_N/n$(printf %04d $N)_dr1.5_s$S $IL -p play.py $BN \
        --sl_residual /tmp/sl/res_tilt_n$N.pt --sl_ema 0.3 --seed $S --arm_name L1SL_tilt_ema_n$N
  done
done
echo "SL GEN DONE $(date +%T)" > /tmp/sl_N/gen.done
