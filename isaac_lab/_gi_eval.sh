#!/bin/bash
# Gain-invariant residual, injected. The offline cross-policy R2 says the
# transfer gap is gone (tilt 0.611->0.554 across policies; tilt+gi 0.668->0.679),
# but R2 did NOT predict the exp_029 collapse (R2 fell 0.057 while the oracle-gain
# capture fell 85% -> 22.9%), so the claim only counts once it is flown.
#   sl_P  : seed-2 policy, regressor fitted on the seed-1 policy  <- THE test
#   sl_GI : seed-1 policy, same regressor                         <- in-policy control
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
CK2=/tmp/l0_s2/logs/drone_bombard_ppo/2026-08-30_09-57-12_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
cd /tmp/rebuild
SEEDS="3000 4000 5000"
B="--paired_eval --headless --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale 1.5"

run() { local f=$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }

mkdir -p /tmp/sl_GI
cp -n /tmp/sleval/L0_dr1.5_s*.json /tmp/sleval/SLtiltE_dr1.5_s*.json \
      /tmp/sleval/ORCwind_dr1.5_s*.json /tmp/sl_GI/ 2>/dev/null

for S in $SEEDS; do
  run /tmp/sl_P/SLgiE_dr1.5_s$S  $IL -p play.py --policy $CK2 $B \
      --sl_residual /tmp/sl/res_gi.pt --sl_ema 0.3 --seed $S --arm_name L1SL_gi_ema_xfer
  run /tmp/sl_GI/SLgiE_dr1.5_s$S $IL -p play.py --policy $CK1 $B \
      --sl_residual /tmp/sl/res_gi.pt --sl_ema 0.3 --seed $S --arm_name L1SL_gi_ema
done
echo "GI EVAL DONE $(date +%T)"
