#!/bin/bash
# EMA alpha sensitivity for the adopted method (L0 + GRU-S), steady wind DR 1.5, seeds {3000,4000,5000}.
# alpha 0.3 reference already at /tmp/gru/GRUS_E_tau0_s*.json. Output /tmp/ema/GRUS_ema{A}_s{S}.json
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
OUT=/tmp/ema; mkdir -p $OUT; cd /tmp/rebuild
run() { local f=$OUT/$1.json; shift; [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for A in ${ALPHAS:-0.1 0.15 0.2 0.4 0.5 0.7 1.0}; do for S in 3000 4000 5000; do
  run "GRUS_ema${A}_s${S}" $IL -p play.py --policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 \
      --marker_dist 18 22 --dr_scale 1.5 --seed $S --sl_residual /tmp/sl/res_gru_S.pt --sl_ema $A --arm_name GRUS_ema$A
done; done
echo "EMA SWEEP DONE $(date +%T)"
