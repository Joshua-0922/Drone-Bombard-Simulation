#!/bin/bash
# Gate-aware stage 1: smoothness-trained GRUs injected WITHOUT EMA, seeds {3000,4000,5000},
# conditions: constant wind (tau0), realistic A (20% gust tau10), realistic B (30% gust tau3).
#   ARMS="Sm5 Sm20 Sm50" EMA=1.0 bash _gate_eval.sh
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
OUT=/tmp/gate; mkdir -p $OUT; cd /tmp/rebuild
ARMS=${ARMS:-"Sm5 Sm20 Sm50"}; EMA=${EMA:-1.0}
declare -A COND=( [tau0]="" [A]="--wind_tau 10 --wind_gust 0.2" [B]="--wind_tau 3 --wind_gust 0.3" )
run() { local f=$OUT/$1.json; shift; [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for C in tau0 A B; do for N in $ARMS; do for S in 3000 4000 5000; do
  run "GRU${N}_ema${EMA}_${C}_s${S}" $IL -p play.py --policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 \
      --marker_dist 18 22 --dr_scale 1.5 --seed $S ${COND[$C]} --sl_residual /tmp/sl/res_gru_$N.pt --sl_ema $EMA --arm_name GRU${N}_ema$EMA
done; done; done
echo "GATE EVAL DONE $(date +%T)"
