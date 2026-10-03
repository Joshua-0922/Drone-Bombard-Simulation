#!/bin/bash
# No-retrain evaluation of the trained L0 and L0 + GRU-S (EMA 0.3) under the PX4 v1.15.4 velocity
# PID (--controller px4). Same paired seeds/conditions as the P-mode tables, so P vs PX4 is paired.
#   bash _px4_eval.sh                 # guard 2.0 rad/s (as all previous tables)
#   ANGV=6 TAG=_av6 CONDS=tau0 bash _px4_eval.sh   # relaxed angular-velocity guard
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
OUT=/tmp/px4e; mkdir -p $OUT; cd /tmp/rebuild
TAG=${TAG:-}; EXTRA=""; [ -n "${ANGV:-}" ] && EXTRA="--ang_vel_limit $ANGV"
declare -A COND=( [tau0]="" [A]="--wind_tau 10 --wind_gust 0.2" [B]="--wind_tau 3 --wind_gust 0.3" )
declare -A ARM=( [L0]="--no_residual" [GRUS]="--sl_residual /tmp/sl/res_gru_S.pt --sl_ema 0.3" )
run() { local f=$OUT/$1.json; shift; [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for C in ${CONDS:-tau0 A B}; do for A in L0 GRUS; do for S in 3000 4000 5000; do
  run "${A}_px4${TAG}_${C}_s${S}" $IL -p play.py --policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 \
      --marker_dist 18 22 --dr_scale 1.5 --seed $S --controller px4 $EXTRA ${COND[$C]} ${ARM[$A]} --arm_name ${A}_px4${TAG}
done; done; done
echo "PX4 EVAL DONE $(date +%T)"
