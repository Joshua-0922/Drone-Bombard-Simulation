#!/bin/bash
# Final candidate: GRU + MSE + temporal-consistency (lambda 20), no EMA at deployment.
# Train 1000 epochs on the v2 steady-wind dumps, then the full table battery, seeds {3000,4000,5000}.
# Output /tmp/final_C/GRUC_{cond}_s{seed}.json ; compare against GRU-S + EMA 0.3 references already on disk.
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
OUT=/tmp/final_C; PT=/tmp/sl/res_gru_C.pt; mkdir -p $OUT; cd /tmp/rebuild
if [ ! -s $PT ]; then
  echo "=== $(date +%T) train"
  $IL -p _fit_sl_seq.py /tmp/sl/v2_s1a.npz /tmp/sl/v2_s1b.npz --smooth 20 --epochs 1000 --report_every 100 \
      --export $PT > $OUT/train.log 2>&1 || { echo "TRAIN FAIL"; exit 1; }
fi
B="--policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 --seed"
G="--sl_residual $PT --sl_ema 1.0 --arm_name GRUC"
declare -A COND=( [tau0]="--marker_dist 18 22 --dr_scale 1.5" [A]="--marker_dist 18 22 --dr_scale 1.5 --wind_tau 10 --wind_gust 0.2"
  [B]="--marker_dist 18 22 --dr_scale 1.5 --wind_tau 3 --wind_gust 0.3"
  [dr0]="--marker_dist 18 22 --dr_scale 0" [dr0.5]="--marker_dist 18 22 --dr_scale 0.5" [dr1.0]="--marker_dist 18 22 --dr_scale 1.0"
  [dr2.5]="--marker_dist 18 22 --dr_scale 2.5" [range]="--marker_dist 26 30 --dr_scale 1.5"
  [tau10]="--marker_dist 18 22 --dr_scale 1.5 --wind_tau 10" [tau3]="--marker_dist 18 22 --dr_scale 1.5 --wind_tau 3"
  [tau1]="--marker_dist 18 22 --dr_scale 1.5 --wind_tau 1" [tau0.3]="--marker_dist 18 22 --dr_scale 1.5 --wind_tau 0.3" )
run() { local f=$OUT/$1.json; shift; [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for C in tau0 A B dr0 dr0.5 dr1.0 dr2.5 range tau10 tau3 tau1 tau0.3; do for S in 3000 4000 5000; do
  run "GRUC_${C}_s${S}" $IL -p play.py $B $S ${COND[$C]} $G
done; done
echo "FINAL C DONE $(date +%T)"
