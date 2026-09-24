#!/bin/bash
# Realistic-wind evaluation: per-episode MEAN wind + OU gust (Dryden-like), all arms,
# seeds {3000,4000,5000}. Conditions: A = 20% gust, tau 10 s (low-altitude Dryden);
# B = 30% gust, tau 3 s (harsher). Output /tmp/gust/{ARM}_{COND}_s{SEED}.json.
#   ARMS="L0 gi GRUS GRUR GRUU ORC" bash _gust_eval.sh
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
OUT=/tmp/gust; mkdir -p $OUT
cd /tmp/rebuild
ARMS=${ARMS:-"L0 gi GRUS GRUR GRUU ORC"}
declare -A ARG=( [L0]="--no_residual" [gi]="--sl_residual /tmp/sl/res_gi.pt --sl_ema 0.3"
  [GRUS]="--sl_residual /tmp/sl/res_gru_S.pt --sl_ema 0.3" [GRUR]="--sl_residual /tmp/sl/res_gru_R.pt --sl_ema 0.3"
  [GRUU]="--sl_residual /tmp/sl/res_gru_U.pt --sl_ema 0.3" [GRUUrel2]="--sl_residual /tmp/sl/res_gru_Urel2.pt --sl_ema 0.3"
  [GRUUrel4]="--sl_residual /tmp/sl/res_gru_Urel4.pt --sl_ema 0.3" [ORC]="--oracle_residual_wind_only --oracle_ema 0.3" )
declare -A COND=( [A]="--wind_tau 10 --wind_gust 0.2" [B]="--wind_tau 3 --wind_gust 0.3" )
run() { local f=$OUT/$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for C in A B; do for A in $ARMS; do for S in 3000 4000 5000; do
  run "${A}_${C}_s${S}" $IL -p play.py --policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 \
      --marker_dist 18 22 --dr_scale 1.5 --seed $S ${COND[$C]} ${ARG[$A]} --arm_name ${A}_${C}
done; done; done
echo "GUST EVAL DONE $(date +%T)"
