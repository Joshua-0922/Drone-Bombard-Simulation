#!/bin/bash
# exp_034: learned temporal filter (GRU) vs the hand-crafted accumulators.
# Arms are TorchScript regressors injected through the same path as res_gi.pt
# (play.py --sl_residual, EMA 0.3, residual.scale = oracle_scale 2.0), evaluated
# on the stationary condition and on the OU ladder, seeds {3000,4000,5000}.
# References already on disk: /tmp/sl_GI/{L0,SLgiE,ORCwind}_dr1.5_s*  and
# /tmp/ou/{L0,SLgiE,ORCwindE}_tau*_s*.
#   ARMS="GRUS:/tmp/sl/res_gru_S.pt GRUM:/tmp/sl/res_gru_M.pt MLPM:/tmp/sl/res_gi_M.pt" bash _gru_eval.sh
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
OUT=/tmp/gru; mkdir -p $OUT
cd /tmp/rebuild
ARMS=${ARMS:-"GRUS:/tmp/sl/res_gru_S.pt"}
EMA=${EMA:-0.3}
TAUS=${TAUS:-"0 10 3 1 0.3"}
run() { local f=$OUT/$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for A in $ARMS; do
  NAME=${A%%:*}; PT=${A#*:}
  for T in $TAUS; do
    for S in 3000 4000 5000; do
      run "${NAME}_E_tau${T}_s${S}" $IL -p play.py --policy $CK1 --paired_eval --headless \
          --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale 1.5 --seed $S --wind_tau $T \
          --sl_residual $PT --sl_ema $EMA --arm_name ${NAME}_ema$EMA
    done
  done
done
echo "GRU EVAL DONE $(date +%T)"
