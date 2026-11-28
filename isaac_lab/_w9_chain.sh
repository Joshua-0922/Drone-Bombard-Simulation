#!/bin/bash
# Table 3 (wind conditions) on the PX4 plant: L0 vs L0+GRU-S, 3 L0 seeds x eval seeds 3000/4000/5000 x 200.
# OU conditions use wind_std = 6/sqrt(6) = 2.449 (per-axis rms of the default U[0,6] m/s wind) and cap 8.165
# (old cap/std ratio 5/1.5); ladder starts from the stationary Gaussian (wind_speed_dist=gaussian).
# usage: bash _w9_chain.sh            (full grid)   |   CHECK=1 bash _w9_chain.sh   (wind dumps only)
set -u
IL=/workspace/isaaclab/isaaclab.sh; R=/tmp/wind9; mkdir -p $R/eval $R/check; cd /tmp/rebuild
OU="wind_std=2.449 wind_max=8.165"
declare -A W=(
  [default]=""
  [steady]="--model_err_set wind_gust_model=none"
  [realA]="--wind_tau 10 --wind_gust 0.2 --model_err_set $OU"
  [realB]="--wind_tau 3 --wind_gust 0.3 --model_err_set $OU"
  [tau10]="--wind_tau 10 --model_err_set wind_speed_dist=gaussian $OU"
  [tau3]="--wind_tau 3 --model_err_set wind_speed_dist=gaussian $OU"
  [tau1]="--wind_tau 1 --model_err_set wind_speed_dist=gaussian $OU"
  [tau0.3]="--wind_tau 0.3 --model_err_set wind_speed_dist=gaussian $OU"
)
declare -A CK=(
  [1]=/tmp/l0_s2r7/logs/drone_bombard_ppo/2026-11-23_12-11-18_l0_s2r7_s1/model_1000.pt
  [2]=/tmp/l0_s2r7/logs/drone_bombard_ppo/2026-11-23_12-12-19_l0_s2r7_s2/model_1000.pt
  [4]=/tmp/l0_s2r7b/logs/drone_bombard_ppo/2026-11-26_13-39-07_l0_s2r7b_s4/model_final.pt
)
run() { local f=$1.json; shift; [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f" | tee -a $R/status; }
if [ "${CHECK:-0}" = 1 ]; then
  for C in default steady realA realB tau10 tau3 tau1 tau0.3; do
    [ -s $R/check/$C.npz ] || run $R/check/$C $IL -p play.py --policy ${CK[1]} --no_residual --paired_eval --unpaired --headless \
      --num_envs 256 --episodes 512 --marker_dist 18 22 --dr_scale 1.0 --seed 3000 ${W[$C]} --dump_sl $R/check/$C.npz --arm_name chk_$C
  done
  echo "CHECK DONE $(date +%F_%T)" >> $R/status; exit
fi
for C in ${CONDS:-steady realA realB tau10 tau3 tau1 tau0.3}; do
  for T in 1 2 4; do for S in 3000 4000 5000; do
    B="--policy ${CK[$T]} --paired_eval --headless --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale 1.0 --seed $S ${W[$C]}"
    run $R/eval/L0_s${T}_${C}_s$S $IL -p play.py $B --no_residual --arm_name L0_s$T
    run $R/eval/GRUS_s${T}_${C}_s$S $IL -p play.py $B --sl_residual /tmp/grus8/s$T/res_gru_S.pt --sl_ema 0.3 --arm_name GRUS_s$T
  done; done
  echo "$C DONE $(date +%F_%T)" >> $R/status
done
echo "ALL DONE $(date +%F_%T)" >> $R/status
