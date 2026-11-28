#!/bin/bash
# exp_055: scripted baselines T0/T2 on the PX4 plant (tuned on validation seed 1000: release_alt 5.3),
# L0 / L0+GRU at unseen DR 2.0/2.5, capacity-matched GRU (hidden 128, 300 epochs), GRU without EMA in realistic gust B.
set -u
IL=/workspace/isaaclab/isaaclab.sh; O=/tmp/base9; mkdir -p $O/eval $O/fit; cd /tmp/rebuild
run() { local f=$O/eval/$1.json; shift; [ -s "$f" ] && return; echo "=== $(date +%T) $f"
  "$@" --headless --episodes 200 --num_envs 200 --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f" >> $O/status; }
declare -A CK=( [s1]=$(ls -d /tmp/l0_s2r7/logs/drone_bombard_ppo/*_s1)/model_1000.pt [s2]=$(ls -d /tmp/l0_s2r7/logs/drone_bombard_ppo/*_s2)/model_1000.pt [s4]=$(ls -d /tmp/l0_s2r7b/logs/drone_bombard_ppo/*_s4)/model_final.pt )
base() { # tag dr-or-far seed
  local c=$2 M="--marker_dist 18 22" D=$2; [ "$c" = far ] && { M="--marker_dist 26 30"; D=1.0; c=R26-30_dr1.0; } || c=dr$c
  run T0_${c}_s$3 $IL -p baseline_drop.py --arm hover --release_alt 5.3 --dr_scale $D $M --seed $3
  run T2v1.5_${c}_s$3 $IL -p baseline_drop.py --arm argmin --pass_speed 1.5 --release_alt 5.3 --dr_scale $D $M --seed $3
  run T2v1.0_${c}_s$3 $IL -p baseline_drop.py --arm argmin --pass_speed 1.0 --release_alt 5.3 --dr_scale $D $M --seed $3
}
for S in 3000 4000 5000; do base x 1.0 $S; run T2v2.5_dr1.0_s$S $IL -p baseline_drop.py --arm argmin --pass_speed 2.5 --release_alt 5.3 --dr_scale 1.0 --marker_dist 18 22 --seed $S; done
echo "BASE DR1 DONE $(date +%T)" >> $O/status
for C in 0 0.5 1.5 far 2.0 2.5; do for S in 3000 4000 5000; do base x $C $S; done; done
echo "BASE DONE $(date +%T)" >> $O/status
for T in s1 s2 s4; do for DR in 2.0 2.5; do for S in 3000 4000 5000; do
  B="$IL -p play.py --policy ${CK[$T]} --paired_eval --marker_dist 18 22 --dr_scale $DR --seed $S"
  run L0_${T}_dr${DR}_s$S $B --no_residual --arm_name L0_$T
  run GRUS_${T}_dr${DR}_s$S $B --sl_residual /tmp/grus8/$T/res_gru_S.pt --sl_ema 0.3 --arm_name GRUS_$T
done; done; done
echo "DR2 DONE $(date +%T)" >> $O/status
for T in s1 s2 s4; do
  [ -s $O/fit/gru128_$T.pt ] || $IL -p _fit_sl_seq.py /tmp/grus8/$T/v2_s1a.npz /tmp/grus8/$T/v2_s1b.npz --label miss --hidden 128 --epochs 300 --export $O/fit/gru128_$T.pt > $O/fit/gru128_$T.log 2>&1 || echo "FAIL fit $T" >> $O/status
  for S in 3000 4000 5000; do
    B="$IL -p play.py --policy ${CK[$T]} --paired_eval --marker_dist 18 22 --dr_scale 1.0 --seed $S"
    run GRU128_${T}_dr1.0_s$S $B --sl_residual $O/fit/gru128_$T.pt --sl_ema 0.3 --arm_name GRU128_$T
    run GRUnoEMA_${T}_realB_s$S $B --sl_residual /tmp/grus8/$T/res_gru_S.pt --sl_ema 1.0 --wind_tau 3 --wind_gust 0.3 --model_err_set wind_std=2.449 wind_max=8.165 --arm_name GRUnoEMA_$T
  done
done
echo "ALL DONE $(date +%T)" >> $O/status
