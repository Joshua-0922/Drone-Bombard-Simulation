#!/bin/bash
# exp_056: hand-designed accumulated features + MLP (/tmp/abl9/fit/sT/mlp_gi.pt, fit on DR 1.0 dumps), no EMA,
# on the full grid: DR 0..2.5, 26-30 m, wind conditions of _w9_chain.sh. Pairs with existing L0/GRUS JSONs.
set -u
IL=/workspace/isaaclab/isaaclab.sh; O=/tmp/hand9; mkdir -p $O/eval; cd /tmp/rebuild
OU="wind_std=2.449 wind_max=8.165"
declare -A W=(
  [dr0]="--dr_scale 0" [dr0.5]="--dr_scale 0.5" [dr1.0]="--dr_scale 1.0" [dr1.5]="--dr_scale 1.5" [dr2.0]="--dr_scale 2.0" [dr2.5]="--dr_scale 2.5"
  [R26-30_dr1.0]="--dr_scale 1.0 --marker_dist 26 30"
  [steady]="--dr_scale 1.0 --model_err_set wind_gust_model=none"
  [realA]="--dr_scale 1.0 --wind_tau 10 --wind_gust 0.2 --model_err_set $OU"
  [realB]="--dr_scale 1.0 --wind_tau 3 --wind_gust 0.3 --model_err_set $OU"
  [tau10]="--dr_scale 1.0 --wind_tau 10 --model_err_set wind_speed_dist=gaussian $OU"
  [tau3]="--dr_scale 1.0 --wind_tau 3 --model_err_set wind_speed_dist=gaussian $OU"
  [tau1]="--dr_scale 1.0 --wind_tau 1 --model_err_set wind_speed_dist=gaussian $OU"
  [tau0.3]="--dr_scale 1.0 --wind_tau 0.3 --model_err_set wind_speed_dist=gaussian $OU"
)
declare -A CK=( [s1]=$(ls -d /tmp/l0_s2r7/logs/drone_bombard_ppo/*_s1)/model_1000.pt [s2]=$(ls -d /tmp/l0_s2r7/logs/drone_bombard_ppo/*_s2)/model_1000.pt [s4]=$(ls -d /tmp/l0_s2r7b/logs/drone_bombard_ppo/*_s4)/model_final.pt )
for C in dr1.0 dr0 dr0.5 dr1.5 dr2.0 dr2.5 R26-30_dr1.0 steady realA realB tau10 tau3 tau1 tau0.3; do
  M="--marker_dist 18 22"; [[ "${W[$C]}" == *marker_dist* ]] && M=""
  for T in s1 s2 s4; do for S in 3000 4000 5000; do
    f=$O/eval/HAND_${T}_${C}_s$S.json; [ -s $f ] && continue; echo "=== $(date +%T) $f"
    $IL -p play.py --policy ${CK[$T]} --paired_eval --headless --episodes 200 --num_envs 200 $M ${W[$C]} --seed $S \
      --sl_residual /tmp/abl9/fit/$T/mlp_gi.pt --arm_name HAND_$T --out-json $f > ${f%.json}.log 2>&1 || echo "FAIL $f" >> $O/status
  done; done
  echo "$C DONE $(date +%T)" >> $O/status
done
echo "ALL DONE $(date +%T)" >> $O/status
