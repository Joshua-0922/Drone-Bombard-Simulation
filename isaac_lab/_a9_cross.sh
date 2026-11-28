#!/bin/bash
# exp_057 (A9): cross-injection of the running-mean residual (RMR). Policy L0 seed P flies with the residual fitted on L0 seed R.
# Diagonal = /tmp/hand9/eval/HAND_sP_dr1.0_s*.json (exp_056). DR 1.0, 18-22 m, no EMA.
set -u
IL=/workspace/isaaclab/isaaclab.sh; O=/tmp/a9x; mkdir -p $O/eval; cd /tmp/rebuild
declare -A CK=( [s1]=$(ls -d /tmp/l0_s2r7/logs/drone_bombard_ppo/*_s1)/model_1000.pt [s2]=$(ls -d /tmp/l0_s2r7/logs/drone_bombard_ppo/*_s2)/model_1000.pt [s4]=$(ls -d /tmp/l0_s2r7b/logs/drone_bombard_ppo/*_s4)/model_final.pt )
for P in s1 s2 s4; do for R in s1 s2 s4; do for S in 3000 4000 5000; do
  f=$O/eval/RMR_P${P}_R${R}_s$S.json
  [ $P = $R ] && { ln -sf /tmp/hand9/eval/HAND_${P}_dr1.0_s$S.json $f; continue; }
  [ -s $f ] && continue; echo "=== $(date +%T) $f"
  $IL -p play.py --policy ${CK[$P]} --paired_eval --headless --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale 1.0 --seed $S \
    --sl_residual /tmp/abl9/fit/$R/mlp_gi.pt --arm_name RMR_P${P}_R$R --out-json $f > ${f%.json}.log 2>&1 || echo "FAIL $f" >> $O/status
done; done; done
echo "ALL DONE $(date +%T)" >> $O/status
