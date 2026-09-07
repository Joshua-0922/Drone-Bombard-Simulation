#!/bin/bash
# Figure 4 (the main claim): CEP vs model-error strength, one curve per arm.
# The existing sweep has two defects that Table 1 just had fixed:
#   (a) its T2 column releases at 6.0 m -- untuned. Tuning the release altitude
#       alone took T2's CEP50 from 0.461 to 0.282 at DR 1.5, so the published
#       curve would be drawn against a weak baseline.
#   (b) it was flown on eval seeds {1000,2000,3000} while the learned arms live
#       on {3000,4000,5000}.
# DR 2.5 is already covered (t_reseed) and DR 1.5 by the main sweep, so only
# {0, 0.5, 1.0} is missing. 3 arms x 3 DR x 3 seeds = 27 runs.
set -u
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
IL=/workspace/isaaclab/isaaclab.sh
OUT=/tmp/dr_axis; mkdir -p $OUT
cd /tmp/rebuild
run() { local f=$OUT/$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }

for D in 0 0.5 1.0; do
  for S in 3000 4000 5000; do
    B="--headless --episodes 200 --num_envs 200 --marker_dist 18 22 --dr_scale $D --seed $S"
    run "L0_dr${D}_s${S}"     $IL -p play.py --policy $CK1 --paired_eval --no_residual --arm_name L0 $B
    run "SLtiltE_dr${D}_s${S}" $IL -p play.py --policy $CK1 --paired_eval \
        --sl_residual /tmp/sl/res_tilt.pt --sl_ema 0.3 --arm_name L1SL_tilt_ema $B
    run "T2p15tuned_dr${D}_s${S}" $IL -p baseline_drop.py --arm argmin --pass_speed 1.5 \
        --release_alt 3.5 $B
  done
done
echo "DR AXIS DONE $(date +%T)"
