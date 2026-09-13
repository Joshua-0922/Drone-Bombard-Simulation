#!/bin/bash
# L1-RL pilot: the residual-RL arm that is COMPARABLE to L1-SL gi.
#   frozen L0 seed 1 (rows 0:5, deterministic rollouts) + residual trunk 38->128->128->2 (rows 5:7)
#   same inputs (accum_obs), same authority (scale 2.0), same smoothing (EMA 0.3), same eval seeds.
# Two arms: zero-init ("can PPO find the drift?") and res_gi.pt-init ("does the terminal
# reward add anything on top of drift prediction?"). notes/research/l1_rl_preflight.md.
#
# Launch from the HOST with the wandb key (docker start leaves the container without it):
#   docker exec --env-file /opt/drone-bombard/.wandb.env -e PYTHONUNBUFFERED=1 -d isaac-verify \
#       bash /tmp/rebuild/_l1rl_pilot.sh
# ~2.3 h per arm at 2048 envs (16-18 s/iter measured 2026-09-13); evals ~1 min each.
set -u
cd /tmp/rebuild
IL=/workspace/isaaclab/isaaclab.sh
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt
ITERS=${ITERS:-500}
ARMS=${ARMS:-zero}      # "zero" = residual RL from delta=0 (the arm compared with the existing L1-SL table);
                        # "zero slinit" adds PPO fine-tuning that starts from res_gi.pt (still RL, not a new fit)
OUT=/tmp/l1rl; mkdir -p $OUT/logs
TRAIN="--task_env --resume $CK1 --residual_net --accum_obs --residual_scale 2.0 --residual_ema 0.3 \
       --nominal_std 0.01 --residual_init_std 0.05 --entropy_coef 0.0 --dr_scale 1.5 --headless --num_envs 2048 --seed 1 \
       --max_iterations $ITERS --log_root $OUT/logs"
SEEDS="3000 4000 5000"

run() { local f=$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }

for ARM in $ARMS; do
  if [ "$ARM" = slinit ]; then EXTRA="--residual_init_from /tmp/sl/res_gi.pt"; else EXTRA=""; fi
  echo "=== $(date +%T) train L1RL_$ARM ($ITERS iter)"
  $IL -p train.py $TRAIN $EXTRA --run_name pilot_$ARM > $OUT/train_$ARM.log 2>&1 || echo "FAIL train $ARM"
  CK=$(ls -d $OUT/logs/drone_bombard_ppo/*pilot_$ARM/model_final.pt 2>/dev/null | tail -1)
  [ -n "$CK" ] || { echo "no checkpoint for $ARM"; continue; }
  # Evaluate exactly like _gi_headline.sh: paired, 200 ep, DR 1.5, range 18-22, 3 seeds.
  B="--policy $CK --paired_eval --headless --episodes 200 --num_envs 200 --marker_dist 18 22 \
     --accum_obs --residual_scale 2.0 --residual_ema 0.3 --arm_name L1RL_$ARM"
  for S in $SEEDS; do
    run $OUT/L1RL_${ARM}_dr1.5_s$S $IL -p play.py $B --dr_scale 1.5 --seed $S
    run $OUT/L1RL_${ARM}_dr2.5_s$S $IL -p play.py $B --dr_scale 2.5 --seed $S
    # unseen range row of the existing table (/tmp/sl_R)
    run $OUT/L1RL_${ARM}_R_dr1.5_s$S $IL -p play.py ${B/--marker_dist 18 22/--marker_dist 26 30} --dr_scale 1.5 --seed $S
  done
done
echo "=== $(date +%T) L1RL PILOT DONE"
# Then, on one table with L0 / L1-SL gi / oracle (seed sets are asserted equal):
#   _agg_table1.py  -- add $OUT/L1RL_{zero,slinit}_dr1.5_s*.json next to /tmp/sl_GI/*.json
