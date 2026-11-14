#!/bin/bash
# Train L0 on several seeds in parallel, then run _l0_s2r_eval.sh on every final checkpoint.
# Runs entirely inside the container (docker exec -d), so it survives the Claude session.
# usage: bash _l0_s2r_run.sh <tag> <iterations> <seed>...    e.g.  bash _l0_s2r_run.sh l0_s2r5 1500 1 2 3
set -u
TAG=$1; IT=$2; shift 2
ROOT=/tmp/$TAG; mkdir -p $ROOT
cd /tmp/rebuild
for S in "$@"; do
  /workspace/isaaclab/isaaclab.sh -p train.py --task_env --no_residual --dr_scale 1.0 --num_envs 2048 \
    --max_iterations $IT --seed $S --headless --run_name ${TAG}_s$S --log_root $ROOT/logs > $ROOT/train_s$S.log 2>&1 &
  sleep 60   # stagger Kit startup
done
wait
echo "TRAIN DONE $(date +%F_%T)" >> $ROOT/status
ARGS=""
for S in "$@"; do
  CK=$(ls -d $ROOT/logs/drone_bombard_ppo/*_${TAG}_s$S/ | tail -1)model_final.pt
  [ -f "$CK" ] && ARGS="$ARGS s$S=$CK" || echo "NO CKPT seed $S" >> $ROOT/status
done
OUT=$ROOT/eval bash /tmp/rebuild/_l0_s2r_eval.sh $ARGS > $ROOT/eval.log 2>&1
echo "EVAL DONE $(date +%F_%T)" >> $ROOT/status
