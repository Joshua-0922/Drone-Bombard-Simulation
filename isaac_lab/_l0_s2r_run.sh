#!/bin/bash
# Train L0 on several seeds in parallel, then run _l0_s2r_eval.sh on every final checkpoint.
# Runs entirely inside the container (docker exec -d), so it survives the Claude session.
# usage: bash _l0_s2r_run.sh <tag> <iterations> <seed>...    e.g.  bash _l0_s2r_run.sh l0_s2r5 1500 1 2 3
# EXTRA="--l0_std_fix" adds train.py flags. After training, every 250th checkpoint is scored on
# the VALIDATION seed 1000 x 200 (DR 1.0, disjoint from the paired-eval seeds) -> $ROOT/val.
set -u
TAG=$1; IT=$2; shift 2
ROOT=/tmp/$TAG; mkdir -p $ROOT
cd /tmp/rebuild
for S in "$@"; do
  /workspace/isaaclab/isaaclab.sh -p train.py --task_env --no_residual --dr_scale 1.0 --num_envs 2048 \
    --max_iterations $IT --seed $S --headless --run_name ${TAG}_s$S --log_root $ROOT/logs ${EXTRA:-} > $ROOT/train_s$S.log 2>&1 &
  sleep 60   # stagger Kit startup
done
wait
echo "TRAIN DONE $(date +%F_%T)" >> $ROOT/status
mkdir -p $ROOT/val
for S in "$@"; do
  D=$(ls -d $ROOT/logs/drone_bombard_ppo/*_${TAG}_s$S/ | tail -1)
  for C in $(seq 250 250 $IT); do
    CK=${D}model_$C.pt; [ $C -eq $IT ] && CK=${D}model_final.pt
    F=$ROOT/val/L0_s${S}_i${C}_dr1.0_s1000.json
    [ -f "$CK" ] && /workspace/isaaclab/isaaclab.sh -p play.py --policy $CK --no_residual --paired_eval --headless \
      --episodes 200 --num_envs 200 --arm_name L0_s${S}_i$C --marker_dist 18 22 --dr_scale 1.0 --seed 1000 \
      --out-json $F > ${F%.json}.log 2>&1
  done
done
echo "VAL DONE $(date +%F_%T)" >> $ROOT/status
ARGS=""
for S in "$@"; do
  CK=$(ls -d $ROOT/logs/drone_bombard_ppo/*_${TAG}_s$S/ | tail -1)model_final.pt
  [ -f "$CK" ] && ARGS="$ARGS s$S=$CK" || echo "NO CKPT seed $S" >> $ROOT/status
done
OUT=$ROOT/eval bash /tmp/rebuild/_l0_s2r_eval.sh $ARGS > $ROOT/eval.log 2>&1
echo "EVAL DONE $(date +%F_%T)" >> $ROOT/status
