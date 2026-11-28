#!/bin/bash
# GRU-S on a sim2real L0 (PX4 plant): dump -> fit -> paired eval, per L0 seed.
# Dump at the L0 training scale (DR 1.0), collection seeds 1000/2000 x 1500 episodes;
# eval = the _l0_s2r_eval.sh grid (DR 0/0.5/1.0/1.5 at 18-22 m, 26-30 m at DR 1.0, seeds 3000/4000/5000 x 200)
# so every GRUS json pairs with the L0 json of the same seed. Oracle (wind-only, EMA 0.3) at DR 1.0 as the ceiling.
# usage: ROOT=/tmp/grus7 bash _grus_s2r.sh s1=/path/model_1000.pt [s2=...]
set -u
IL=/workspace/isaaclab/isaaclab.sh
ROOT=${ROOT:-/tmp/grus7}
EP=${EP:-1500}
LABEL=${LABEL:-instant}   # miss = L0 leftover landing miss (exp_052)
ORC=${ORC:-1}
GAIN=${GAIN:-1.0}   # injection gain for GRU and oracle (exp_050: L0 self-compensates part of the drift)
DRS=${DRS:-"1.0 0 0.5 1.5"}; FAR=${FAR:-1}   # quick check: DRS=1.0 FAR=0
[ "$GAIN" = 1.0 ] && SFX= || SFX=_g$GAIN
cd /tmp/rebuild
run() { local f=$1.json; shift; [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"; "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"; }
for arg in "$@"; do
  T=${arg%%=*}; CK=${arg#*=}; D=$ROOT/$T; mkdir -p $D/eval
  for P in "a 1000" "b 2000"; do set -- $P
    [ -s $D/v2_s1$1.npz ] || $IL -p play.py --policy $CK --no_residual --paired_eval --unpaired --headless --num_envs 256 --episodes $EP \
      --marker_dist 18 22 --dr_scale 1.0 --seed $2 --dump_sl $D/v2_s1$1.npz --arm_name dump_$T --out-json $D/dump_$1.json > $D/dump_$1.log 2>&1 || echo "FAIL dump $T $1"
  done
  [ -s $D/res_gru_S.pt ] || $IL -p _fit_sl_seq.py $D/v2_s1a.npz $D/v2_s1b.npz --label $LABEL --export $D/res_gru_S.pt > $D/fit.log 2>&1 || echo "FAIL fit $T"
  [ -s $D/res_gru_S.pt ] || continue
  B="--policy $CK --paired_eval --headless --episodes 200 --num_envs 200"
  G="--sl_residual $D/res_gru_S.pt --sl_ema 0.3 --sl_gain $GAIN --arm_name GRUS_$T"
  for S in 3000 4000 5000; do
    for DR in $DRS; do
      run $D/eval/GRUS_${T}${SFX}_dr${DR}_s$S $IL -p play.py $B --marker_dist 18 22 --dr_scale $DR --seed $S $G
    done
    [ "$FAR" = 1 ] && run $D/eval/GRUS_${T}${SFX}_R26-30_dr1.0_s$S $IL -p play.py $B --marker_dist 26 30 --dr_scale 1.0 --seed $S $G
    [ "$ORC" = 1 ] && run $D/eval/ORC_${T}${SFX}_dr1.0_s$S $IL -p play.py $B --marker_dist 18 22 --dr_scale 1.0 --seed $S \
      --oracle_residual_wind_only --oracle_ema 0.3 --oracle_gain $GAIN --arm_name ORC_$T
  done
  echo "GRUS $T DONE $(date +%F_%T)" >> $ROOT/status
done
echo "GRUS ALL DONE $(date +%F_%T)" >> $ROOT/status
