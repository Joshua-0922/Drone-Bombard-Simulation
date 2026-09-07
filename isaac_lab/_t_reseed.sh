#!/bin/bash
# Table 1 seed alignment. The scripted arms (exp_027) were flown on eval seeds
# {1000,2000,3000}; the L1-SL arms (exp_028/029) on {3000,4000,5000}. Only seed
# 3000 overlaps, so no T row has ever been paired against an L1 row. Re-fly the
# T arms on 4000/5000 so every row of the table sits on the same 600 scenarios.
#   DR 1.5 block: seed 3000 already exists in /tmp/sweep2 (6.0 m) and
#                 /tmp/sweep3 (3.5 m) -- only 4000/5000 are missing.
#   DR 2.5 block: the tuned 3.5 m arms were never run at DR 2.5 at all, so all
#                 three seeds are needed there.
set -u
OUT=/tmp/t_reseed; mkdir -p $OUT
cd /tmp/rebuild
IL=/workspace/isaaclab/isaaclab.sh
COMMON="--headless --episodes 200 --num_envs 200 --marker_dist 18 22"

run() {
  local f=$OUT/$1.json; shift
  [ -s "$f" ] && { echo "SKIP $f"; return; }
  echo "=== $(date +%T) $f"
  "$@" --out-json "$f" > "${f%.json}.log" 2>&1 || echo "FAIL $f"
}

# ---- DR 1.5 (seen) : the main Table 1 block --------------------------------
for S in 4000 5000; do
  B="$COMMON --seed $S --dr_scale 1.5"
  run "T0hover_seen_s${S}"      $IL -p baseline_drop.py --arm hover  $B
  run "T0hover_alt3.5_s${S}"    $IL -p baseline_drop.py --arm hover  --release_alt 3.5 $B
  run "T1ccip_seen_s${S}"       $IL -p baseline_drop.py --arm ccip   --pass_speed 3.0 $B
  run "T2p15_seen_s${S}"        $IL -p baseline_drop.py --arm argmin --pass_speed 1.5 $B
  run "T2p15_alt3.5_s${S}"      $IL -p baseline_drop.py --arm argmin --pass_speed 1.5 --release_alt 3.5 $B
  run "T2p30_seen_s${S}"        $IL -p baseline_drop.py --arm argmin --pass_speed 3.0 $B
  run "T3orac_seen_s${S}"       $IL -p baseline_drop.py --arm oracle --pass_speed 3.0 $B
done

# ---- DR 2.5 (unseenD) : tuned arms, never run at this DR --------------------
for S in 3000 4000 5000; do
  B="$COMMON --seed $S --dr_scale 2.5"
  run "T0hover_D_alt3.5_s${S}"  $IL -p baseline_drop.py --arm hover  --release_alt 3.5 $B
  run "T2p15_D_alt3.5_s${S}"    $IL -p baseline_drop.py --arm argmin --pass_speed 1.5 --release_alt 3.5 $B
  run "T3orac_D_s${S}"          $IL -p baseline_drop.py --arm oracle --pass_speed 3.0 $B
done
echo "T_RESEED DONE $(date +%T)"
