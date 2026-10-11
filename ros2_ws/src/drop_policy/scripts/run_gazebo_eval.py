#!/usr/bin/env python3
"""Run drop-policy episodes in Gazebo + PX4 SITL, one fresh launch per episode (run INSIDE the
drone-bombard-harmonic container, after sourcing /root/ros2_ws and /workspace/ros2_ws installs).

  python3 run_gazebo_eval.py --out /tmp/gz_eval [--only S1] [--arms L0,GRU]

Each scenario fixes the Isaac handoff state (heading, cruise speed, altitude, distance, lateral offset)
and a constant wind -- deterministic test conditions, no domain randomization. The drone is spawned
on the ground `runup` metres before the handoff point, facing the approach heading.
"""
import argparse, json, math, os, subprocess, time

MODELS = "/workspace/gazebo_models"
PKG = "/workspace/ros2_ws/src/drop_policy"
SCENARIOS = {  # name: heading_deg, speed, alt, marker_dist, lateral, wind (x, y) ENU m/s
    "S1_straight":      dict(heading=0,   speed=4.0, alt=10.0, dist=20.0, lat=0.0,  wind=(0.0, 0.0)),
    "S2_left20_slow":   dict(heading=20,  speed=3.0, alt=9.0,  dist=19.0, lat=2.0,  wind=(0.0, 0.0)),
    "S3_right25_fast":  dict(heading=-25, speed=5.0, alt=11.0, dist=21.0, lat=-2.0, wind=(0.0, 0.0)),
    "W1_cross3":        dict(heading=0,   speed=4.0, alt=10.0, dist=20.0, lat=0.0,  wind=(0.0, 3.0)),
    "W2_head4":         dict(heading=0,   speed=4.0, alt=10.0, dist=20.0, lat=0.0,  wind=(-4.0, 0.0)),
    "W3_tail3_left20":  dict(heading=20,  speed=3.0, alt=9.0,  dist=19.0, lat=2.0,
                             wind=(3.0 * math.cos(math.radians(20)), 3.0 * math.sin(math.radians(20)))),
}
TARGET = (40.0, 10.0)
RUNUP = 18.0


def cleanup():
    for pat in ("[g]z sim", "[p]x4_sitl_default/bin/px4", "[M]icroXRCEAgent", "[p]arameter_bridge",
                "[d]rone_controller", "[d]rop_mission", "[r]os2 launch drop_policy"):
        subprocess.run(["pkill", "-9", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)


def run(name, sc, arm, out):
    h = math.radians(sc["heading"]); u = (math.cos(h), math.sin(h)); n = (-u[1], u[0])
    back = sc["dist"] + RUNUP
    sx = TARGET[0] - u[0] * back + n[0] * sc["lat"]
    sy = TARGET[1] - u[1] * back + n[1] * sc["lat"]
    tag = f"{name}_{arm}"
    world = f"/tmp/drop_world_{tag}.sdf"
    tpl = open(f"{MODELS}/worlds/drop_world.sdf.in").read()
    open(world, "w").write(tpl.replace("@TX@", str(TARGET[0])).replace("@TY@", str(TARGET[1]))
                           .replace("@SX@", f"{sx:.3f}").replace("@SY@", f"{sy:.3f}").replace("@SYAW@", f"{h:.5f}"))
    res = f"{out}/{tag}.json"
    params = f"/tmp/drop_params_{tag}.yaml"
    open(params, "w").write(f"""drop_mission_node:
  ros__parameters:
    models_path: {PKG}/models/drop_policy_models.npz
    target_x: {TARGET[0]}
    target_y: {TARGET[1]}
    spawn_x: {sx:.3f}
    spawn_y: {sy:.3f}
    heading_deg: {float(sc['heading'])}
    cruise_speed: {sc['speed']}
    cruise_alt: {sc['alt']}
    marker_dist: {sc['dist']}
    use_residual: {'true' if arm == 'GRU' else 'false'}
    result_path: {res}
    tag: {tag}
""")
    if os.path.exists(res):
        os.remove(res)
    cleanup()
    t0 = time.time()
    log = open(f"{out}/{tag}.log", "w")
    p = subprocess.Popen(["ros2", "launch", "drop_policy", "drop_eval.launch.py", f"world:={world}",
                          f"params:={params}", f"wind_x:={sc['wind'][0]:.3f}", f"wind_y:={sc['wind'][1]:.3f}"],
                         stdout=log, stderr=subprocess.STDOUT)
    while time.time() - t0 < 240 and not os.path.exists(res):
        time.sleep(1)
    time.sleep(3)
    p.terminate()
    try:
        p.wait(10)
    except subprocess.TimeoutExpired:
        p.kill()
    cleanup()
    if not os.path.exists(res):
        print(f"{tag}: NO RESULT after {time.time()-t0:.0f}s (see {out}/{tag}.log, /tmp/px4_drop.log)", flush=True)
        return None
    r = json.load(open(res))
    rel = r.get("release") or {}
    print(f"{tag}: {r['outcome']} err={r.get('impact_err')} hover_mount={r.get('hover_mount_offset')} "
          f"release_v={rel.get('vel')} t={time.time()-t0:.0f}s", flush=True)
    return r


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/gz_eval")
    ap.add_argument("--only", default="")
    ap.add_argument("--arms", default="L0,GRU")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for name, sc in SCENARIOS.items():
        if a.only and name not in a.only.split(","):
            continue
        for arm in a.arms.split(","):
            run(name, sc, arm, a.out)
