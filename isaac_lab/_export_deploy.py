"""Export the trained flight policy (L0 actor) and the GRU residual to a numpy .npz for the ROS 2
deployment node (ros2_ws/src/drop_policy), plus golden input/output pairs so the numpy port can be
checked against torch. Runs in the Isaac container (torch); the output needs only numpy.

  isaaclab.sh -p _export_deploy.py --ckpt L0.pt --gru res_gru_S.pt --out drop_policy_models.npz
"""
import argparse, importlib.util, os
import numpy as np
import torch

p = argparse.ArgumentParser()
p.add_argument("--ckpt", required=True)
p.add_argument("--gru", required=True)
p.add_argument("--out", required=True)
a = p.parse_args()
spec = importlib.util.spec_from_file_location("mu", os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone_bombard", "math_utils.py"))
mu = importlib.util.module_from_spec(spec); spec.loader.exec_module(mu)

sd = torch.load(a.ckpt, map_location="cpu", weights_only=False)["model_state_dict"]
actor = {f"actor_{k.split('.')[1]}_{k.split('.')[2]}": sd[k].numpy() for k in sd if k.startswith("actor.")}
g = torch.jit.load(a.gru, map_location="cpu").eval()
gru = {"gru_w_ih": g.cell.weight_ih.detach().numpy(), "gru_w_hh": g.cell.weight_hh.detach().numpy(),
       "gru_b_ih": g.cell.bias_ih.detach().numpy(), "gru_b_hh": g.cell.bias_hh.detach().numpy(),
       "gru_head_w": g.head.weight.detach().numpy(), "gru_head_b": g.head.bias.detach().numpy(),
       "gru_mu": g.mu.detach().numpy().reshape(-1), "gru_sd": g.sd.detach().numpy().reshape(-1)}

# --- golden pairs ---
rng = np.random.default_rng(0)
obs = rng.uniform(-0.6, 0.6, size=(32, 26)).astype(np.float32)
net = torch.nn.Sequential(torch.nn.Linear(26, 256), torch.nn.ELU(), torch.nn.Linear(256, 256), torch.nn.ELU(), torch.nn.Linear(256, 7))
net.load_state_dict({k[len("actor."):]: v for k, v in sd.items() if k.startswith("actor.")})
with torch.no_grad():
    act = net(torch.from_numpy(obs)).numpy()
    h = torch.zeros(1, int(g.hidden_size)); gru_out = []
    for t in range(32):
        d, h = g(torch.from_numpy(obs[t:t + 1]), h); gru_out.append(d.numpy()[0])
gru_out = np.stack(gru_out)
# nominal CCIP exactly as DroneBombardEnv._nominal_impact (tau 0.22, mount -0.14, land eps 0.10, drag 0.07, g 9.81)
n = 64
pos = rng.uniform(-20, 20, (n, 2)); vel = rng.uniform(-6, 6, (n, 2)); vz = rng.uniform(-3, 1, n); alt = rng.uniform(2, 12, n)
T = lambda x: torch.tensor(x, dtype=torch.float32)
tau = 0.22
pos2 = T(pos) + T(vel) * tau; alt2 = T(alt) + T(vz) * tau
fall = alt2 - 0.14 - (0.0 + 0.10)
nom = mu.ballistic_impact(pos2, T(vel), T(vz), fall, 0.0, 9.81, torch.full((n,), 0.07), torch.zeros(n, 2)).numpy()
tf = mu.time_to_fall(T(alt), T(vz), 9.81).numpy()
np.savez(a.out, **actor, **gru, actor_std=sd["std"].numpy(),
         golden_obs=obs, golden_act=act, golden_gru=gru_out,
         golden_pos=pos, golden_vel=vel, golden_vz=vz, golden_alt=alt, golden_nom=nom, golden_tf=tf)
print("wrote", a.out, {k: v.shape for k, v in {**actor, **gru}.items()})
