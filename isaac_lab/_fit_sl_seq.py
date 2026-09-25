"""Learned temporal filter for the supervised impact residual (replaces the
hand-crafted accumulators of _fit_sl_residual.py).

The prefix-mean tilt/velocity features assume the wind is constant for the
whole episode; under OU wind that assumption is what breaks (Rule 42). A GRU
over the raw 26-channel observation stream can learn its own window -- how
much to trust old evidence -- from data that mixes correlation times.

Same protocol as the MLP: labels are the wind-only oracle drift (metres,
unclamped), the loss is scored on frames the payload is carried, the split is
BY EPISODE, and the export is a TorchScript module whose normalisation travels
with the weights. The exported module is STATEFUL: ``forward(x, h) -> (d, h)``
with ``x`` the raw observation of ONE policy step, so play.py --sl_residual
runs it online with one hidden state per env (reset on episode start).

  isaaclab.sh -p _fit_sl_seq.py /tmp/sl/dr1.5_s1a.npz /tmp/sl/dr1.5_s1b.npz [--test_npz ...] --export /tmp/sl/res_gru.pt
"""
import argparse
import os

import numpy as np
import torch
import torch.nn as nn

p = argparse.ArgumentParser()
p.add_argument("npz", nargs="+")
p.add_argument("--test_npz", nargs="*", default=[],
               help="Hold these files out entirely (else: episode-wise 2:1 split of the train files)")
p.add_argument("--hidden", type=int, default=64)
p.add_argument("--epochs", type=int, default=150)
p.add_argument("--lr", type=float, default=1e-3)
p.add_argument("--seed", type=int, default=0)
p.add_argument("--export", default=None, metavar="PT")
p.add_argument("--label", default="instant", choices=["instant", "realised"],
               help="instant = the dump's oracle drift (wind at that instant, held for the fall). "
                    "realised = re-integrate the fall from the frame's raw state with the wind that "
                    "ACTUALLY followed (the dump's recorded stream, nominal latency/coefficient) -- "
                    "the quantity the landing depends on, and what a real drop would reveal. "
                    "Identical under constant wind; differs by the wind's change during the fall.")
p.add_argument("--report_every", type=int, default=100)
p.add_argument("--nll", action="store_true",
               help="Uncertainty head: predict mean AND log-variance of the drift, trained with the "
                    "Gaussian negative log-likelihood. The export shrinks the correction by "
                    "sigma0^2 / (sigma0^2 + sigma^2), so a state whose label is unpredictable (fast "
                    "wind) backs off toward zero correction (= L0) on its own.")
p.add_argument("--sigma0", type=float, default=0.3,
               help="--nll: shrinkage scale in metres (~ std of the true drift).")
p.add_argument("--shrink", default="absolute", choices=["absolute", "relative"],
               help="--nll: absolute = sigma0^2/(sigma0^2+sigma^2) (shrinks even in steady wind). "
                    "relative = min(1, (sigma_ref/sigma)^p) with sigma_ref = the held-out sigma of "
                    "the STEADY-wind files -- no loss in steady wind, backs off only when the state "
                    "is more uncertain than that.")
p.add_argument("--shrink_p", type=float, default=4.0)
p.add_argument("--steady_tag", default="s1", help="substring of the steady-wind dump names")
a = p.parse_args()
torch.manual_seed(a.seed)
np.random.seed(a.seed)
dev = "cuda" if torch.cuda.is_available() else "cpu"


import importlib.util as _ilu
_spec = _ilu.spec_from_file_location("mu", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                        "drone_bombard", "math_utils.py"))
_mu = _ilu.module_from_spec(_spec); _spec.loader.exec_module(_mu)
FUT = 15   # policy steps of future wind carried per frame (1.5 s > latency 0.22 + fall ~0.9)


def realised_drift(d, new):
    """Counterfactual REALISED drift per frame: integrate the fall from frame t's
    raw state with the recorded wind from t onward (zero-order hold, latency
    offset), minus the same integration under the constant wind w_t. Adding
    that difference to the dump's instant drift cancels the nominal exactly, so
    predictor and label stay on one integrator (Rule 31)."""
    if "state" not in d:
        raise SystemExit("this dump has no raw state; re-collect with the current play.py --dump_sl")
    tau_s, mount_z, ground_z, bc, g, dt = [float(x) for x in d["consts"]]
    st, wind, drift = d["state"], d["wind"], d["drift"]
    T, N = new.shape
    # future wind stream per frame, held at the last sample inside the episode
    fut = np.empty((T, N, FUT, 2), np.float32)
    end = np.full(N, T)                                  # exclusive end of the current episode
    for t in range(T - 1, -1, -1):
        if t + 1 < T:
            end = np.where(new[t + 1], t + 1, end)
        idx = np.minimum(t + np.arange(FUT)[:, None], end[None, :] - 1)   # (FUT, N)
        fut[t] = np.transpose(wind[idx, np.arange(N)[None, :]], (1, 0, 2))
    F = T * N
    pos = torch.as_tensor(st[..., 0:2].reshape(F, 2)); vel = torch.as_tensor(st[..., 2:4].reshape(F, 2))
    alt = torch.as_tensor(st[..., 4].reshape(F)); vz = torch.as_tensor(st[..., 5].reshape(F))
    w0 = torch.as_tensor(wind.reshape(F, 2)); ws = torch.as_tensor(fut.reshape(F, FUT, 2))
    tau = torch.full((F,), tau_s)
    pos_rel, alt_rel = pos + vel * tau.unsqueeze(-1), alt + vz * tau + mount_z
    bcv = torch.full((F,), bc)
    real_const = _mu.integrate_payload_impact(pos_rel, vel, vz, alt_rel, w0, bcv, g, ground_z=ground_z, dt=dt)
    real_seq = _mu.integrate_payload_impact(pos_rel, vel, vz, alt_rel, w0, bcv, g, ground_z=ground_z, dt=dt,
                                            wind_seq=ws, wind_hold=int(round(0.1 / dt)),
                                            wind_offset=int(round(tau_s / dt)))
    return drift + (real_seq - real_const).numpy().reshape(T, N, 2)


def episodes(f):
    """Split one dump (T, N, ...) into per-episode sequences."""
    d = np.load(f)
    obs, drift, ep_len, att = d["obs"], d["drift"], d["ep_len"], d["attached"]
    T, N = ep_len.shape
    new = np.ones((T, N), bool)
    new[1:] = ep_len[1:] <= ep_len[:-1]
    if a.label == "realised":
        drift = realised_drift(d, new)
        gap = np.linalg.norm(drift - d["drift"], axis=-1)[att > 0.5]
        print(f"  {os.path.basename(f)}: realised vs instant label |diff| mean {gap.mean():.3f} m  p90 {np.percentile(gap, 90):.3f} m")
    out = []
    for n in range(N):
        starts = np.flatnonzero(new[:, n]).tolist() + [T]
        for s, e in zip(starts[:-1], starts[1:]):
            m = att[s:e, n] > 0.5
            if m.sum() < 5:
                continue
            out.append((obs[s:e, n].astype(np.float32), drift[s:e, n].astype(np.float32), m))
    return out


train, test, tag = [], [], []
for f in a.npz:
    eps = episodes(f)
    if a.test_npz:
        train += eps
    else:
        rng = np.random.default_rng(a.seed)
        te = np.zeros(len(eps), bool)
        te[rng.permutation(len(eps))[: len(eps) // 3]] = True
        train += [e for e, t in zip(eps, te) if not t]
        test += [e for e, t in zip(eps, te) if t]
        tag += [os.path.basename(f)] * int(te.sum())
for f in a.test_npz:
    eps = episodes(f)
    test += eps
    tag += [os.path.basename(f)] * len(eps)
print(f"train episodes {len(train)}  test episodes {len(test)}  "
      f"frames train {sum(e[2].sum() for e in train)}  test {sum(e[2].sum() for e in test)}")

# input normalisation from the carried frames of the training set
Xtr = np.concatenate([e[0][e[2]] for e in train])
mu, sd = Xtr.mean(0), Xtr.std(0) + 1e-6


def batchify(eps):
    L = max(len(e[0]) for e in eps)
    X = np.zeros((len(eps), L, eps[0][0].shape[-1]), np.float32)
    Y = np.zeros((len(eps), L, 2), np.float32)
    M = np.zeros((len(eps), L), bool)
    for i, (o, y, m) in enumerate(eps):
        X[i, :len(o)] = (o - mu) / sd
        Y[i, :len(o)] = y
        M[i, :len(o)] = m
    return (torch.as_tensor(X, device=dev), torch.as_tensor(Y, device=dev),
            torch.as_tensor(M, device=dev))


class Filter(nn.Module):
    def __init__(self, n_in, h, n_out=2):
        super().__init__()
        self.gru = nn.GRU(n_in, h, batch_first=True)
        self.head = nn.Linear(h, n_out)

    def forward(self, x):
        y, _ = self.gru(x)
        return self.head(y)


def split(out):
    """(mu, logvar) for the NLL head; logvar clamped so early training cannot blow up."""
    return out[..., :2], out[..., 2:].clamp(-6.0, 4.0)


def loss_fn(out, Y, M):
    if a.nll:
        mu_, lv = split(out)
        err = ((Y - mu_) ** 2 / lv.exp() + lv).sum(-1)
    else:
        err = ((out - Y) ** 2).sum(-1)
    return (err * M).sum() / M.sum()


net = Filter(Xtr.shape[-1], a.hidden, 4 if a.nll else 2).to(dev)

def r2(pred, y):
    return 1 - ((pred - y) ** 2).sum() / ((y - y.mean(0)) ** 2).sum()


Xte, Yte, Mte = batchify(test)
LAST = torch.zeros_like(Mte)
for i, (_, _, m) in enumerate(test):
    LAST[i, np.flatnonzero(m)[-10:]] = True
TAGS = np.array(tag) if tag else None


def report(ep):
    net.eval()
    with torch.no_grad():
        P = net(Xte)
    SIG = None
    if a.nll:
        P, lv = split(P)
        SIG = (0.5 * lv).exp().mean(-1)          # per-frame sigma (metres), averaged over x,y
    line = f"[report] epoch {ep:4d}"
    for name, mask in (("carried", Mte), ("release", LAST)):
        pr, yy = P[mask].cpu().numpy(), Yte[mask].cpu().numpy()
        line += f" | R2 {name} {r2(pr, yy):.3f}"
    jit = np.abs(np.diff(P.cpu().numpy(), axis=1)).sum(-1)[Mte[:, 1:].cpu().numpy()].mean()
    line += f" | step change {jit:.3f} m/step"
    if TAGS is not None and len(set(tag)) > 1:
        for t in sorted(set(tag)):
            sel = torch.as_tensor(TAGS == t, device=dev).unsqueeze(-1) & Mte
            line += f" | {t.replace('dr1.5_', '').replace('.npz', '').replace('v2_', '')} {r2(P[sel].cpu().numpy(), Yte[sel].cpu().numpy()):.3f}"
            if SIG is not None:
                line += f" (sig {SIG[sel].mean().item():.3f})"
    print(line, flush=True)
    net.train()

opt = torch.optim.Adam(net.parameters(), lr=a.lr, weight_decay=1e-4)
sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
B = 256
for ep in range(a.epochs):
    order = np.random.permutation(len(train))
    tot, cnt = 0.0, 0
    for i in range(0, len(order), B):
        X, Y, M = batchify([train[j] for j in order[i:i + B]])
        opt.zero_grad()
        loss = loss_fn(net(X), Y, M)
        loss.backward()
        nn.utils.clip_grad_norm_(net.parameters(), 1.0)
        opt.step()
        tot += loss.item() * M.sum().item(); cnt += M.sum().item()
    sch.step()
    if ep % 25 == 0 or ep == a.epochs - 1:
        print(f"epoch {ep:4d}  train mse {tot / cnt:.4f}", flush=True)
    if (ep + 1) % a.report_every == 0 or ep == a.epochs - 1:
        report(ep + 1)
net.eval()




class Exported(nn.Module):
    """Raw observation of ONE step + hidden state -> (drift in metres, new state)."""

    recurrent: bool
    hidden_size: int
    nll: bool
    s0sq: float
    relative: bool
    p: float
    ref_var: float

    def __init__(self, gru, head, mu, sd):
        super().__init__()
        self.cell = nn.GRUCell(gru.input_size, gru.hidden_size)
        with torch.no_grad():                       # single-layer GRU == GRUCell layout
            self.cell.weight_ih.copy_(gru.weight_ih_l0)
            self.cell.weight_hh.copy_(gru.weight_hh_l0)
            self.cell.bias_ih.copy_(gru.bias_ih_l0)
            self.cell.bias_hh.copy_(gru.bias_hh_l0)
        self.head = head
        self.register_buffer("mu", torch.as_tensor(mu))
        self.register_buffer("sd", torch.as_tensor(sd))
        self.recurrent = True
        self.hidden_size = int(gru.hidden_size)
        self.nll = bool(a.nll)
        self.s0sq = float(a.sigma0) ** 2
        self.relative = a.shrink == "relative"
        self.p = float(a.shrink_p)
        self.ref_var = float(REF_SIGMA) ** 2

    def forward(self, x, h):
        h = self.cell((x - self.mu) / self.sd, h)
        out = self.head(h)
        if self.nll:
            mu_, lv = out[:, :2], out[:, 2:].clamp(-6.0, 4.0)
            var = lv.exp().mean(-1, keepdim=True)
            if self.relative:
                gain = torch.clamp((self.ref_var / var) ** (0.5 * self.p), max=1.0)
            else:
                gain = self.s0sq / (self.s0sq + var)
            return mu_ * gain, h
        return out, h


REF_SIGMA = float(a.sigma0)
if a.nll and TAGS is not None:
    net.eval()
    with torch.no_grad():
        _, lv = split(net(Xte))
        steady = torch.as_tensor(np.array([a.steady_tag in t for t in tag]), device=dev).unsqueeze(-1) & Mte
        REF_SIGMA = (0.5 * lv).exp().mean(-1)[steady].mean().item()
    print(f"[shrink] steady-wind held-out sigma_ref = {REF_SIGMA:.3f} m  (mode {a.shrink}, p {a.shrink_p})")

if a.export:
    mod = Exported(net.gru.cpu(), net.head.cpu(), mu, sd).eval()
    # sanity: stepping the cell reproduces the sequence model on one held-out episode
    o, _, _ = test[0]
    with torch.no_grad():
        seq = net.cpu()(torch.as_tensor((o - mu) / sd).unsqueeze(0))[0]
        if a.nll:
            m_, lv = split(seq); var = lv.exp().mean(-1, keepdim=True)
            seq = m_ * (torch.clamp((mod.ref_var / var) ** (0.5 * mod.p), max=1.0) if mod.relative
                        else mod.s0sq / (mod.s0sq + var))
        h = torch.zeros(1, mod.hidden_size)
        step = []
        for t in range(len(o)):
            d, h = mod(torch.as_tensor(o[t:t + 1]), h)
            step.append(d)
        assert torch.allclose(torch.cat(step), seq, atol=1e-4), "GRUCell export diverged from nn.GRU"
    torch.jit.script(mod).save(a.export)
    print(f"[export] {a.export}  in={Xtr.shape[-1]}  hidden={mod.hidden_size}  out=2 (metres)  recurrent=True  nll={a.nll} sigma0={a.sigma0}")
