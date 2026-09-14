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
a = p.parse_args()
torch.manual_seed(a.seed)
np.random.seed(a.seed)
dev = "cuda" if torch.cuda.is_available() else "cpu"


def episodes(f):
    """Split one dump (T, N, ...) into per-episode sequences."""
    d = np.load(f)
    obs, drift, ep_len, att = d["obs"], d["drift"], d["ep_len"], d["attached"]
    T, N = ep_len.shape
    new = np.ones((T, N), bool)
    new[1:] = ep_len[1:] <= ep_len[:-1]
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
    def __init__(self, n_in, h):
        super().__init__()
        self.gru = nn.GRU(n_in, h, batch_first=True)
        self.head = nn.Linear(h, 2)

    def forward(self, x):
        y, _ = self.gru(x)
        return self.head(y)


net = Filter(Xtr.shape[-1], a.hidden).to(dev)
opt = torch.optim.Adam(net.parameters(), lr=a.lr, weight_decay=1e-4)
sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
B = 256
for ep in range(a.epochs):
    order = np.random.permutation(len(train))
    tot, cnt = 0.0, 0
    for i in range(0, len(order), B):
        X, Y, M = batchify([train[j] for j in order[i:i + B]])
        opt.zero_grad()
        err = ((net(X) - Y) ** 2).sum(-1)
        loss = (err * M).sum() / M.sum()
        loss.backward()
        nn.utils.clip_grad_norm_(net.parameters(), 1.0)
        opt.step()
        tot += loss.item() * M.sum().item(); cnt += M.sum().item()
    sch.step()
    if ep % 25 == 0 or ep == a.epochs - 1:
        print(f"epoch {ep:4d}  train mse {tot / cnt:.4f}")


def r2(pred, y):
    return 1 - ((pred - y) ** 2).sum() / ((y - y.mean(0)) ** 2).sum()


net.eval()
with torch.no_grad():
    X, Y, M = batchify(test)
    P = net(X)
    # frames the payload is carried, and the last 10 carried frames (the gate's frames)
    last = torch.zeros_like(M)
    for i, (_, _, m) in enumerate(test):
        idx = np.flatnonzero(m)
        last[i, idx[-10:]] = True
    for name, mask in (("carried", M), ("release(last10)", last)):
        pr, yy = P[mask].cpu().numpy(), Y[mask].cpu().numpy()
        jit = np.abs(np.diff(P.cpu().numpy(), axis=1)).sum(-1)[M[:, 1:].cpu().numpy()].mean()
        print(f"held-out R2 {name:16s} {r2(pr, yy):.3f}   |drift| {np.linalg.norm(yy, axis=1).mean():.3f} m   "
              f"pred step change {jit:.3f} m/step")
    tags = np.array(tag) if tag else None
    if tags is not None and len(set(tag)) > 1:
        for t in sorted(set(tag)):
            sel = torch.as_tensor(tags == t, device=dev).unsqueeze(-1) & M
            print(f"  per file {t:22s} R2 carried {r2(P[sel].cpu().numpy(), Y[sel].cpu().numpy()):.3f}")


class Exported(nn.Module):
    """Raw observation of ONE step + hidden state -> (drift in metres, new state)."""

    recurrent: bool
    hidden_size: int

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

    def forward(self, x, h):
        h = self.cell((x - self.mu) / self.sd, h)
        return self.head(h), h


if a.export:
    mod = Exported(net.gru.cpu(), net.head.cpu(), mu, sd).eval()
    # sanity: stepping the cell reproduces the sequence model on one held-out episode
    o, _, _ = test[0]
    with torch.no_grad():
        seq = net.cpu()(torch.as_tensor((o - mu) / sd).unsqueeze(0))[0]
        h = torch.zeros(1, mod.hidden_size)
        step = []
        for t in range(len(o)):
            d, h = mod(torch.as_tensor(o[t:t + 1]), h)
            step.append(d)
        assert torch.allclose(torch.cat(step), seq, atol=1e-4), "GRUCell export diverged from nn.GRU"
    torch.jit.script(mod).save(a.export)
    print(f"[export] {a.export}  in={Xtr.shape[-1]}  hidden={mod.hidden_size}  out=2 (metres)  recurrent=True")
