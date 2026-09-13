"""Unit tests for drone_bombard/residual_actor.py (L1-RL: frozen nominal + residual trunk).

Runs WITHOUT isaaclab / rsl_rl: the module is pure torch and is loaded by file
path so ``drone_bombard/__init__.py`` (which imports isaaclab) is never touched.

What is checked, and why it matters for the L1-RL vs L1-SL comparison:
  * zero-init -> iteration 0 IS the nominal policy (delta = 0)
  * training the residual never moves the nominal rows (attribution)
  * a fake "loaded Adam state" cannot move them either -- the defect the old
    --freeze_nominal path had (masked grad is a zero tensor, not None)
  * folding res_gi.pt reproduces play.py's --sl_residual injection to 1e-6
  * the widened critic is the old critic on the old channels

Run: pytest isaac_lab/tests/test_residual_actor.py -v
"""

import importlib.util
import os

import pytest
import torch
import torch.nn as nn

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "residual_actor", os.path.join(_HERE, "..", "drone_bombard", "residual_actor.py"))
ra = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ra)

N_NOM, N_OBS, ACT = 26, 38, 7


def _nominal():
    torch.manual_seed(0)
    return nn.Sequential(nn.Linear(N_NOM, 32), nn.ELU(), nn.Linear(32, 32), nn.ELU(),
                         nn.Linear(32, ACT))


def test_zero_init_is_the_nominal_policy():
    nom = _nominal()
    actor = ra.ResidualActor(nom, N_NOM, N_OBS, hidden=(16, 16))
    x = torch.randn(8, N_OBS)
    out = actor(x)
    assert torch.equal(out[:, :5], nom(x[:, :N_NOM])[:, :5])
    assert torch.all(out[:, 5:7] == 0.0)


def test_training_the_residual_leaves_the_nominal_bit_identical():
    nom = _nominal()
    actor = ra.ResidualActor(nom, N_NOM, N_OBS, hidden=(16, 16))
    x = torch.randn(8, N_OBS)
    before = actor(x)[:, :5].clone()
    nom_state = {k: v.clone() for k, v in nom.state_dict().items()}
    opt = torch.optim.Adam([p for p in actor.parameters() if p.requires_grad], lr=0.1)
    target = torch.ones(8, ACT)
    for _ in range(5):
        opt.zero_grad()
        # A regression loss, not out**2: at the zero-init point the gradient of
        # out**2 through the residual is itself zero (PPO's surrogate is not).
        (actor(x) - target).pow(2).sum().backward()   # touches every output row
        opt.step()
    assert torch.equal(actor(x)[:, :5], before)
    for k, v in nom.state_dict().items():
        assert torch.equal(v, nom_state[k])
    assert torch.any(actor(x)[:, 5:7] != 0.0), "the residual rows must learn"


def test_masked_gradient_alone_does_not_protect_from_adam_momentum():
    """Documents WHY attach_frozen_nominal rebuilds the optimizer instead of
    calling runner.load: a zero (not None) gradient plus a restored Adam state
    still moves the parameter."""
    p = nn.Parameter(torch.ones(3))
    opt = torch.optim.Adam([p], lr=0.1)
    (p * torch.tensor([1.0, 1.0, 1.0])).sum().backward()
    opt.step()                                    # now Adam has non-zero moments
    p.register_hook(lambda g: g * 0.0)            # what --freeze_nominal does
    before = p.detach().clone()
    opt.zero_grad()
    (p * 3.0).sum().backward()
    assert torch.all(p.grad == 0.0)
    opt.step()
    assert not torch.equal(p.detach(), before), "masked grad still moved: momentum leak"


def test_mlp_from_state_dict_roundtrip():
    nom = _nominal()
    sd = {f"actor.{k}": v for k, v in nom.state_dict().items()}
    sd["critic.0.weight"] = torch.zeros(1, 1)     # unrelated keys are ignored
    rebuilt = ra.mlp_from_state_dict(sd, "actor.")
    x = torch.randn(4, N_NOM)
    assert torch.equal(rebuilt(x), nom(x))


class _Exported(nn.Module):
    """Same shape _fit_sl_residual.py exports: net((x - mu) / sd) -> metres."""

    def __init__(self, net, mu, sd):
        super().__init__()
        self.net = net
        self.register_buffer("mu", mu)
        self.register_buffer("sd", sd)

    def forward(self, x):
        return self.net((x - self.mu) / self.sd)


def test_fold_sl_regressor_matches_play_py_injection(tmp_path):
    torch.manual_seed(1)
    net = nn.Sequential(nn.Linear(N_OBS, 128), nn.ELU(), nn.Linear(128, 128), nn.ELU(),
                        nn.Linear(128, 2))
    mu, sd = torch.randn(N_OBS), torch.rand(N_OBS) + 0.5
    path = str(tmp_path / "res.pt")
    torch.jit.script(_Exported(net, mu, sd)).save(path)

    scale = 2.0
    actor = ra.ResidualActor(_nominal(), N_NOM, N_OBS)     # default hidden = (128, 128)
    ra.fold_sl_regressor(actor.residual, path, scale)
    x = torch.randn(16, N_OBS)
    want = torch.jit.load(path)(x) / scale                  # play.py: sl(obs) / residual.scale
    assert torch.allclose(actor(x)[:, 5:7], want, atol=1e-5)


def test_fold_rejects_a_shape_mismatch(tmp_path):
    net = nn.Sequential(nn.Linear(N_OBS, 64), nn.ELU(), nn.Linear(64, 2))
    path = str(tmp_path / "bad.pt")
    torch.jit.script(_Exported(net, torch.zeros(N_OBS), torch.ones(N_OBS))).save(path)
    actor = ra.ResidualActor(_nominal(), N_NOM, N_OBS)
    with pytest.raises(ValueError):
        ra.fold_sl_regressor(actor.residual, path, 1.0)


def test_pad_first_linear_keeps_the_old_function():
    critic = nn.Sequential(nn.Linear(N_NOM, 16), nn.ELU(), nn.Linear(16, 1))
    sd = {f"critic.{k}": v for k, v in critic.state_dict().items()}
    wide = nn.Sequential(nn.Linear(N_OBS, 16), nn.ELU(), nn.Linear(16, 1))
    wide.load_state_dict(ra._pad_first_linear(sd, "critic.", N_OBS))
    x = torch.randn(4, N_OBS)
    assert torch.allclose(wide(x), critic(x[:, :N_NOM]))


def test_attach_from_checkpoint_rebuilds_the_same_actor():
    nom = _nominal()
    actor = ra.ResidualActor(nom, N_NOM, N_OBS, hidden=(24, 12))
    with torch.no_grad():
        actor.residual[-1].weight.normal_()
    sd = {f"actor.{k}": v for k, v in actor.state_dict().items()}
    policy = nn.Module()
    policy.actor = nn.Linear(N_OBS, ACT)      # stand-in for rsl_rl's fresh MLP
    ra.attach_from_checkpoint(policy, sd)
    policy.load_state_dict({k: v for k, v in sd.items()}, strict=True)
    x = torch.randn(4, N_OBS)
    assert torch.equal(policy.actor(x), actor(x))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
