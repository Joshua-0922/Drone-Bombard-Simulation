"""L1-RL actor: a FROZEN nominal policy plus a SEPARATE residual trunk.

Standard residual RL (Johannink et al. 2019): the nominal controller is fixed
and a second network adds a correction. Here the nominal is the L0 checkpoint
(26 obs -> 7 actions, of which only rows 0:5 -- velocity + drop signal -- are
used) and the residual is its own MLP over the full observation, including the
accumulator channels the env appends under ``accum_obs`` (38 wide). Its output
lands on action rows 5:7, the impact-space residual.

Why not ``train.py --freeze_nominal``: that recipe trains only the residual ROWS
of L0's output layer, so the residual is a linear read-out of features that
were trained for flight, not for wind -- and it cannot see any observation
channel L0 did not see, because the first layer is frozen too. A residual with
its own trunk removes both limits and keeps the attribution exact: the nominal
weights never change, so "L1-RL minus L0" is the residual and nothing else.

The residual MLP has the SAME shape as the supervised regressor
(``_fit_sl_residual.py``: in -> 128 -> 128 -> 2, ELU), so it can be initialised
from ``res_gi.pt`` (``fold_sl_regressor``). That arm starts exactly at L1-SL and
asks the one question left for RL: does the terminal reward add anything on
top of drift prediction?

Pure torch -- no rsl_rl / isaaclab import -- so ``tests/test_residual_actor.py``
runs on the dev box. The rsl_rl glue (which module to replace, optimizer
rebuild) is in ``attach_frozen_nominal`` / ``attach_from_checkpoint`` and only
touches attributes that rsl_rl 3.1.2 exposes (``alg.policy.{actor,critic,std}``,
``alg.optimizer``, ``alg.learning_rate``).
"""

import torch
import torch.nn as nn

NOMINAL_DIMS = 5
"""Action rows the nominal owns: [0:4] velocity + yaw rate, [4] drop signal."""
RESIDUAL_DIMS = slice(5, 7)
"""Action rows the residual owns (task_env._pre_physics_step)."""
RESIDUAL_HIDDEN = (128, 128)
"""Matches _fit_sl_residual.py's regressor so res_gi.pt can initialise it."""


class ResidualActor(nn.Module):
    def __init__(self, nominal: nn.Module, n_nominal_obs: int, n_obs: int,
                 hidden=RESIDUAL_HIDDEN):
        super().__init__()
        self.nominal = nominal
        for p in self.nominal.parameters():
            p.requires_grad_(False)
        self.n_nominal_obs = int(n_nominal_obs)
        layers, d = [], int(n_obs)
        for h in hidden:
            layers += [nn.Linear(d, h), nn.ELU()]
            d = h
        layers.append(nn.Linear(d, RESIDUAL_DIMS.stop - RESIDUAL_DIMS.start))
        self.residual = nn.Sequential(*layers)
        # Zero-init the output layer: iteration 0 behaves exactly like L0
        # (delta = 0). A random residual moves the aim by up to residual.scale
        # metres from the first step -- the 08-29 pilot's collapse.
        with torch.no_grad():
            self.residual[-1].weight.zero_()
            self.residual[-1].bias.zero_()

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        nom = self.nominal(obs[..., :self.n_nominal_obs])
        return torch.cat([nom[..., :NOMINAL_DIMS], self.residual(obs)], dim=-1)


class NarrowObs(nn.Module):
    """Run a policy trained on the first ``n`` channels inside a wider env
    (an L0 checkpoint flying under ``accum_obs`` while a residual is injected).
    All 7 rows pass through unchanged, so the arm is exactly the old L0."""

    def __init__(self, inner: nn.Module, n: int):
        super().__init__()
        self.inner, self.n = inner, int(n)

    def forward(self, obs):
        return self.inner(obs[..., :self.n])


def mlp_from_state_dict(sd: dict, prefix: str) -> nn.Sequential:
    """Rebuild a Linear/ELU stack whose keys match rsl_rl's MLP ('0', '2', '4', ...)
    from a checkpoint state dict, so an L0 actor loads without importing rsl_rl."""
    idx = sorted(int(k[len(prefix):].split(".")[0])
                 for k in sd if k.startswith(prefix) and k.endswith(".weight"))
    if not idx:
        raise KeyError(f"no '{prefix}*.weight' keys in checkpoint")
    layers = []
    for i in idx:
        w = sd[f"{prefix}{i}.weight"]
        layers += [nn.Linear(w.shape[1], w.shape[0]), nn.ELU()]
    layers.pop()                       # linear last layer
    m = nn.Sequential(*layers)
    m.load_state_dict({k[len(prefix):]: v for k, v in sd.items() if k.startswith(prefix)})
    return m


def fold_sl_regressor(residual: nn.Sequential, path: str, scale: float) -> None:
    """Initialise the residual trunk from a TorchScript regressor exported by
    _fit_sl_residual.py (``net((x - mu) / sd)`` -> metres).

    The input normalisation is folded into the first layer and the metres
    output is divided by ``scale`` so the channel carries action units
    (``task_env._ccip`` multiplies by ``residual.scale``). Bit-for-bit this is
    what ``play.py --sl_residual`` injects, up to the +-1 clamp.
    """
    sd = torch.jit.load(path, map_location="cpu").state_dict()
    mu, s = sd["mu"], sd["sd"]
    lins = [m for m in residual if isinstance(m, nn.Linear)]
    keys = sorted({k.split(".")[1] for k in sd if k.startswith("net.")}, key=int)
    if len(keys) != len(lins):
        raise ValueError(f"{path}: {len(keys)} linear layers, residual has {len(lins)}")
    with torch.no_grad():
        for j, (lin, k) in enumerate(zip(lins, keys)):
            w, b = sd[f"net.{k}.weight"], sd[f"net.{k}.bias"]
            if tuple(w.shape) != tuple(lin.weight.shape):
                raise ValueError(f"{path}: layer {k} is {tuple(w.shape)}, residual layer is "
                                 f"{tuple(lin.weight.shape)}")
            if j == 0:
                w, b = w / s, b - w @ (mu / s)
            if j == len(lins) - 1:
                w, b = w / scale, b / scale
            lin.weight.copy_(w)
            lin.bias.copy_(b)


def _pad_first_linear(sd: dict, prefix: str, n_in: int) -> dict:
    """Zero-pad the first layer's input columns to ``n_in`` (new channels start
    with no influence, so the loaded function is unchanged on the old ones)."""
    out = {k[len(prefix):]: v for k, v in sd.items() if k.startswith(prefix)}
    w = out["0.weight"]
    if w.shape[1] < n_in:
        out["0.weight"] = torch.cat([w, torch.zeros(w.shape[0], n_in - w.shape[1],
                                                    dtype=w.dtype, device=w.device)], dim=1)
    return out


def _first_linear(m: nn.Module) -> nn.Linear:
    for sub in m.modules():
        if isinstance(sub, nn.Linear):
            return sub
    raise RuntimeError("no nn.Linear found")


def attach_frozen_nominal(runner, ckpt_path: str, init_std: float,
                          init_from: str | None, scale: float,
                          nominal_std: float | None = None, fixed_std: bool = False) -> None:
    """train.py --residual_net: replace the runner's actor with L0-frozen +
    residual trunk, warm-start the critic (input widened), rebuild the optimizer.

    ``runner.load`` is deliberately NOT used: it would also restore L0's Adam
    moments, and Adam keeps stepping a parameter whose gradient is masked to
    zero (only ``grad is None`` is skipped) -- the frozen rows would drift.
    """
    device = runner.device
    sd = torch.load(ckpt_path, map_location=device, weights_only=False)["model_state_dict"]
    policy = runner.alg.policy
    n_obs = _first_linear(policy.actor).in_features

    nominal = mlp_from_state_dict(sd, "actor.")
    actor = ResidualActor(nominal, _first_linear(nominal).in_features, n_obs)
    if init_from:
        fold_sl_regressor(actor.residual, init_from, scale)
    policy.actor = actor.to(device)
    # The critic keeps training (the return changes once the residual acts);
    # start it from L0's value function, blind to the new channels.
    policy.critic.load_state_dict(_pad_first_linear(sd, "critic.", n_obs))
    with torch.no_grad():
        policy.std.copy_(sd["std"])
        if nominal_std is not None:
            # The nominal rows are frozen, so their std only sets how noisily
            # the frozen policy FLIES during rollouts -- their log-prob terms are
            # identical under old and new policy and cancel in the PPO ratio.
            # L0 ended training at std ~3.1 (bang-bang under sampling); the
            # supervised arm's data came from the DETERMINISTIC L0, and so does
            # every evaluation. A small value puts the rollouts on that same
            # flight distribution. Must stay > 0 (Normal log-prob).
            policy.std[:NOMINAL_DIMS] = nominal_std
        policy.std[RESIDUAL_DIMS] = init_std
    if fixed_std:
        # exp_032: a learnable residual std collapses to ~0.01 within 80
        # iterations (the first-crossing gate punishes every bit of jitter) and
        # the mean then has no exploration left to learn from. Pin it -- the
        # parameter leaves the optimizer entirely, so nothing can move it.
        policy.std.requires_grad_(False)
    else:
        mask = torch.zeros_like(policy.std)
        mask[RESIDUAL_DIMS] = 1.0
        policy.std.register_hook(lambda g: g * mask)   # nominal exploration never trains

    trainable = [p for p in policy.parameters() if p.requires_grad]
    runner.alg.optimizer = torch.optim.Adam(trainable, lr=runner.alg.learning_rate)
    n_nom = sum(p.numel() for p in nominal.parameters())
    n_res = sum(p.numel() for p in actor.residual.parameters())
    print(f"[L1-RL] nominal frozen ({n_nom} params, {actor.n_nominal_obs} obs, rollout std "
          f"{policy.std[0].item():.3g}) + residual trunk ({n_res} params, {n_obs} obs, "
          f"{'init from ' + init_from if init_from else 'zero-init'}) | critic warm-started, "
          f"residual std {init_std}{' FIXED' if fixed_std else ' (learnable)'}, scale {scale} m, fresh Adam")


def attach_from_checkpoint(policy, sd: dict) -> None:
    """play.py: an L1-RL checkpoint carries ``actor.nominal.*`` / ``actor.residual.*``;
    give the policy the matching module so a strict ``load_state_dict`` fits."""
    nominal = mlp_from_state_dict(sd, "actor.nominal.")
    res_keys = sorted((k for k in sd if k.startswith("actor.residual.") and k.endswith(".weight")),
                      key=lambda k: int(k.split(".")[2]))
    hidden = tuple(sd[k].shape[0] for k in res_keys[:-1])
    n_obs = sd[res_keys[0]].shape[1]
    device = next(policy.parameters()).device
    policy.actor = ResidualActor(nominal, _first_linear(nominal).in_features, n_obs, hidden).to(device)
