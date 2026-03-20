"""Optuna hyperparameter optimization for SAC drone drop policy.

Searches SAC hyperparameters (learning_rate, buffer_size, batch_size,
gamma, tau, net_arch) while keeping reward weights fixed.

Usage:
    ros2 run rl_navigation tune_optuna [--config PATH] [--n-trials N]

Results are stored in a SQLite database for Spot VM resilience.
Each trial logs to a separate WandB run in the 'optuna-study' group.
"""

import argparse
import os
import sys
import time

import numpy as np
import optuna
from optuna.pruners import MedianPruner
import rclpy
import yaml
import wandb
from ament_index_python.packages import get_package_share_directory
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback

from rl_navigation.drone_drop_env import DroneDropEnv

_DEFAULT_CONFIG = os.path.join(
    get_package_share_directory('rl_navigation'), 'config', 'hyperparams.yaml')

_NET_ARCH_MAP = {
    'small': [128, 128],
    'medium': [256, 256],
    'large': [256, 256, 128],
}


class OptunaEvalCallback(BaseCallback):
    """Evaluate success rate periodically and report to Optuna for pruning."""

    def __init__(self, trial, eval_freq=5000):
        super().__init__()
        self.trial = trial
        self.eval_freq = eval_freq
        self.best_success_rate = 0.0
        self._ep_successes = []
        self._report_step = 0

    def _on_step(self):
        infos = self.locals.get('infos', [])
        dones = self.locals.get('dones', [])
        for info, done in zip(infos, dones):
            if done and 'is_success' in info:
                self._ep_successes.append(float(info['is_success']))

        if self.num_timesteps >= (self._report_step + 1) * self.eval_freq:
            self._report_step += 1
            if self._ep_successes:
                sr = np.mean(self._ep_successes[-50:])  # last 50 episodes
                self.best_success_rate = max(self.best_success_rate, sr)

                self.trial.report(sr, self._report_step)

                if wandb.run:
                    wandb.log({
                        'optuna/success_rate': sr,
                        'optuna/best_success_rate': self.best_success_rate,
                    }, step=self.num_timesteps)

                if self.trial.should_prune():
                    if wandb.run:
                        wandb.finish()
                    raise optuna.TrialPruned()
        return True


def _create_objective(config_path, cfg, cfg_optuna):
    """Create the Optuna objective function with closed-over config."""

    trial_budget = cfg_optuna.get('trial_budget', 50000)
    eval_freq = cfg_optuna.get('eval_freq', 5000)
    restart_every = cfg_optuna.get('restart_infra_every', 10)
    cfg_wandb = cfg.get('wandb', {})

    def objective(trial):
        # --- Sample SAC hyperparameters ---
        lr = trial.suggest_float('learning_rate', 1e-5, 1e-3, log=True)
        buffer_size = trial.suggest_categorical(
            'buffer_size', [50000, 100000, 200000])
        batch_size = trial.suggest_categorical(
            'batch_size', [128, 256, 512])
        gamma = trial.suggest_float('gamma', 0.95, 0.999)
        tau = trial.suggest_float('tau', 0.001, 0.02)
        net_arch_key = trial.suggest_categorical(
            'net_arch', ['small', 'medium', 'large'])
        net_arch = _NET_ARCH_MAP[net_arch_key]

        # --- WandB for this trial ---
        wandb.init(
            project=cfg_wandb.get('project', 'drone-bombard-sac'),
            entity=cfg_wandb.get('entity') or None,
            group='optuna-study',
            name=f'trial-{trial.number}',
            tags=['optuna'] + cfg_wandb.get('tags', []),
            config=trial.params,
            reinit=True,
        )

        # --- Environment ---
        # Restart infra periodically to prevent Gazebo memory leaks
        keep_infra = (trial.number % restart_every != 0)
        env = DroneDropEnv(config_path=config_path, instance_id=0)

        try:
            # --- SAC model ---
            model = SAC(
                'MlpPolicy',
                env,
                learning_rate=lr,
                buffer_size=buffer_size,
                batch_size=batch_size,
                tau=tau,
                gamma=gamma,
                learning_starts=min(1000, trial_budget // 10),
                policy_kwargs=dict(net_arch=net_arch),
                device=cfg.get('sac', {}).get('device', 'cuda'),
                tensorboard_log=None,
                verbose=0,
            )

            # --- Train with pruning callback ---
            eval_cb = OptunaEvalCallback(trial, eval_freq=eval_freq)
            model.learn(
                total_timesteps=trial_budget,
                callback=eval_cb,
            )

            result = eval_cb.best_success_rate

        except optuna.TrialPruned:
            raise
        except Exception as e:
            print(f'Trial {trial.number} failed: {e}')
            result = 0.0
        finally:
            env.close(keep_infra=keep_infra)
            if wandb.run:
                wandb.finish()

        return result

    return objective


def _parse_args():
    parser = argparse.ArgumentParser(description='Optuna HPO for SAC')
    parser.add_argument(
        '--config', type=str, default=_DEFAULT_CONFIG,
        help='Path to hyperparams.yaml')
    parser.add_argument(
        '--n-trials', type=int, default=None,
        help='Override number of trials from config')
    parser.add_argument(
        '--trial-budget', type=int, default=None,
        help='Override training steps per trial')
    return parser.parse_args()


def main(args=None):
    """Entry point: ros2 run rl_navigation tune_optuna."""
    cli = _parse_args()

    config_path = os.path.realpath(cli.config)
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    cfg_optuna = cfg.get('optuna', {})
    n_trials = cli.n_trials or cfg_optuna.get('n_trials', 50)
    if cli.trial_budget:
        cfg_optuna['trial_budget'] = cli.trial_budget
    storage = cfg_optuna.get('storage', 'sqlite:///optuna_drone.db')

    print('=== Optuna SAC Hyperparameter Optimization ===')
    print(f'  Config      : {config_path}')
    print(f'  Trials      : {n_trials}')
    print(f'  Budget/trial: {cfg_optuna.get("trial_budget", 50000):,} steps')
    print(f'  Storage     : {storage}')

    study = optuna.create_study(
        study_name='drone-bombard-sac',
        direction='maximize',
        pruner=MedianPruner(
            n_startup_trials=3,
            n_warmup_steps=5,
        ),
        storage=storage,
        load_if_exists=True,  # resume after Spot VM preemption
    )

    objective = _create_objective(config_path, cfg, cfg_optuna)
    study.optimize(objective, n_trials=n_trials)

    # --- Report best parameters ---
    print('\n=== Best Trial ===')
    print(f'  Number       : {study.best_trial.number}')
    print(f'  Success rate : {study.best_value:.4f}')
    print(f'  Parameters   :')
    for key, val in study.best_params.items():
        print(f'    {key}: {val}')

    # --- Save best params to YAML ---
    best_yaml_path = os.path.join(
        os.path.dirname(config_path), 'best_hyperparams.yaml')
    best_params = study.best_params.copy()
    # Convert net_arch key to actual list
    if 'net_arch' in best_params:
        best_params['net_arch'] = _NET_ARCH_MAP[best_params['net_arch']]
    with open(best_yaml_path, 'w') as f:
        yaml.dump({'sac': best_params}, f, default_flow_style=False)
    print(f'\nBest params saved to {best_yaml_path}')

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
