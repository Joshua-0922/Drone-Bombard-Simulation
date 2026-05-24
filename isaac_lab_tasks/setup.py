"""Isaac Lab 드론 폭격 태스크 패키지 설치 스크립트.

설치:
  pip install -e .

필수 의존성 (Docker 이미지에 사전 포함):
  - isaaclab          : pip install isaac-lab
  - stable-baselines3 : pip install stable-baselines3[extra]
  - wandb             : pip install wandb
  - gymnasium         : pip install gymnasium
"""

from setuptools import find_packages, setup

setup(
    name="drone_bombard_tasks",
    version="0.1.0",
    description="Drone Bombard Simulation — Isaac Lab RL Tasks",
    author="Drone Bombard Team",
    python_requires=">=3.10",
    packages=find_packages(),
    install_requires=[
        "stable-baselines3>=2.0",
        "wandb>=0.16",
        "gymnasium>=0.29",
    ],
    # isaaclab / isaaclab_rl 은 Isaac Sim Docker 이미지에 사전 설치 (Dockerfile.isaac)
    extras_require={
        "dev": [
            "black",
            "isort",
            "flake8",
        ]
    },
)
