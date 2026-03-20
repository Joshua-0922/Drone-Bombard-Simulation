from setuptools import find_packages, setup

package_name = 'rl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/hyperparams.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rl_navigation_node = rl_navigation.rl_navigation_node:main',
            'simple_drop        = rl_navigation.simple_drop_node:main',
            'train_sac        = rl_navigation.train_sac:main',
            'evaluate         = rl_navigation.evaluate:main',
            'tune_optuna      = rl_navigation.tune_optuna:main',
        ],
    },
)
