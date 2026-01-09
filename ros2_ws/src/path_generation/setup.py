from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_generation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # [추가] Launch 파일 설치 경로 지정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # [추가] Config 파일 설치 경로 지정
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'generator = path_generation.path_generation_node:main',
        ],
    },
)
