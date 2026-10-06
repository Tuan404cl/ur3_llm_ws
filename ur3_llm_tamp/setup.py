import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3_llm_tamp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuan',
    maintainer_email='quangtuanvinhlong2005@gmail.com',
    description='UR3 + Robotiq: LLM skill planning with camera perception (Bai 03)',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception = ur3_llm_tamp.perception_node:main',
            'llm_commander = ur3_llm_tamp.commander_node:main',
            'orchestrator = ur3_llm_tamp.master_orchestrator_node:main',
            'skills_test = ur3_llm_tamp.robot_skills:main',
        ],
    },
)
