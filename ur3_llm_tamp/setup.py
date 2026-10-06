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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuan',
    maintainer_email='tuan@todo.todo',
    description='UR3 LLM TAMP Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception = ur3_llm_tamp.perception_node:main',
            'skills = ur3_llm_tamp.robot_skills:main'
        ],
    },
)
