#!/usr/bin/env python3
"""Generate cube / zone SDF models (run once; outputs are committed)."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

CUBES = {
    'red_cube': '1 0 0 1',
    'yellow_cube': '1 1 0 1',
    'blue_cube': '0 0 1 1',
    'green_cube': '0 1 0 1',
    'purple_cube': '0.6 0 0.8 1',
}

CUBE_TMPL = """<?xml version="1.0" ?>
<!-- {name}: 4 cm cube, 50 g. Contact params tuned so the Robotiq 2F-85
     can hold it purely by friction (no pose teleport / no attach plugin). -->
<sdf version="1.6">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>0.005</mass>
        <inertia><ixx>1.33e-6</ixx><iyy>1.33e-6</iyy><izz>1.33e-6</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>0.04 0.04 0.04</size></box></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.04 0.04 0.04</size></box></geometry>
        <surface>
          <friction><ode><mu>100.0</mu><mu2>100.0</mu2></ode></friction>
          <contact><ode>
            <kp>1e5</kp><kd>10</kd><max_vel>0.0</max_vel><min_depth>0.005</min_depth>
          </ode></contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""

ZONE_TMPL = """<?xml version="1.0" ?>
<!-- {name}: flat 9x9 cm white pad on the table (visual only, no collision). -->
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.09 0.09 0.002</size></box></geometry>
        <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

if __name__ == '__main__':
    for n, c in CUBES.items():
        with open(os.path.join(HERE, f'{n}.sdf'), 'w') as f:
            f.write(CUBE_TMPL.format(name=n, rgba=c))
    for z in ('zone_a', 'zone_b', 'zone_c'):
        with open(os.path.join(HERE, f'{z}.sdf'), 'w') as f:
            f.write(ZONE_TMPL.format(name=z))
    print('models generated')

