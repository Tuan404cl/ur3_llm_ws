import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ur3_llm = get_package_share_directory('ur3_llm_tamp')
    world_file = os.path.join(pkg_ur3_llm, 'worlds', 'tabletop.world')

    # Khởi động Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Trình spawn vật thể (Ví dụ spawn red_cube)
    red_cube_path = os.path.join(pkg_ur3_llm, 'models', 'red_cube.sdf')
    spawn_red = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'red_cube', '-file', red_cube_path, '-x', '0.3', '-y', '0.2', '-z', '0.85'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_red
    ])
