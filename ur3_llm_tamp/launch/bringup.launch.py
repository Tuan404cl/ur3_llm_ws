"""Launch toàn bộ hệ thống nền (Gazebo + Controllers + MoveIt 2 + Perception).
Sau khi chạy file này, môi trường sẵn sàng 100% để bạn chạy lệnh test hoặc LLM Orchestrator ở 1 terminal riêng.

Sử dụng:
  ros2 launch ur3_llm_tamp bringup.launch.py
  (Tùy chọn: gui:=true/false, rviz:=true/false, scenario:=demo_occupied)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tamp = get_package_share_directory('ur3_llm_tamp')
    pkg_moveit = get_package_share_directory('ur3_gripper_moveit_config')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    scenario = LaunchConfiguration('scenario')

    # 1. Khởi động Gazebo + Spawner robot + controllers (tamp_sim.launch.py)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tamp, 'launch', 'tamp_sim.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'scenario': scenario,
        }.items()
    )

    # 2. Khởi động MoveIt 2 (move_group) sau 8 giây khi controllers đã sẵn sàng
    moveit_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_moveit, 'launch', 'ur3_moveit.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'launch_rviz': rviz,
                }.items()
            )
        ]
    )

    # 3. Khởi động Perception Node sau 12 giây khi Gazebo camera và các khối đã spawn xong
    perception_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ur3_llm_tamp',
                executable='perception',
                name='perception_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Chạy Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='false', description='Chạy RViz MoveIt'),
        DeclareLaunchArgument('scenario', default_value='demo_occupied', description='Kịch bản mô phỏng'),
        sim_launch,
        moveit_launch,
        perception_node
    ])

