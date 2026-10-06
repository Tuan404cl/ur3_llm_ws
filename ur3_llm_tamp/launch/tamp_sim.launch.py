import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('ur3_llm_tamp')
    world_file = os.path.join(pkg_path, 'worlds', 'tamp_env.world')

    # 1. Trỏ đường dẫn Mesh cho Gazebo (khắc phục hoàn toàn lỗi No mesh specified)
    gazebo_model_path = os.path.pathsep.join([
        os.path.join(get_package_share_directory('ur_description'), '..'),
        os.path.join(get_package_share_directory('robotiq_description'), '..'),
        os.path.join(pkg_path, 'models')
    ])
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)

    # 2. Khởi động Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 3. Nạp Robot Description
    xacro_file = os.path.join(pkg_path, 'urdf', 'ur3_robotiq.xacro')
    robot_description_doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_doc.toxml()}

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ur3_tamp'],
        output='screen'
    )

    # 4. Kích hoạt Controllers
    jsb = TimerAction(period=3.0, actions=[Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])])
    jtc = TimerAction(period=4.5, actions=[Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller'])])
    gac = TimerAction(period=6.0, actions=[Node(package='controller_manager', executable='spawner', arguments=['gripper_action_controller'])])

    # 4b. Sau khi controllers active, ngay lập tức hold pose home để arm không rơi
    # Pose home khớp với initial_value trong URDF (shoulder_lift=-1.57, elbow=1.57, wrist_1/2=-1.57)
    hold_home = TimerAction(period=7.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '/joint_trajectory_controller/follow_joint_trajectory',
                 'control_msgs/action/FollowJointTrajectory',
                 '{"trajectory": {"joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], "points": [{"positions": [0.0, -1.57, 1.57, -1.57, -1.57, 0.0], "time_from_start": {"sec": 2}}]}, "goal_time_tolerance": {"sec": 5}}'],
            output='screen'
        )
    ])

    # 4c. Hold gripper mở (position=0.0) ngay sau khi gripper controller active
    # để các joints mimic + left_knuckle không bị rơi theo trọng lực
    hold_gripper = TimerAction(period=6.5, actions=[
        ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '/gripper_action_controller/gripper_cmd',
                 'control_msgs/action/GripperCommand',
                 '{command: {position: 0.0, max_effort: 50.0}}'],
            output='screen'
        )
    ])

    # 5. Spawn 3 Zones (Mặt phẳng cao z = 0.801m)
    def spawn_zone(name, x, y):
        sdf = os.path.join(pkg_path, 'models', f'{name}.sdf')
        return Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-entity', name, '-file', sdf, '-x', str(x), '-y', str(y), '-z', '0.801'], output='screen')

    zones = [
        spawn_zone('zone_a', 0.35, -0.20),
        spawn_zone('zone_b', 0.35, 0.00),
        spawn_zone('zone_c', 0.35, 0.20),
    ]

    # 6. Spawn 5 Blocks (Cao z = 0.825m)
    # blue_cube nằm trên zone_b để thử nghiệm tính năng phát hiện zone bị chiếm
    def spawn_block(name, x, y):
        sdf = os.path.join(pkg_path, 'models', f'{name}.sdf')
        return Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-entity', name, '-file', sdf, '-x', str(x), '-y', str(y), '-z', '0.825'], output='screen')

    blocks = [
        spawn_block('blue_cube', 0.35, 0.00),   # Nằm ngay trong zone_b
        spawn_block('red_cube', 0.22, -0.10),   # Trên bàn
        spawn_block('yellow_cube', 0.22, 0.10),  # Trên bàn
        spawn_block('green_cube', 0.45, -0.10),  # Trên bàn
        spawn_block('purple_cube', 0.45, 0.10),  # Trên bàn
    ]

    # Hẹn giờ 2s để Gazebo load xong trước khi spawn vật thể
    spawn_env_objects = TimerAction(period=2.0, actions=zones + blocks)

    return LaunchDescription([set_model_path, gazebo, rsp, spawn_robot, jsb, jtc, gac, hold_gripper, hold_home, spawn_env_objects])