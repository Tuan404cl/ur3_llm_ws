"""Gazebo simulation: UR3 + Robotiq 2F-85 + overhead camera + table + 3 zones + 5 cubes.

Arguments:
  gui:=true|false          chạy gzclient (GUI) hay không
  scenario:=<name>         file trong share/ur3_llm_tamp/scenarios/<name>.yaml
"""
import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [-1.57, -1.57, 1.57, -1.57, -1.57, 0.0]

def _spawn_objects(context, pkg_path):
    """Đọc scenario YAML và tạo các node spawn zone + cube."""
    scenario = LaunchConfiguration('scenario').perform(context)
    scen_file = scenario if scenario.endswith('.yaml') else os.path.join(
        pkg_path, 'scenarios', f'{scenario}.yaml')
    with open(scen_file, 'r') as f:
        scen = yaml.safe_load(f)
    with open(os.path.join(pkg_path, 'config', 'scene.yaml'), 'r') as f:
        scene = yaml.safe_load(f)

    table_z = float(scene['table']['z'])
    nodes = []
    for zname, z in scene['zones'].items():
        nodes.append(Node(
            package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{zname}', output='log',
            arguments=['-entity', zname, '-file', os.path.join(pkg_path, 'models', f'{zname}.sdf'),
                       '-x', str(z['x']), '-y', str(z['y']), '-z', str(table_z + 0.001)]))
    half = float(scene['cube_size']) / 2.0
    for name, (x, y, yaw) in scen['objects'].items():
        nodes.append(Node(
            package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{name}', output='log',
            arguments=['-entity', name, '-file', os.path.join(pkg_path, 'models', f'{name}.sdf'),
                       '-x', str(x), '-y', str(y), '-z', str(table_z + half + 0.003),
                       '-Y', str(yaw)]))
    return nodes

def generate_launch_description():
    pkg_path = get_package_share_directory('ur3_llm_tamp')
    world_file = os.path.join(pkg_path, 'worlds', 'tamp_env.world')

    gazebo_model_path = os.path.pathsep.join([
        os.path.join(get_package_share_directory('ur_description'), '..'),
        os.path.join(get_package_share_directory('robotiq_description'), '..'),
        os.path.join(pkg_path, 'models'),
        '/opt/ros/humble/share',
        os.path.join(get_package_share_directory('ur3_llm_tamp'), '..'),
    ])
    gazebo_resource_path = os.path.pathsep.join([
        '/usr/share/gazebo-11',
        '/opt/ros/humble/share',
        os.path.join(get_package_share_directory('robotiq_description'), '..'),
        pkg_path,
    ])
    gazebo_plugin_path = os.path.pathsep.join([
        os.path.join(get_package_share_directory('gazebo_grasp_plugin'), '..', '..', 'lib'),
        '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
        '/opt/ros/humble/lib',
    ])

    gui = LaunchConfiguration('gui')

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_state.so'],
        output='screen')
    gzclient = ExecuteProcess(cmd=['gzclient'], output='log', condition=IfCondition(gui))

    raw_urdf = xacro.process_file(os.path.join(pkg_path, 'urdf', 'ur3_robotiq.xacro')).toxml()
    ur_share = get_package_share_directory('ur_description')
    robotiq_share = get_package_share_directory('robotiq_description')
    resolved_urdf = raw_urdf.replace('package://ur_description/', f'file://{ur_share}/')
    resolved_urdf = resolved_urdf.replace('package://robotiq_description/', f'file://{robotiq_share}/')

    robot_description = {'robot_description': resolved_urdf}

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='log', parameters=[robot_description, {'use_sim_time': True}])

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_ur3', output='screen',
                       arguments=['-topic', 'robot_description', '-entity', 'ur3_tamp', '-package_to_model'])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager-timeout', '60'])

    jsb = TimerAction(period=6.0, actions=[spawner('joint_state_broadcaster')])
    jtc = TimerAction(period=8.0, actions=[spawner('joint_trajectory_controller')])
    gac = TimerAction(period=10.0, actions=[spawner('gripper_action_controller')])

    traj = ('{trajectory: {joint_names: [' + ', '.join(ARM_JOINTS) + '], points: [{positions: ['
            + ', '.join(str(v) for v in HOME) + '], time_from_start: {sec: 2}}]}}')
    hold_home = TimerAction(period=11.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '/joint_trajectory_controller/follow_joint_trajectory',
                 'control_msgs/action/FollowJointTrajectory', traj], output='log')
    ])

    hold_gripper = TimerAction(period=11.5, actions=[
        ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '/gripper_action_controller/gripper_cmd',
                 'control_msgs/action/GripperCommand',
                 '{command: {position: 0.0, max_effort: 50.0}}'], output='log')
    ])

    # Spawn camera-visible objects sau khi hệ thống ổn định
    spawn_objects = TimerAction(period=5.0, actions=[
        OpaqueFunction(function=_spawn_objects, args=[pkg_path])
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='demo_occupied'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        gzserver, gzclient, rsp,
        TimerAction(period=3.0, actions=[spawn_robot]),
        jsb, jtc, gac, hold_home, hold_gripper, spawn_objects
    ])