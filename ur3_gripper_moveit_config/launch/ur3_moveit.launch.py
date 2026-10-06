# Launch MoveIt (move_group + RViz) cho robot ur3_gripper (UR3 + Robotiq 2F-85).
#
# Khác với ur_moveit.launch.py của ur_moveit_config, file này load:
#   - robot_description từ ur3_llm_tamp/urdf/ur3_robotiq.xacro (CÓ gripper)
#   - SRDF từ package này (config/ur3_gripper.srdf) -> khớp với URDF
#   - controllers khớp với ros2_control đang chạy trong Gazebo
#
# Cách dùng (Terminal 2, sau khi tamp_sim.launch.py đã chạy):
#   ros2 launch ur3_gripper_moveit_config ur3_moveit.launch.py

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    pkg_share = get_package_share_directory("ur3_gripper_moveit_config")

    # 1. robot_description - URDF đầy đủ (UR3 + gripper) từ ur3_llm_tamp
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur3_llm_tamp"), "urdf", "ur3_robotiq.xacro"]),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 2. robot_description_semantic - SRDF của package này (robot name: ur3_gripper)
    with open(os.path.join(pkg_share, "config", "ur3_gripper.srdf"), "r") as file:
        robot_description_semantic_content = file.read()
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    publish_robot_description_semantic = {"publish_robot_description_semantic": True}

    # 3. Kinematics (KDL cho group ur_manipulator) + joint limits
    robot_description_kinematics = os.path.join(pkg_share, "config", "kinematics.yaml")
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "ur3_gripper_moveit_config", "config/joint_limits.yaml"
        )
    }

    # 4. OMPL planning pipeline
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur3_gripper_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # 5. Trajectory execution - map sang controllers đang chạy trong Gazebo
    controllers_yaml = load_yaml("ur3_gripper_moveit_config", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz với config MoveIt (Fixed Frame: world)
    rviz_config_file = os.path.join(pkg_share, "rviz", "view_robot.rviz")
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Dùng /clock của Gazebo (bắt buộc khi chạy cùng mô phỏng).",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Khởi động RViz cùng move_group.",
            ),
            move_group_node,
            rviz_node,
        ]
    )
