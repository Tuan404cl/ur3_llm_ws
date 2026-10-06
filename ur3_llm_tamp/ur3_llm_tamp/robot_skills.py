#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import math
import subprocess
import json
import sys
from typing import Optional, List, Tuple

from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

class UR3Skills(Node):
    def __init__(self, node_name='ur3_skills'):
        super().__init__(node_name)
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_action_controller/gripper_cmd')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.vacuum_client = self.create_client(SetBool, '/ur3_gripper/vacuum_switch')

        self.get_logger().info("Connecting to Action Clients and MoveIt 2 IK service...")
        self.gripper_client.wait_for_server()
        self.traj_client.wait_for_server()
        self.ik_client.wait_for_service()
        self.vacuum_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("Robot Skills interface ready!")

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        # Xoay toàn bộ robot sang 90 độ (shoulder_pan = -1.57 rad) ở Home
        # để gập sang một bên, hoàn toàn không che khuất tầm nhìn camera thẳng đứng xuống mặt bàn
        self.home_pose = [-1.57, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.current_joints = list(self.home_pose)
        self.holding_object = None
        self.vacuum_grasped = False
        self.latest_scene_dict = None

        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self._joint_callback, 10
        )
        self.sub_vacuum = self.create_subscription(
            Bool, '/ur3_gripper/vacuum_grasping', self._vacuum_callback, 10
        )
        self.sub_scene = self.create_subscription(
            String, '/scene_state', self._scene_callback, 10
        )

    def _scene_callback(self, msg: String):
        try:
            self.latest_scene_dict = json.loads(msg.data)
        except Exception:
            pass

    def _vacuum_callback(self, msg):
        if hasattr(msg, 'data'):
            self.vacuum_grasped = bool(msg.data)

    def _joint_callback(self, msg: JointState):
        name_map = dict(zip(msg.name, msg.position))
        pos = [name_map.get(name) for name in self.joint_names]
        if all(p is not None and not math.isnan(p) for p in pos):
            self.current_joints = pos

    def set_vacuum(self, state: bool):
        if self.vacuum_client.service_is_ready():
            req = SetBool.Request()
            req.data = state
            future = self.vacuum_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not state:
            self.vacuum_grasped = False

    def control_gripper(self, position: float, max_effort: float = 60.0, slow_steps: int = 5):
        """
        position: 0.0 (open, ~85mm), 0.8 (closed)
        Đóng mở từ từ qua slow_steps bước để chuyển động mượt mà, không giật mạnh.
        """
        # Nếu đang đóng ngón vào (position > 0.1), chia nhỏ thành các bước để khép êm
        if position > 0.1 and slow_steps > 1:
            for s in range(1, slow_steps + 1):
                inter_pos = (position / slow_steps) * s
                goal = GripperCommand.Goal()
                goal.command.position = float(inter_pos)
                goal.command.max_effort = float(max_effort)
                future = self.gripper_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.5)
                time.sleep(0.15) # Nghỉ nhẹ giữa các bước để ngón di chuyển êm ái
            time.sleep(0.5)
            return True
        else:
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = float(max_effort)
            self.get_logger().info(f"[Gripper] Command -> pos: {position}, max_effort: {max_effort}")
            future = self.gripper_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            time.sleep(0.5)
            return True

    def solve_ik(self, x: float, y: float, z: float, yaw: float = 0.0, retry_with_jitter: bool = True):
        """
        Solve IK for tool0 frame via MoveIt 2 /compute_ik service.
        x, y, z are in Gazebo 'world' frame.
        Base of UR3 is at world (0.0, 0.0, 0.80).
        So z_base = z_world - 0.80.
        tool0 orientation points straight down (Z_tool0 along -Z_world).
        Includes automatic retry with yaw jitter and slight height adjustment.
        """
        candidate_yaws = [yaw]
        if retry_with_jitter:
            # Jitter approach angles: +/- 15 deg, +/- 30 deg, +/- 45 deg, +/- 90 deg
            for delta in [0.2618, -0.2618, 0.5236, -0.5236, 0.7854, -0.7854, 1.5708, -1.5708]:
                candidate_yaws.append(yaw + delta)

        for candidate_yaw in candidate_yaws:
            sol = self._query_single_ik(x, y, z, candidate_yaw)
            if sol is not None:
                return sol

        self.get_logger().error(f"MoveIt IK failed at target ({x:.3f}, {y:.3f}, {z:.3f}) even after yaw jitter retries.")
        return None

    def _query_single_ik(self, x: float, y: float, z: float, yaw: float):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.ik_link_name = "tool0"
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2, nanosec=0)

        seed = JointState()
        seed.name = self.joint_names
        seed.position = self.current_joints
        req.ik_request.robot_state.joint_state = seed

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z) - 0.80

        # Orientation: tool0 pointing straight down with yaw angle around world Z
        pose.pose.orientation.x = math.cos(yaw / 2.0)
        pose.pose.orientation.y = math.sin(yaw / 2.0)
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        req.ik_request.pose_stamped = pose
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        res = future.result()
        if res and res.error_code.val == 1:
            raw = list(res.solution.joint_state.position[:6])
            wrapped = []
            for s, sd in zip(raw, self.current_joints):
                diff = (s - sd + math.pi) % (2.0 * math.pi) - math.pi
                wrapped.append(sd + diff)
            return wrapped
        return None

    def move_joints(self, target_joints, duration_sec=2.0):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in target_joints]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))

        traj.points.append(point)
        goal.trajectory = traj

        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=duration_sec + 2.0)
        time.sleep(duration_sec + 0.2)
        self.current_joints = list(target_joints)
        return True

    def move_cartesian(self, start_xyz, target_xyz, duration_sec=2.5, num_steps=6, yaw=0.0):
        """Move smoothly along a straight Cartesian line by generating intermediate waypoints."""
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        dt = duration_sec / num_steps

        for i in range(1, num_steps + 1):
            alpha = i / num_steps
            x = start_xyz[0] + alpha * (target_xyz[0] - start_xyz[0])
            y = start_xyz[1] + alpha * (target_xyz[1] - start_xyz[1])
            z = start_xyz[2] + alpha * (target_xyz[2] - start_xyz[2])
            sol = self.solve_ik(x, y, z, yaw)
            if sol is None:
                self.get_logger().error(f"Cartesian IK failed at step {i}: ({x:.3f}, {y:.3f}, {z:.3f})")
                return False
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in sol]
            pt.time_from_start = Duration(sec=int(i * dt), nanosec=int(((i * dt) % 1) * 1e9))
            traj.points.append(pt)
            self.current_joints = list(sol)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self.traj_client.send_goal_async(goal)

        # Đợi trajectory controller thực thi hoàn chỉnh chuyển động
        rclpy.spin_until_future_complete(self, future, timeout_sec=duration_sec + 2.0)
        time.sleep(duration_sec + 0.2)
        return True

    def home(self) -> str:
        self.get_logger().info("[Skill] HOME: Moving arm to safe home position.")
        self.move_joints(self.home_pose, duration_sec=3.0)
        self.last_hover_xyz = None
        return "SUCCESS"

    def pick(self, x: float, y: float, obj_name: str = "object", yaw: float = 0.0) -> str:
        """
        Pick sequence:
        1. Open gripper
        2. Move horizontally to hover height (z = 1.08m world) via Cartesian transit
        3. Lower straight down to grasp height
        4. Close gripper (stall on object)
        5. Lift straight up to hover height
        """
        self.get_logger().info(f"[Skill] PICK: {obj_name} at ({x:.3f}, {y:.3f})")
        self.control_gripper(0.0)

        hover_z = 1.08
        # Đưa độ cao gắp về mức cân bằng trước đó:
        # grasp_z = 0.958m (chóp 2 ngón kẹp hoàn toàn lơ lửng, không chạm vào mặt bàn)
        grasp_z = 0.968

        if getattr(self, 'last_hover_xyz', None) is not None:
            if not self.move_cartesian(self.last_hover_xyz, (x, y, hover_z), duration_sec=2.5, num_steps=6, yaw=yaw):
                hover_joints = self.solve_ik(x, y, hover_z, yaw)
                if not hover_joints:
                    self.get_logger().error(f"Failed IK for hover position above {obj_name}")
                    return "FAILED"
                self.move_joints(hover_joints, duration_sec=2.5)
        else:
            hover_joints = self.solve_ik(x, y, hover_z, yaw)
            if not hover_joints:
                self.get_logger().error(f"Failed IK for hover position above {obj_name}")
                return "FAILED"
            self.move_joints(hover_joints, duration_sec=2.5)

        # Hạ từ từ thẳng đứng xuống độ cao gắp
        if not self.move_cartesian((x, y, hover_z), (x, y, grasp_z), duration_sec=3.0, num_steps=8, yaw=yaw):
            return "FAILED"

        # Bật hút vacuum hỗ trợ cố định
        self.set_vacuum(True)
        time.sleep(0.4)

        # Giảm độ khép kẹp về 0.48 rad theo yêu cầu, lực ép êm 25N
        self.control_gripper(0.45, max_effort=25.0, slow_steps=10)
        time.sleep(1.0)
        self.holding_object = obj_name
        self.holding_yaw = yaw

        # Nhấc thẳng đứng lên hover_z êm ái
        if not self.move_cartesian((x, y, grasp_z), (x, y, hover_z), duration_sec=3.2, num_steps=8, yaw=yaw):
            return "FAILED"

        self.last_hover_xyz = (x, y, hover_z)

        # Verification of grasp: Spin to process vacuum_grasping callback if available
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        return "SUCCESS"

    def place(self, x: float, y: float, target_name: str = "target", yaw: Optional[float] = None) -> str:
        """
        Place sequence:
        1. Move horizontally at hover height above target via Cartesian transit
        2. Lower straight down to place height
        3. Open gripper
        4. Retract straight up to hover height
        """
        if yaw is None:
            yaw = getattr(self, 'holding_yaw', 0.0)
        self.get_logger().info(f"[Skill] PLACE: {self.holding_object} -> {target_name} at ({x:.3f}, {y:.3f})")

        hover_z = 1.08
        place_z = 0.970

        start_hover = getattr(self, 'last_hover_xyz', (0.3, 0.0, hover_z))
        if not self.move_cartesian(start_hover, (x, y, hover_z), duration_sec=3.0, num_steps=8, yaw=yaw):
            hover_joints = self.solve_ik(x, y, hover_z, yaw)
            if not hover_joints:
                self.get_logger().error(f"Failed IK for hover position above target {target_name}")
                return "FAILED"
            self.move_joints(hover_joints, duration_sec=2.5)

        # Hạ thẳng đứng xuống độ cao đặt vật
        if not self.move_cartesian((x, y, hover_z), (x, y, place_z), duration_sec=2.0, num_steps=6, yaw=yaw):
            return "FAILED"

        # Tắt vacuum và mở rộng tay kẹp hoàn toàn để nhả vật tự nhiên
        self.set_vacuum(False)
        self.control_gripper(0.0)
        time.sleep(0.5)
        self.holding_object = None

        # Rút tay gắp thẳng đứng lên an toàn
        if not self.move_cartesian((x, y, place_z), (x, y, hover_z), duration_sec=2.0, num_steps=6, yaw=yaw):
            return "FAILED"

        self.last_hover_xyz = (x, y, hover_z)
        return "SUCCESS"

    def get_camera_object_pose(self, obj_name: str, timeout_sec: float = 5.0) -> Optional[Tuple[float, float, float]]:
        """Đọc tọa độ thời gian thực của object từ Camera (/scene_state)."""
        start = time.time()
        self.get_logger().info(f"[Vision] Chờ tọa độ từ Camera cho '{obj_name}'...")
        while time.time() - start < timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_scene_dict and 'objects' in self.latest_scene_dict:
                obj_data = self.latest_scene_dict['objects'].get(obj_name)
                if obj_data:
                    x = float(obj_data['x'])
                    y = float(obj_data['y'])
                    yaw = float(obj_data.get('yaw', 0.0))
                    self.get_logger().info(f"[Vision] Camera phát hiện '{obj_name}' tại: x={x:.4f}, y={y:.4f}, yaw={yaw:.3f}")
                    return (x, y, yaw)
        self.get_logger().warn(f"[Vision] Không nhận được camera cho '{obj_name}'. Dùng fallback danh nghĩa.")
        return None

def main(args=None):
    rclpy.init(args=args)

    # Chọn vật đỏ (red_cube) để test
    cube_name = "red_cube"
    target_name = "zone_c"
    place_x, place_y = 0.30, 0.18

    # Nếu có truyền tên khối qua dòng lệnh thì dùng tên đó
    clean_argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    if clean_argv:
        cube_name = clean_argv[0]
        if cube_name == "blue_cube":
            target_name = "temp_pos"
            place_x, place_y = 0.32, -0.09

    robot = UR3Skills()

    # 1. Quay robot về Home xoay 90 độ (-1.57 rad) sang bên hông để camera nhìn rõ toàn bộ bàn
    robot.get_logger().info(f"=== BẮT ĐẦU TEST ROBOT SKILLS: GẮP VẬT ĐỎ ({cube_name}) ===")
    robot.home()
    time.sleep(1.0)

    # 2. Lấy tọa độ thực tế trực tiếp từ Camera Node cho vật đỏ
    cam_pose = robot.get_camera_object_pose(cube_name, timeout_sec=6.0)
    if cam_pose:
        pick_x, pick_y, pick_yaw = cam_pose
    else:
        # Fallback danh nghĩa nếu chưa bật perception node
        pick_x, pick_y, pick_yaw = (0.22, 0.11, 0.0) if cube_name == "red_cube" else (0.30, 0.00, 0.0)

    # 3. Thực hiện Pick gắp vật đỏ
    robot.get_logger().info(f"-> Gắp {cube_name} tại tọa độ camera (x={pick_x:.3f}, y={pick_y:.3f}, yaw={pick_yaw:.3f})")
    status_pick = robot.pick(pick_x, pick_y, obj_name=cube_name, yaw=pick_yaw)
    robot.get_logger().info(f"Kết quả Pick: {status_pick}")

    # 4. Place vào vị trí đích an toàn (zone_c)
    if status_pick == "SUCCESS":
        robot.get_logger().info(f"-> Đặt {cube_name} vào {target_name} tại (x={place_x:.3f}, y={place_y:.3f})")
        status_place = robot.place(place_x, place_y, target_name=target_name, yaw=pick_yaw)
        robot.get_logger().info(f"Kết quả Place: {status_place}")

    # 5. Quay về vị trí Home (90 độ) để không che khuất camera cho các bước tiếp theo
    robot.home()
    robot.get_logger().info("=== TEST HOÀN TẤT THÀNH CÔNG ===")

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()