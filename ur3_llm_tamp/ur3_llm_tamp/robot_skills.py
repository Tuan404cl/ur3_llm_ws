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

class UR3Skills(Node):
    def __init__(self):
        super().__init__('ur3_skills')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_action_controller/gripper_cmd')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.get_logger().info("Chờ kết nối tới Controllers và MoveIt 2...")
        self.gripper_client.wait_for_server()
        self.traj_client.wait_for_server()
        self.ik_client.wait_for_service()
        self.get_logger().info("Hệ thống Robot Skills đã sẵn sàng!")
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.home_pose = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    def control_gripper(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 50.0
        self.get_logger().info(f"Gripper -> {position}")
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)

    def solve_ik(self, x, y, z):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.ik_link_name = "tool0"
        req.ik_request.avoid_collisions = False 
        req.ik_request.timeout = Duration(sec=2, nanosec=0)

        # CHỈ NẠP 6 KHỚP CỦA UR3 ĐỂ KHÔNG LÀM CRASH MOVEIT
        seed = JointState()
        seed.name = self.joint_names
        seed.position = self.home_pose
        req.ik_request.robot_state.joint_state = seed

        pose = PoseStamped()
        pose.header.frame_id = "base_link" 
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z) - 0.8
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        
        req.ik_request.pose_stamped = pose
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res.error_code.val == 1:
            return list(res.solution.joint_state.position[:6])
        else:
            self.get_logger().error(f"MoveIt IK lỗi tại Z_base={pose.pose.position.z} - Mã: {res.error_code.val}")
            return None

    def move_joints(self, target_joints, duration_sec=2.0):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        
        traj.points.append(point)
        goal.trajectory = traj
        
        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(duration_sec)

    def home(self):
        self.get_logger().info("Về tư thế Home an toàn")
        self.move_joints(self.home_pose, duration_sec=3.0)

    def pick(self, x, y):
        self.get_logger().info(f"Bắt đầu PICK tại tọa độ ({x}, {y})")
        self.control_gripper(0.0) 
        
        hover_joints = self.solve_ik(x, y, 1.05)
        if hover_joints: self.move_joints(hover_joints)
        
        pick_joints = self.solve_ik(x, y, 0.985)
        if pick_joints: self.move_joints(pick_joints, duration_sec=1.5)
        
        self.control_gripper(0.8)
        
        if hover_joints: self.move_joints(hover_joints)

    def place(self, x, y):
        self.get_logger().info(f"Bắt đầu PLACE tại tọa độ ({x}, {y})")
        
        hover_joints = self.solve_ik(x, y, 1.05)
        if hover_joints: self.move_joints(hover_joints)
        
        place_joints = self.solve_ik(x, y, 0.985)
        if place_joints: self.move_joints(place_joints, duration_sec=1.5)
        
        self.control_gripper(0.0)
        
        if hover_joints: self.move_joints(hover_joints)

def main(args=None):
    rclpy.init(args=args)
    robot = UR3Skills()
    
    robot.home()
    robot.pick(0.35, 0.00)
    robot.place(0.35, 0.20)
    robot.home()
    
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()