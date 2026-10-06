#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from ur3_llm_tamp.world_model import SceneState, ObjectState, ZoneState, load_scene_config

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        # Load scene configuration
        pkg_path = get_package_share_directory('ur3_llm_tamp')
        cfg_path = os.path.join(pkg_path, 'config', 'scene.yaml')
        self.cfg = load_scene_config(cfg_path)

        self.table_z = self.cfg['table']['z']
        self.cube_half_z = self.cfg['cube_size'] / 2.0
        self.cube_z = self.table_z + self.cube_half_z
        self.cam_pos = np.array(self.cfg['camera']['position']) # [0.30, 0.0, 1.60]
        self.delta_z = self.cam_pos[2] - self.cube_z # 1.60 - 0.82 = 0.78m

        self.zone_occupancy_radius = self.cfg['zone_occupancy_radius']

        # Camera intrinsics (default or updated via /camera/camera_info)
        # horizontal fov 1.047 rad (60 deg), 800x800 -> fx = fy = (800/2) / tan(1.047/2) ~= 692.8
        self.fx = 692.8
        self.fy = 692.8
        self.cx = 400.0
        self.cy = 400.0

        self.color_ranges = {}
        for name, intervals in self.cfg['colors'].items():
            parsed_intervals = []
            for low, high in intervals:
                parsed_intervals.append((np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)))
            self.color_ranges[name] = parsed_intervals

        self.min_blob_area = self.cfg.get('min_blob_area_px', 150)

        # Nominal zones
        self.nominal_zones = self.cfg['zones'] # name -> {x: .., y: ..}

        # Subscribers & Publisher
        self.sub_info = self.create_subscription(
            CameraInfo, self.cfg['camera']['info_topic'], self.info_callback, 10)
        self.sub_img = self.create_subscription(
            Image, self.cfg['camera']['image_topic'], self.image_callback, 10)

        self.pub_scene = self.create_publisher(String, '/scene_state', 10)

        self.current_scene = SceneState()
        # Initialize zones
        for zname, zinfo in self.nominal_zones.items():
            self.current_scene.zones[zname] = ZoneState(
                name=zname, x=float(zinfo['x']), y=float(zinfo['y']), detected=True, occupant=None)

        self.get_logger().info("Perception Node initialized, listening to camera...")

    def info_callback(self, msg: CameraInfo):
        if msg.k[0] > 0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def pixel_to_world(self, u: float, v: float, z_target: float) -> tuple:
        """
        Overhead camera mounted at [cam_x, cam_y, cam_z] looking straight down (pitch = +90 deg).
        Camera optical frame:
          u (image X -> right): aligns with Gazebo -Y axis
          v (image Y -> down):  aligns with Gazebo -X axis
        """
        dz = self.cam_pos[2] - z_target
        # In optical frame:
        # x_c = (u - cx) * dz / fx
        # y_c = (v - cy) * dz / fy
        # Mapping to world frame:
        world_x = self.cam_pos[0] - (v - self.cy) * dz / self.fy
        world_y = self.cam_pos[1] - (u - self.cx) * dz / self.fx
        return float(world_x), float(world_y)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        new_objects = {}

        for cube_name, intervals in self.color_ranges.items():
            mask = None
            for lower, upper in intervals:
                m = cv2.inRange(hsv_image, lower, upper)
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) >= self.min_blob_area:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        u = float(M["m10"] / M["m00"])
                        v = float(M["m01"] / M["m00"])
                        wx, wy = self.pixel_to_world(u, v, self.cube_z)

                        # Minimum area rect for yaw orientation
                        rect = cv2.minAreaRect(c)
                        # rect[2] is angle in degrees
                        angle_deg = rect[2]
                        yaw = math.radians(angle_deg)

                        new_objects[cube_name] = ObjectState(
                            name=cube_name,
                            x=round(wx, 4),
                            y=round(wy, 4),
                            z=round(self.cube_z, 4),
                            yaw=round(yaw, 3),
                            zone=None
                        )

        # Update scene
        self.current_scene.objects = new_objects
        self.current_scene.stamp = self.get_clock().now().nanoseconds / 1e9
        self.current_scene.recompute_occupancy(self.zone_occupancy_radius)

        # Publish SceneState JSON
        msg_out = String()
        msg_out.data = self.current_scene.to_json()
        self.pub_scene.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()