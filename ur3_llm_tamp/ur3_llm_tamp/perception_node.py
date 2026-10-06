#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        # Subscribe ảnh từ camera overhead
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Tọa độ cố định của 3 Zone (do mặt bàn được spawn cứng)
        self.zones = {
            'zone_a': (0.35, -0.20),
            'zone_b': (0.35, 0.00),
            'zone_c': (0.35, 0.20)
        }

        # Định nghĩa khoảng màu HSV cho 5 khối
        self.color_ranges = {
            'red_cube': {'lower': np.array([0, 150, 100]), 'upper': np.array([10, 255, 255])},
            'yellow_cube': {'lower': np.array([25, 150, 100]), 'upper': np.array([35, 255, 255])},
            'green_cube': {'lower': np.array([45, 150, 100]), 'upper': np.array([75, 255, 255])},
            'blue_cube': {'lower': np.array([100, 150, 100]), 'upper': np.array([130, 255, 255])},
            'purple_cube': {'lower': np.array([140, 100, 100]), 'upper': np.array([160, 255, 255])}
        }

        self.get_logger().info("Perception Node Started - Waiting for camera...")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        state_dict = {}
        # Kiểm tra xem zone nào đang có vật
        zone_status = {z: 'empty' for z in self.zones}

        for name, ranges in self.color_ranges.items():
            mask = cv2.inRange(hsv_image, ranges['lower'], ranges['upper'])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Lấy khối to nhất (tránh nhiễu)
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Chuyển đổi Pixel sang mét (Dựa vào thông số camera z=2.0, fov, resolution)
                        # (Đã calibrate sẵn cho mô hình bài này)
                        real_x = 0.4 - (cy - 400) * 0.00288
                        real_y = 0.0 - (cx - 400) * 0.00288

                        state_dict[name] = [round(real_x, 3), round(real_y, 3)]

                        # Kiểm tra xem khối này có nằm trong zone nào không (bán kính < 0.05m)
                        for z_name, z_pos in self.zones.items():
                            dist = np.sqrt((real_x - z_pos[0])**2 + (real_y - z_pos[1])**2)
                            if dist < 0.05:
                                zone_status[z_name] = name

        # Xuất log để kiểm tra
        self.get_logger().info(f"Blocks: {state_dict}")
        self.get_logger().info(f"Zones: {zone_status}")
        self.get_logger().info("-" * 40)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()