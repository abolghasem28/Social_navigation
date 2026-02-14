#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import time

class FakeCameraNode(Node):
    def __init__(self):
        super().__init__('fake_camera_publisher')

        # Topics
        self.rgb_topic = '/camera/camera/color/image_raw'
        #self.pc_topic = '/camera/camera/depth/color/points' 
        self.camera_info_topic = '/camera/camera/color/camera_info'

        self.pub_rgb = self.create_publisher(Image, self.rgb_topic, 10)
        self.pub_camera_info = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.bridge = CvBridge()

        self.image_path = '/home/aesmaeily/ros2_ws/src/test_images/test_image3.jpg'
        self.cv_image = cv2.imread(self.image_path)

        if self.cv_image is None:
            self.get_logger().error(f"Failed to load image from {self.image_path}")
            exit()

        self.get_logger().info(f"Loaded image from {self.cv_image.shape} " ) 

        self.timer = self.create_timer(1.0, self.publish_image)

    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fake_camera_link'

        self.pub_rgb.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.height = self.cv_image.shape[0]
        info.width = self.cv_image.shape[1]

        # --- NEW: Add Fake Calibration Data (Focal Length) ---
        # estimating fx = width roughly (approx 90 deg FOV)
        fx = info.width * 0.8 
        fy = fx
        cx = info.width / 2.0
        cy = info.height / 2.0
        
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0] 
        # -----------------------------------------------------

        self.pub_camera_info.publish(info)

        self.get_logger().info(f"Published Fake Image frame")

def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()