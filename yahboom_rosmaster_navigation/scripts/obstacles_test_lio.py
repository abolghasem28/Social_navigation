#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np

class SocialWallSimulator(Node):
    def __init__(self):
        super().__init__('social_wall_simulator')
        
        # Subscribe to your YOLO output
        self.sub = self.create_subscription(PoseArray, '/detected_humans', self.pose_cb, 10)
        
        # Publish to the topic your Nav2 YAML is listening to
        self.pub = self.create_publisher(PointCloud2, '/social_obstacles', 10)
        self.get_logger().info("Social Wall Simulator Running: Projecting 1m lines.")

    def pose_cb(self, msg: PoseArray):
        points = []
        
        # For every human detected by YOLO
        for pose in msg.poses:
            start_x = pose.position.x
            start_y = pose.position.y
            z = 0.5 # Hover slightly above ground
            
            # Create a line of points 1 meter forward in the X direction
            # Interpolating a point every 5 cm (0.05m) to make a solid wall
            for offset in np.arange(0.0, 1.05, 0.05):
                points.append([start_x + offset, start_y, z])
                
        # Create the Header matching your camera/robot frame
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id 

        # Create and publish the PointCloud2 message
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud_msg)

def main():
    rclpy.init()
    rclpy.spin(SocialWallSimulator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()