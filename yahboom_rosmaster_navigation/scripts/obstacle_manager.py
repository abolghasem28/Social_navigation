#!/usr/bin/env python3
"""# 
Virtual Obstacle Manager - Complete Working Version
Publishes obstacles as PointCloud2 for Nav2's ObstacleLayer
run: nav1
Usage: ros2 run yahboom_rosmaster_navigation obstacle_manager.py
Removing 
author: Abolghasem Esmaeily 
:date: December 5, 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math


class ObstacleManager(Node):
    """
    Manages virtual obstacles and publishes them as PointCloud2.
    Works seamlessly with Nav2's existing ObstacleLayer.
    """
    
    def __init__(self):
        super().__init__('obstacle_manager')
        
        # Publisher for virtual obstacles
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            '/virtual_obstacles',
            10
        )
        
        # Store obstacles
        self.obstacles = []
        
        # Timer to publish all obstacles
        self.timer = self.create_timer(1.0, self.publish_all_obstacles)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Obstacle Manager Started!")
        self.get_logger().info("Publishing to: /virtual_obstacles")
        self.get_logger().info("Message type: PointCloud2")
        self.get_logger().info("=" * 50)
        
        # Add test obstacles
        # hee we can defien different radius for each obstacle in order to with type of goal we are looking
        # self.add_obstacle(x=3.0, y=2.0, radius=1.0)
        # # 
        self.add_obstacle(x=1.06, y=2, radius=0.8)
        # self.add_obstacle(x=-1.0, y=1.03, radius=0.5)
        #self.add_obstacle(x=-0.5, y=-7.2, radius=0.6)
        # #
        # self.add_obstacle(x=-3.0, y=-3.0, radius=0.5)

        # Obstacle for Person A (left person, facing right)
        self.add_obstacle(x=-0.75, y=-7.0, radius=0.8)

        # Obstacle for Person B (right person, facing left)  
        self.add_obstacle(x=0.75, y=-7.0, radius=0.8)
            
    def add_obstacle(self, x, y, radius=0.5):
        """
        Add a circular obstacle at position (x, y).
        
        Args:
            x, y: Position in map frame (meters)
            radius: Obstacle radius (meters)
        """
        obstacle = {
            'type': 'circle',
            'x': x,
            'y': y,
            'radius': radius
        }
        self.obstacles.append(obstacle)
        self.get_logger().info(f"Added obstacle: ({x:.1f}, {y:.1f}) radius={radius:.1f}m")
    
    def clear_obstacles(self):
        """Remove all obstacles."""
        self.obstacles.clear()
        self.get_logger().info("Cleared all obstacles")
    
    def publish_all_obstacles(self):
        """Publish all obstacles as a single point cloud."""
        if not self.obstacles:
            return
        
        # Collect all points from all obstacles
        all_points = []
        for obstacle in self.obstacles:
            points = self.generate_obstacle_points(obstacle)
            all_points.extend(points)
        
        if not all_points:
            return
        
        # Create and publish point cloud
        cloud = self.create_point_cloud(all_points)
        self.obstacle_pub.publish(cloud)
    
    def generate_obstacle_points(self, obstacle):
        """Generate 3D cylinder of points for obstacle."""
        points = []
        x_center = obstacle['x']
        y_center = obstacle['y']
        radius = obstacle['radius']
        height = 0.5  # 50cm tall
        
        # Dense sampling: 36 angles, multiple radii, 10 heights
        for angle_idx in range(36):
            angle = 2 * math.pi * angle_idx / 36
            for r_idx in range(max(5, int(radius / 0.05)) + 1):
                r = radius * r_idx / max(5, int(radius / 0.05))
                x = x_center + r * math.cos(angle)
                y = y_center + r * math.sin(angle)
                for h_idx in range(11):
                    z = height * h_idx / 10
                    points.append([x, y, z])
        
        return points
    
    def create_point_cloud(self, points):
        """Create a PointCloud2 message from list of [x, y, z] points."""
        cloud = PointCloud2()
        
        # Header
        cloud.header.frame_id = "map"
        cloud.header.stamp = self.get_clock().now().to_msg()
        
        # Fields
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Properties
        cloud.is_bigendian = False
        cloud.point_step = 12  # 3 floats * 4 bytes
        cloud.row_step = cloud.point_step * len(points)
        cloud.is_dense = True
        cloud.width = len(points)
        cloud.height = 1
        
        # Pack point data
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', 
                float(point[0]), 
                float(point[1]), 
                float(point[2])
            ))
        
        cloud.data = b''.join(buffer)
        
        return cloud


def main():
    rclpy.init()
    manager = ObstacleManager()
    
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.get_logger().info("Shutting down...")
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()