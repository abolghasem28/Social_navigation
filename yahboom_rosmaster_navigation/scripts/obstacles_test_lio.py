#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct

class SyntheticSocialEntity(Node):
    def __init__(self):
        super().__init__('synthetic_social_entity_injector')
        self.pub = self.create_publisher(PointCloud2, '/social_obstacles', 10)
        self.create_timer(0.1, self.inject_payload)

    def inject_payload(self):
        cloud = PointCloud2()
        cloud.header.frame_id = 'LIO_base_link'
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1
        
        # Matrix: Generate a vertical pillar at X=1.5m, Y=0.0m
        points = [struct.pack('fff', 1.5, 0.0, z * 0.2) for z in range(10)]
        
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.data = b''.join(points)
        self.pub.publish(cloud)

def main():
    rclpy.init()
    rclpy.spin(SyntheticSocialEntity())
    rclpy.shutdown()

if __name__ == '__main__':
    main()