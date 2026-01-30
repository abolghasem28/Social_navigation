#!/usr/bin/env python3
# this code tells this node to synchronize with the simulation clock, or it will fail.
# ros2 run yahboom_rosmaster_navigation tf_probe.py --ros-args --use-sim-time true
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

class TFProbe(Node):
    def __init__(self):
        super().__init__('tf_probe')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Check every 1 second
        self.timer = self.create_timer(1.0, self.check_transform)
        self.get_logger().info("TF Probe Started. Looking for 'map' -> 'cam_1_link'...")

    def check_transform(self):
        from_frame = 'cam_1_link' # Or cam_1_depth_optical_frame
        to_frame = 'map'
        
        try:
            # 1. Try to get the Latest Available transform (Time 0)
            # This is the easiest way to check connectivity ignoring time lag
            if self.tf_buffer.can_transform(to_frame, from_frame, Time()):
                t = self.tf_buffer.lookup_transform(to_frame, from_frame, Time())
                
                # Calculate coordinates
                x = t.transform.translation.x
                y = t.transform.translation.y
                
                # Log the timestamps to debug the mismatch
                now = self.get_clock().now().nanoseconds / 1e9
                tf_time = t.header.stamp.sec + t.header.stamp.nanosec / 1e9
                lag = now - tf_time
                
                self.get_logger().info(
                    f"✅ SUCCESS! \n"
                    f"   Position: x={x:.2f}, y={y:.2f}\n"
                    f"   Node Time: {now:.2f}s | TF Time: {tf_time:.2f}s | Lag: {lag:.2f}s"
                )
            else:
                self.get_logger().warn(f"⏳ Waiting... Buffer has frames but path {to_frame}->{from_frame} not ready.")
                
                # Debug: Print what IS in the buffer
                all_frames = self.tf_buffer.all_frames_as_yaml()
                if 'map' not in all_frames:
                    self.get_logger().error("   'map' frame NOT found in buffer yet.")
                
        except Exception as e:
            self.get_logger().error(f"❌ ERROR: {e}")

def main():
    rclpy.init()
    node = TFProbe()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()