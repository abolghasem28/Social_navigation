#!/usr/bin/env python3
"""
DATASET LOGGER (Ground Truth Edition)
=====================================
Subscribes to the live /annotated_image topic from Node 1.
Displays a live video feed.
Press keys 1-8 to instantly save the frame with the correct Ground Truth label.
Press 'q' to quit.

In terminal 1:
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true

In terminal 2:
ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 ~/anaconda3/envs/gemini_env/lib/libstdc++.so.6
python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/mediapipe_detector.py

In terminal 3:
python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/dataset_logger.py

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import threading

class DatasetLogger(Node):
    def __init__(self):
        super().__init__('dataset_logger')
        
        self.output_dir = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        self.bridge = CvBridge()
        self.latest_annotated_img = None
        self.latest_raw_img = None
        self.latest_depth_img = None
        
        # Simpler mapping for terminal input
        self.scenarios = {
            '1': 'photography',
            '2': 'conversation',
            '3': 'ignoring',
            '4': 'proxemics',
            '5': 'presenting',
            '6': 'playing',
            '7': 'sharedtask',
            '8': 'parallelwalking'
        }
        
        #self.sub_annotated = self.create_subscription(Image, '/annotated_image', self.annotated_callback, 10)
        self.sub_raw = self.create_subscription(Image, '/camera/camera/color/image_raw', self.raw_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        # Start a background thread specifically to listen to your Terminal
        self.kb_thread = threading.Thread(target=self.terminal_listener)
        self.kb_thread.daemon = True
        self.kb_thread.start()
        
        self.get_logger().info("=========================================")
        self.get_logger().info("Dataset Logger Started (Dual-Save Mode)!")
        self.get_logger().info("Type a number in THIS TERMINAL and press ENTER:")
        self.get_logger().info("  [1] Photography")
        self.get_logger().info("  [2] Conversation")
        self.get_logger().info("  [3] Ignoring (Mutual Disengagement)")
        self.get_logger().info("  [4] Proxemics (Corridors)")
        self.get_logger().info("  [5] Presenting")
        self.get_logger().info("  [6] Playing")
        self.get_logger().info("  [7] Shared Task")
        self.get_logger().info("  [8] Parallel Walking")
        self.get_logger().info("  [q] QUIT")
        self.get_logger().info("=========================================")

    # def annotated_callback(self, msg):
    #     try:
    #         cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    #         self.latest_annotated_img = cv_img.copy()
    #         # Scrern dispaly
    #         cv2.imshow("RealSense Live Annotated Feed", cv_img)
    #         cv2.waitKey(1)
    #     except Exception: pass

    def raw_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_raw_img = cv_img.copy()
            #cv2.imshow("RealSense Live Raw Feed", cv_img)
            #cv2.waitKey(1)
        except Exception: pass

    def depth_callback(self, msg):
        try:
            # Depth images are 16-bit integers, not 8-bit colors!
            cv_img = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.latest_depth_img = cv_img.copy()
        except Exception as e: 
            pass


    def terminal_listener(self):
        # This loop runs constantly in the background listening to your keyboard
        while True:
            try:
                user_input = input().strip().lower()
                
                if user_input in self.scenarios:
                    if self.latest_raw_img is not None and self.latest_depth_img is not None:
                        scenario_name = self.scenarios[user_input]
                        timestamp = int(time.time() * 1000)
                        
                        # Save raw image
                        raw_filename = os.path.join(self.output_dir, f"{scenario_name}_{timestamp}_raw.jpg")
                        cv2.imwrite(raw_filename, self.latest_raw_img)

                        # Save annotated image
                            # annotated_filename = os.path.join(self.output_dir, f"{scenario_name}_{timestamp}_annotated.jpg")
                            # cv2.imwrite(annotated_filename, self.latest_annotated_img)depth_filename = os.path.join(self.output_dir, f"{scenario_name}_{timestamp}_depth.png")

                        # Save depth image (as 16-bit PNG to preserve depth values)
                        depth_filename = os.path.join(self.output_dir, f"{scenario_name}_{timestamp}_depth.png")
                        cv2.imwrite(depth_filename, self.latest_depth_img)

                        print(f"\n---> SUCCESS: Saved RAW and DEPTH images for {scenario_name}!\n")
                    else:
                        print("\n---> ERROR: Waiting for Raw and Depth camera feeds to initialize...\n")
                        
                elif user_input == 'q':
                    print("\nQuitting Dataset Logger...")
                    os._exit(0) 
                    
            except EOFError:
                break

def main(args=None):
    rclpy.init(args=args)
    node = DatasetLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()