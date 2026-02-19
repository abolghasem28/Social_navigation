#!/usr/bin/env python3
"""
NODE 1: MediaPipe Detector (Depth Image Version)
================================================
1. Subscribes to RGB Image AND Depth Image (aligned).
2. MediaPipe: Finds the 2D pixel (u, v) of the humans.
3. Depth Lookup: Reads the exact distance Z from the Depth Image at (u,v).
4. Deprojection: Calculates real world X, Y using Camera Math.
5. Publishes: Real 3D coordinates to /detected_humans.

In terminal 1:
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true

In terminal 2:
ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 ~/anaconda3/envs/gemini_env/lib/libstdc++.so.6

and run activate the Gemini environment and then:
python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/mediapipe_detector.py

Author: Abolghasem Esmaeily
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import os
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class MultiPersonMediaPipe(Node):
    def __init__(self):
        super().__init__('mediapipe_detector')
        
        self.debug_image_path = "/home/aesmaeily/ros2_ws/src/test_images/mediapipe_image.jpg"
        self.ensure_dir(self.debug_image_path)
        
        # MODEL PATH for MediaPipe Pose Landmarker (Heavy version) in order to detect multiple people. Make sure this file exists!
        model_path = '/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/pose_landmarker_heavy.task'
        if not os.path.exists(model_path):
            self.get_logger().error(f"CRITICAL ERROR: Model file missing at {model_path}")
            return

        # SETUP MEDIAPIPE
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=False,
            num_poses=5,  
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.detector = vision.PoseLandmarker.create_from_options(options)

        # TOPICS
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw' 
        self.cam_info_topic = '/camera/camera/color/camera_info'

        # QOS PROFILE (MATCHING "RELIABLE" FROM OUR LOGS)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Publish from Node 1 (Detector) /detected_humans
        self.human_pub = self.create_publisher(PoseArray, '/detected_humans', 10)
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_intrinsics = None 
        self.data_lock = threading.Lock()
        self.frame_count = 0

        # SUBSCRIBERS, what we need for detection
        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_profile)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos_profile)
        self.create_subscription(CameraInfo, self.cam_info_topic, self.info_cb, qos_profile)
        
        self.create_timer(0.033, self.detect_loop)
        self.get_logger().info("Node Mediapipe Detector Initialized. Waiting for images...")

    # Safty function make sure the program doesn't crash when it tries to save an image but the directory doesn't exist.
    def ensure_dir(self, file_path):
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            try: os.makedirs(directory)
            except: pass

    def rgb_cb(self, msg):
        with self.data_lock: self.latest_rgb = msg

    def depth_cb(self, msg):
        with self.data_lock: self.latest_depth = msg

    def info_cb(self, msg):
        if self.camera_intrinsics is None:
            fx = msg.k[0]; cx = msg.k[2]
            fy = msg.k[4]; cy = msg.k[5]
            self.camera_intrinsics = (fx, fy, cx, cy)
            #self.get_logger().info(f"Camera Intrinsics Set: fx={fx:.1f}")

    def detect_loop(self):
        # Check what we have before trying to process
        has_rgb = False
        has_depth = False
        
        with self.data_lock:
            if self.latest_rgb: has_rgb = True
            if self.latest_depth: has_depth = True
            
            if has_rgb and has_depth and self.camera_intrinsics:
                rgb_curr = self.latest_rgb
                depth_curr = self.latest_depth
                self.latest_rgb = None
                self.latest_depth = None
            else:
                # If missing something, print and RETURN
                # if self.frame_count % 30 == 0: # Print once per second
                #     if not has_rgb: self.get_logger().warn(f"Waiting for RGB: {self.rgb_topic}")
                #     if not has_depth: self.get_logger().warn(f"Waiting for DEPTH: {self.depth_topic}")
                #     if not self.camera_intrinsics: self.get_logger().warn("Waiting for Camera Info...")
                self.frame_count += 1
                return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(rgb_curr, 'bgr8')
            img_h, img_w, _ = cv_img.shape
            rgb_cv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            depth_cv = self.bridge.imgmsg_to_cv2(depth_curr, 'passthrough')

            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_cv)
            detection_result = self.detector.detect(mp_image)
            
        
            num_people = len(detection_result.pose_landmarks)
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"Processing... Found {num_people} people.")

            pose_msg = PoseArray()
            pose_msg.header = rgb_curr.header 
            
            # The code is very dependent on Shoulders as landmakrs, if it is not visible we skip that person. 
            for idx, landmarks in enumerate(detection_result.pose_landmarks):
                l_shoulder = landmarks[11] # Left Shoulder is landmark 11 in MediaPipe Pose ans 12 is Right Shoulder
                r_shoulder = landmarks[12]
                if l_shoulder.visibility < 0.5 or r_shoulder.visibility < 0.5: 
                    continue

                u = int((l_shoulder.x + r_shoulder.x) / 2.0 * img_w)
                v = int((l_shoulder.y + r_shoulder.y) / 2.0 * img_h)
                u = max(0, min(u, img_w - 1))
                v = max(0, min(v, img_h - 1))

                # Depth lookup
                z_raw_mm = 0
                count = 0
                # Look in a 5x5 window around (u,v) to get a more stable depth reading, since depth can be noisy. However the range can be adjusted based what we expact from the environment.
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        uu, vv = u+dx, v+dy
                        if 0 <= uu < img_w and 0 <= vv < img_h:
                            d = depth_cv[vv, uu]
                            if d > 0 and d < 8000:
                                z_raw_mm += d
                                count += 1
                
                if count > 0:
                    Z = ((z_raw_mm / count) / 1000.0) + 0.20
                    fx, fy, cx, cy = self.camera_intrinsics
                    X = (u - cx) * Z / fx
                    Y = (v - cy) * Z / fy
                    self.get_logger().info(f"Person {idx + 1}: (u,v)=({u},{v}), (X,Y,Z)=({X:.2f}, {Y:.2f}, {Z:.2f})")
                    p = Pose()
                    p.position.x, p.position.y, p.position.z = float(X), float(Y), float(Z)
                    pose_msg.poses.append(p)

            if len(pose_msg.poses) > 0:
                self.human_pub.publish(pose_msg)
            
            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    rclpy.spin(MultiPersonMediaPipe())
    rclpy.shutdown()

if __name__ == '__main__':
    main()