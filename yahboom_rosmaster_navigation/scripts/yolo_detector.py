#!/usr/bin/env python3
"""
NODE 1: YOLOv8-Pose Detector (Depth Image Version)
==================================================
1. Subscribes to RGB Image AND Depth Image (aligned).
2. YOLO11s-Pose: Finds perfect full-body bounding boxes AND skeletal joints.
3. Depth Lookup: Reads the exact distance Z from the chest (u,v).
4. Deprojection: Calculates real world X, Y using Camera Math.
5. Publishes: Real 3D coordinates to /detected_humans and boxes to /annotated_image.

0. SETUP:
Activate conda environment

In terminal 1:
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true

# In terminal 2:
# python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/yolo_detector.py

Author: Abolghasem Esmaeily
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import os
from ultralytics import YOLO

class YOLOPoseDetector(Node):
    def __init__(self):
        super().__init__('yolo_pose_detector')
        
        self.debug_image_path = "/home/aesmaeily/ros2_ws/src/test_images/yolo_label_box_Id_image.jpg"
        self.ensure_dir(self.debug_image_path)
        
        # SETUP YOLOv8-Pose
        # The 'n' stands for nano (fastest). It will auto-download 'yolov8n-pose.pt' the first time it runs.
        self.get_logger().info("Loading YOLO11-Pose model...")
        self.detector = YOLO('yolo11s-pose.pt')

        # TOPICS
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw' 
        self.cam_info_topic = '/camera/camera/color/camera_info'

        # QOS PROFILE
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # PUBLISHERS
        self.human_pub = self.create_publisher(PoseArray, '/detected_humans', 10)
        self.annotated_pub = self.create_publisher(Image, '/annotated_image', 10)

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_intrinsics = None 
        self.data_lock = threading.Lock()
        self.frame_count = 0

        # SUBSCRIBERS
        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_profile)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos_profile)
        self.create_subscription(CameraInfo, self.cam_info_topic, self.info_cb, qos_profile)
        
        self.create_timer(0.033, self.detect_loop)
        self.get_logger().info("Node YOLO Detector Initialized. Waiting for images...")

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
            self.camera_intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5]) # fx, fy, cx, cy

    def detect_loop(self):
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
                self.frame_count += 1
                return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(rgb_curr, 'bgr8')
            img_h, img_w, _ = cv_img.shape
            depth_cv = self.bridge.imgmsg_to_cv2(depth_curr, 'passthrough')
            annotated_cv = cv_img.copy()

            # RUN YOLO-POSE INFERENCE
            results = self.detector(cv_img, conf = 0.40, classes=[0], verbose=False)
            valid_people = []

            if len(results) > 0 and results[0].keypoints is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                classes = results[0].boxes.cls.cpu().numpy()
                box_confs = results[0].boxes.conf.cpu().numpy()
                keypoints = results[0].keypoints.xy.cpu().numpy()
                confs = results[0].keypoints.conf.cpu().numpy()

                for i in range(len(boxes)):
                    if int(classes[i]) != 0: 
                        continue # YOLO Class 0 is 'person'

                    # 1. THE PERFECT BOUNDING BOX (No padding math needed!)
                    x_min, y_min, x_max, y_max = map(int, boxes[i])
                    box_conf = float(box_confs[i])

                    # 2. THE LOCALIZATION POINT (Chest Center)
                    # YOLO COCO format: index 5 is Left Shoulder, index 6 is Right Shoulder
                    l_shoulder = keypoints[i][5]
                    r_shoulder = keypoints[i][6]
                    l_conf = float(confs[i][5])
                    r_conf = float(confs[i][6])

                    if l_conf > 0.4 and r_conf > 0.4:
                        u = int((l_shoulder[0] + r_shoulder[0]) / 2.0)
                        v = int((l_shoulder[1] + r_shoulder[1]) / 2.0)
                    else:
                        # Fallback: If shoulders are hidden, use upper-middle of the bounding box
                        u = int((x_min + x_max) / 2.0)
                        v = int(y_min + (y_max - y_min) * 0.3)

                    u = max(0, min(u, img_w - 1))
                    v = max(0, min(v, img_h - 1))

                    valid_people.append({
                        'u': u,
                        'v': v,
                        'bbox': (x_min, y_min, x_max, y_max),
                        'box_conf': box_conf,
                        'l_conf': l_conf,
                        'r_conf': r_conf,
                    })

            # SORT FROM LEFT TO RIGHT (Critical for Gemini Prompt!)
            valid_people.sort(key=lambda person: person['bbox'][0]) 

            pose_msg = PoseArray()
            pose_msg.header = rgb_curr.header 

            for idx, person in enumerate(valid_people):
                u, v = person['u'], person['v']
                x_min, y_min, x_max, y_max = person['bbox']
                
                # Draw the perfect YOLO bounding box
                cv2.rectangle(annotated_cv, (x_min, y_min), (x_max, y_max), (0, 255, 0), thickness=1)
                label = f"ID {idx + 1}"
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated_cv, (x_min, y_min - h - 10), (x_min + w, y_min), (0, 255, 0), -1)
                cv2.putText(annotated_cv, label, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                # Depth lookup (Exact same math as before)
                z_raw_mm = 0.0
                count = 0
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        uu, vv = u+dx, v+dy
                        if 0 <= uu < img_w and 0 <= vv < img_h:
                            d = depth_cv[vv, uu]
                            if d > 0 and d < 8000:
                                z_raw_mm += d
                                count += 1
                
                if count > 0:
                    Z = ((z_raw_mm / count) / 1000.0) + 0.25
                    fx, fy, cx, cy = self.camera_intrinsics
                    X = (u - cx) * Z / fx
                    Y = (v - cy) * Z / fy
                    self.get_logger().info(f"Person {idx + 1}: (u,v)=({u},{v}), Z={Z:.2f}m -> (X,Y,Z)=({X:.2f}, {Y:.2f}, {Z:.2f})")
                    
                    p = Pose()
                    p.position.x, p.position.y, p.position.z = float(X), float(Y), float(Z)
                    # Reuse Pose.orientation to publish detector metadata without changing message types:
                    # x = YOLO box confidence, y/z = left/right shoulder confidence, w = primary display confidence.
                    p.orientation.x = float(person['box_conf'])
                    p.orientation.y = float(person['l_conf'])
                    p.orientation.z = float(person['r_conf'])
                    p.orientation.w = float(person['box_conf'])
                    pose_msg.poses.append(p)

            #if len(pose_msg.poses) > 0:

            # Now, if 0 people are detected, it publishes an empty PoseArray.
            # This empty array tells Nav2 to immediately erase the old cylinders!
            self.human_pub.publish(pose_msg)
            
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_cv, encoding="bgr8")
            annotated_msg.header = rgb_curr.header
            self.annotated_pub.publish(annotated_msg)

            cv2.imwrite(self.debug_image_path, annotated_cv)
            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    rclpy.spin(YOLOPoseDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
