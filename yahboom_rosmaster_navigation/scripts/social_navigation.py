#!/usr/bin/env python3
"""
NODE 2: Social Navigator (Step 3 & 4)
=====================================
1. Filtering (EKF): Stabilizes the noisy input from Node 1.
2. Intelligence (Gemini): Determines engagement.
3. Reporting: Prints 'Raw vs Filtered' statistics to prove stability.

In terminal 1:
ros2 run tf2_ros static_transform_publisher -0.10 0 0.052 0 -1.5708 0 lio_gripper_interface_link camera_link

In terminal 2:
ros2 run tf2_ros static_transform_publisher 0 0 0.5 0 0 0 LIO_base_link camera_link

In terminal 3:
python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/navigation_mediapipe_test.py

Author: Abolghasem Esmaeily
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseArray, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import cv2
import numpy as np
import google.generativeai as genai
from PIL import Image as PILImage
import json
import threading
from concurrent.futures import ThreadPoolExecutor
import struct
import math
import time
import uuid 
import os

# --- STEP 3: EKF STABILIZER ---
class HumanTracker:
    def __init__(self, x, y, z, engagement="low"):
        self.id = uuid.uuid4().int & (1<<32)-1
        
        # EKF State [x, y, vx, vy]
        # Here is P initialization with some uncertainty
        # Q is the process noise, R is the measurement noise
        # State is initialized with the first measurement (x,y) and zero velocity.
        self.state = np.array([[x], [y], [0.0], [0.0]]) 
        self.P = np.eye(4) * 0.5   
        
        # TUNED MATRICES FOR STABILITY
        # Low Process Noise: How fast do we think humans can change speed?
        # For slow move humans, we can set this low to trust our predictions more, like below.
        #self.Q = np.diag([0.01, 0.01, 0.05, 0.05]) 

        # High Measurement Noise: Don't trust the camera completely.
        #self.R = np.eye(2) * 0.5  

        # For fast moving humans, we can set this higher to be more responsive, like below.
        self.Q = np.diag([0.05, 0.05, 0.2, 0.2])
         
        # Low Measurement Noise: Trust more the camera, Faster movement
        self.R = np.eye(2) * 0.3

        self.z = z
        self.engagement = engagement 
        self.last_update = time.time()
        self.alpha = 0.2  # For low-pass filtering of Z (Depth)
        # LOGS FOR REPORT
        self.history_raw = []
        self.history_ekf = []
        self.frame_count = 0

    def predict(self):
        # Predict first
        # A = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0 ], [0, 0, 0, 1 ]])
        # self.state = A @ self.state
        # self.P = A @ self.P @ A.T + self.Q
        # Update with Measurement
        # H is the measurement matrix that maps the state to the measurement space (x,y)
        # y is innovation, the measurement residual, S is the innovation/residual covariance, K is the Kalman Gain
        dt = 0.1
        A = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0 ], [0, 0, 0, 1 ]])
        self.state = A @ self.state
        self.P = A @ self.P @ A.T + self.Q
        # Friction
        self.state[2, 0] *= 0.8
        self.state[3, 0] *= 0.8 

    def update(self, measured_x, measured_y, measured_z):
        self.last_update = time.time()
        
        # Log Raw Input
        self.history_raw.append((measured_x, measured_y, measured_z))
        
        # EKF Math
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        z = np.array([[measured_x], [measured_y]])
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        
        # Low Pass Filter for Z (Depth) since EKF is only on X,Y
        if measured_z:
            self.z = measured_z
            #print(f"Updated Z with low-pass filter: {self.z:.2f}m")
        
        # Log Filtered Output
        self.history_ekf.append((float(self.state[0, 0]), float(self.state[1, 0]), self.z))
        self.frame_count += 1

class SocialNavigator(Node):
    def __init__(self):
        super().__init__('social_navigator_main')
        self.declare_parameter('gemini_api_key', '')
        self.api_key = self.get_parameter('gemini_api_key').value


        if not self.api_key:
            self.api_key = os.getenv('GEMINI_API_KEY', '')
            if not self.api_key:
                self.get_logger().error("MISSING GEMINI API KEY.")
                return
            
        # TOPICS
        self.pose_topic = '/detected_humans'
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.target_frame = 'LIO_base_link' 

        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel('gemini-2.0-flash')
        
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_subscription(PoseArray, self.pose_topic, self.pose_cb, 10)
        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_profile_sensor_data)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/social_obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/human_markers', 10)

        self.trackers = []
        self.latest_rgb = None
        self.gemini_busy = False
        self.thread_executor = ThreadPoolExecutor(max_workers=1)

        self.create_timer(0.1, self.analyse_loop)
        self.create_timer(2.0, self.trigger_gemini)
        
        self.get_logger().info("Node 2 Started. Waiting for data to generate report...")

    def pose_cb(self, msg):
        frame_id = msg.header.frame_id
        stamp = msg.header.stamp
        detections = []


        if not self.tf_buffer.can_transform(self.target_frame, frame_id, rclpy.time.Time()):
            self.get_logger().warn(f"TF FAIL: Cannot transform from '{frame_id}' to '{self.target_frame}'")
            #self.get_logger().warn(f"HINT: Run 'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map {frame_id}'")
            return
        

        if not self.tf_buffer.can_transform(self.target_frame, frame_id, rclpy.time.Time()):
            return

        for pose in msg.poses:
            map_pt = self.transform_point(pose.position.x, pose.position.y, pose.position.z, frame_id, stamp)
            
            # Debug log 
            if map_pt is None:
                self.get_logger().warn("Transform failed silently!")
            else:
                pass # self.get_logger().info(f"Transform OK: {map_pt}")
            
            if map_pt:
                detections.append({'x': map_pt[0], 'y': map_pt[1], 'z': map_pt[2]})

        self.update_ekf(detections)

    def rgb_cb(self, msg):
        self.latest_rgb = msg

    def update_ekf(self, detections):
        for t in self.trackers: 
            t.predict()

        used_trackers = set()
        for det in detections:
            best_t = None
            min_dist = 5.0 
            
            for t in self.trackers:
                if t in used_trackers: continue
                dist = math.sqrt((t.state[0, 0]-det['x'])**2 + (t.state[1, 0]-det['y'])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_t = t
            
            if best_t:
                best_t.update(det['x'], det['y'], det['z'])
                used_trackers.add(best_t)
                
                # PRINT REPORT EVERY 100 FRAMES
                if best_t.frame_count == 100:
                    self.print_stability_report(best_t)
                    best_t.history_raw = []
                    best_t.history_ekf = []
                    best_t.frame_count = 0
            else:
                self.trackers.append(HumanTracker(det['x'], det['y'], det['z']))

        #for t in self.trackers:
         #   if t not in used_trackers: t.predict()
        
        # TTL: Keep memory for 3.0s
        now = time.time()
        self.trackers = [t for t in self.trackers if (now - t.last_update) < 3.0]

    def print_stability_report(self, tracker):
        if len(tracker.history_raw) < 2: return
        raw_arr = np.array(tracker.history_raw)
        ekf_arr = np.array(tracker.history_ekf)
        
        std_raw = np.std(raw_arr, axis=0)
        std_ekf = np.std(ekf_arr, axis=0)
        
        print("\n" + "="*50)
        print(f"   STABILITY REPORT (Human ID: {tracker.id})")
        print("="*50)
        print(f"RAW SENSOR NOISE:")
        print(f"  X: ±{std_raw[0]:.4f} m")
        print(f"  Y: ±{std_raw[1]:.4f} m")
        print(f"  Z: ±{std_raw[2]:.4f} m  Depth ")
        print("-" * 50)
        print(f"FILTERED OUTPUT By KF:")
        print(f"  X: ±{std_ekf[0]:.4f} m")
        print(f"  Y: ±{std_ekf[1]:.4f} m")
        print(f"  Z: ±{std_ekf[2]:.4f} m Depth ")
        print("-" * 50)
        #print("This proves (Depth) is noisy and (EKF) fixes it.")
        print("="*50 + "\n")

    def trigger_gemini(self):
        if self.gemini_busy or not self.latest_rgb or not self.trackers: return
        self.gemini_busy = True
        img_copy = self.latest_rgb
        self.thread_executor.submit(self.process_gemini, img_copy)

    def process_gemini(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
            pil_img = pil_img.resize((320, 240)) 
            prompt = 'Determine engagement. JSON: {"humans": [{"engagement": "high"}]}'
            resp = self.model.generate_content([prompt, pil_img])
            text = resp.text.strip().replace("```json", "").replace("```", "")
            if "'" in text: text = text.replace("'", '"')
            data = json.loads(text[text.find('{'):text.rfind('}')+1])
            humans = data.get('humans', [])

            for h in humans:
                eng = h.get('engagement', 'low').lower()
                if eng == 'high' and self.trackers:
                    self.trackers[0].engagement = 'high'
                    self.get_logger().info("Gemini: High Engagement.")
        except: pass 
        finally: self.gemini_busy = False

    def analyse_loop(self):
        # Generate visualization markers and point cloud for obstacles
        marker_array = MarkerArray()
        cloud_points = []
        
        delete_m = Marker()
        delete_m.action = Marker.DELETEALL
        marker_array.markers.append(delete_m)

        pairs = []
        for i in range(len(self.trackers)):
            t = self.trackers[i]
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "humans"
            m.id = t.id 
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(t.state[0, 0]); m.pose.position.y = float(t.state[1, 0]); m.pose.position.z = 0.9
            m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 1.8
            if t.engagement == 'high' or 'Medium' in t.engagement: 
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0 
            else: 
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0 
            m.color.a = 0.8
            marker_array.markers.append(m)
            self.add_cylinder_points(cloud_points, float(t.state[0, 0]), float(t.state[1, 0]), 0.25)
            
            # find pairs or more for walls
            for j in range(i + 1, len(self.trackers)):
                t2 = self.trackers[j]
                dx = float(t.state[0, 0]) - float(t2.state[0, 0])
                dy = float(t.state[1, 0]) - float(t2.state[1, 0])
                dist = math.sqrt(dx*dx + dy*dy)
                if dist < 2.0:
                    pairs.append((t, t2, dist, dx, dy))

        for (t1, t2, dist, dx, dy) in pairs:
            wall_m = Marker()
            wall_m.header.frame_id = self.target_frame
            wall_m.header.stamp = self.get_clock().now().to_msg()
            wall_m.ns = "walls"
            wall_m.id = (t1.id + t2.id) & 0xFFFFFF
            wall_m.type = Marker.CUBE
            wall_m.action = Marker.ADD

            # Position at midpoint
            wall_m.pose.position.x = (float(t1.state[0, 0]) + float(t2.state[0, 0])) / 2.0
            wall_m.pose.position.y = (float(t1.state[1, 0]) + float(t2.state[1, 0])) / 2.0
            wall_m.pose.position.z = 0.9

            # Calculate orientation, angle between the two trackers
            angle = math.atan2(dy, dx)

            # Covert angle to quaternion
            wall_m.pose.orientation.z = math.sin(angle / 2.0)
            wall_m.pose.orientation.w = math.cos(angle / 2.0)

            wall_m.scale.x = dist # length of the wall
            wall_m.scale.y = 0.2 # thickness of the wall
            wall_m.scale.z = 1.8 # height of the wall
            wall_m.color.r = 0.0; wall_m.color.g = 0.0; wall_m.color.b = 1.0; wall_m.color.a = 0.5 
            marker_array.markers.append(wall_m)
            steps = int(dist / 0.1)
            for s in range(steps):
                # r 
                r = s / float(steps)
                px = (1-r)*float(t1.state[0, 0]) + r*float(t2.state[0, 0])
                py = (1-r)*float(t1.state[1, 0]) + r*float(t2.state[1, 0])
                self.add_cylinder_points(cloud_points, px, py, 0.1)

        self.marker_pub.publish(marker_array)
        if cloud_points: self.publish_cloud(b''.join(cloud_points), len(cloud_points))
        else: self.publish_cloud(b'', 0)

    def transform_point(self, x, y, z, from_frame, time_stamp):
        if self.target_frame == from_frame: return (x,y,z)
        pt = PointStamped()
        pt.header.frame_id = from_frame
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
        try:
            out = self.tf_buffer.transform(pt, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.2))
            return (out.point.x, out.point.y, out.point.z)
        except Exception as e:
            self.get_logger().warn(f"Real TF Error: {e}")
            return None

    def add_cylinder_points(self, cloud_points, cx, cy, radius):
        for h_idx in range(5):
            z = 0.2 + (h_idx * 0.3)
            for i in range(8):
                angle = (2 * math.pi * i) / 8
                cloud_points.append(struct.pack('fff', cx + radius * math.cos(angle), cy + radius * math.sin(angle), z))

    def publish_cloud(self, data_bytes, num_points):
        cloud = PointCloud2()
        cloud.header.frame_id = self.target_frame
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = num_points
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * num_points
        cloud.data = data_bytes
        self.obstacle_pub.publish(cloud)

def main():
    rclpy.init()
    rclpy.spin(SocialNavigator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()