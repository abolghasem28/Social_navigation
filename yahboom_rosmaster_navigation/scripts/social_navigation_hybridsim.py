#!/usr/bin/env python3
"""
Social Navigation - Real Robot EKF Tracker (Depth Image)
========================================================
1. ESTIMATOR: Extended Kalman Filter (EKF) [x, y, vx, vy]. 
   - Predicts motion during detection lag (Constant Velocity Model).
   - Smooths position jitter using probabilistic covariance (P, Q, R).
2. SENSORS: RealSense Depth Image (Fast O(1) lookup) + Visual Fallback.
3. STABILITY: Z-axis Low-Pass Filter to prevent depth noise (up/down jumps).
This code is for the SIMULATION environment.

LOGIC: 
   - Single Human: Represented as a Cylinder.
   - Social Wall: Created between humans ONLY if Engagement is High/Medium.
   - Low Engagement: No wall (Robot can pass through).

author: Abolghasem Esmaeily 
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge
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

class HumanTracker:
    def __init__(self, x, y, z, radius, engagement):
        self.id = uuid.uuid4().int & (1<<32)-1
        # EKF State [x, y, vx, vy]
        # Here is P initialization with some uncertainty
        # Q is the process noise, R is the measurement noise
        # State is initialized with the first measurement (x,y) and zero velocity.
        self.state = np.array([[x], [y], [0.0], [0.0]])
        self.P = np.eye(4) * 0.5   
        self.Q = np.diag([0.01, 0.01, 0.05, 0.05]) 
        self.R = np.eye(2) * 1.0   # Higher R = trust prediction more, lower R = trust measurement more
        self.z = z
        self.radius = radius
        self.engagement = engagement 
        self.last_update = time.time()
        self.hits = 1

    def predict_only(self):
        # Simple constant velocity model for prediction step
        # F is the state transition matrix
        now = time.time()
        dt = now - self.last_update

        if dt <= 0.0: 
            return
        
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0 ], [0, 0, 0, 1 ]])
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q
        self.state[2] *= 0.85
        self.state[3] *= 0.85 

    def update(self, measured_x, measured_y, measured_z, new_radius, new_eng):
        self.last_update = time.time()
        dt = 0.1
        # Predict first
        #F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0 ], [0, 0, 0, 1 ]])
        #self.state = F @ self.state
        #self.P = F @ self.P @ F.T + self.Q
        # Update with Measurement
        # H is the measurement matrix that maps the state to the measurement space (x,y)
        # y is innovation, the measurement residual, S is the innovation/residual covariance, K is the Kalman Gain
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        z = np.array([[measured_x], [measured_y]])
        # 
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state 
        self.state = self.state + K @ y
        # Update covariance
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P
        self.state[2] *= 0.85
        self.state[3] *= 0.85
        
        # Low-Pass Filter for Z (Depth)
        if measured_z is not None and measured_z > 0.1:
            self.z = (self.z * 0.85) + (measured_z * 0.15)

        self.radius = 0.9 * self.radius + 0.1 * new_radius
        self.hits += 1
        self.radius = max(self.radius, new_radius)

class SocialNavigatorHybrid(Node):
    def __init__(self):
        super().__init__('social_navigator_hybrid_Simulation')
        
        self.declare_parameter('gemini_api_key', '')
        self.api_key = self.get_parameter('gemini_api_key').value
        
        self.rgb_topic = '/cam_1/color/image_raw'
        self.pc_topic = '/cam_1/depth/color/points' 
        self.camera_info_topic = '/cam_1/color/camera_info'
        self.target_frame = 'map' 
        self.social_obstacle_topic = '/social_obstacles'
        self.human_marker_topic = '/human_markers'

        if not self.api_key:
            self.api_key = os.getenv('GEMINI_API_KEY', '')
            if not self.api_key:
                self.get_logger().error("MISSING GEMINI API KEY.")
                return

        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel('gemini-2.0-flash')
        
        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.executor_pool = ThreadPoolExecutor(max_workers=1)
        self.gemini_busy = False 

        # DATA HOLDERS FOR DEBUG DRAWING
        self.last_debug_overlays = [] # Stores [ (x1,y1,x2,y2, label), ... ]

        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.pc_topic, self.pc_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, 10)
    
        self.obstacle_pub = self.create_publisher(PointCloud2, self.social_obstacle_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.human_marker_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, '/debug_gemini_vision', 10)

        self.fx = 554.25; self.fy = 554.25; self.cx = 320.5; self.cy = 240.5
        self.calibrated = False

        self.latest_rgb = None
        self.latest_pc = None 
        self.data_lock = threading.Lock()
        
        self.trackers = [] 
        self.tracker_lock = threading.Lock()
        self.obstacle_ttl = 2.0 

        self.create_timer(0.1, self.analyse_loop) 

        self.get_logger().info(f"Social Navigation Hybrid Tracker use Depth point clouds and Bounding Boxes for human detection and creating social obstacles based on their engagement level active on {self.social_obstacle_topic}")

    def rgb_cb(self, msg):
        with self.data_lock: self.latest_rgb = msg

    def pc_cb(self, msg):
        with self.data_lock: self.latest_pc = msg

    def info_cb(self, msg):
        if not self.calibrated:
            self.fx = msg.k[0]; self.cx = msg.k[2]
            self.fy = msg.k[4]; self.cy = msg.k[5]
            self.calibrated = True
 
    def analyse_loop(self):
        """Runs at 10Hz. Handles Viz, Obstacles, and triggers Gemini if free."""
        if not self.gemini_busy:
            with self.data_lock:
                if self.latest_rgb and self.latest_pc:
                    self.gemini_busy = True
                    rgb_copy = self.latest_rgb
                    pc_copy = self.latest_pc
                    self.executor_pool.submit(self.process_gemini, rgb_copy, pc_copy)

        self.update_tracker_predictions()
        self.publish_obstacles()

    def update_tracker_predictions(self):
        with self.tracker_lock:
            now = time.time()
            for t in self.trackers:
                if (now - t.last_update) > 0.05: 
                    t.predict_only()
            self.trackers = [t for t in self.trackers if (now - t.last_update) < self.obstacle_ttl]

    def process_gemini(self, rgb_msg, pc_msg):
        try:
            img_w = rgb_msg.width; img_h = rgb_msg.height
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB))

            scale_x = pc_msg.width / img_w if pc_msg.height > 1 else 1.0
            scale_y = pc_msg.height / img_h if pc_msg.height > 1 else 1.0

            if not self.calibrated:
                fov = 1.5184
                self.fx = img_w / (2 * math.tan(fov / 2)) # Estimate FX,FY, CY, CX 
                self.fy = self.fx                       
                self.cx = img_w / 2.0              
                self.cy = img_h / 2.0
            
            """ PROMPT EXPLANATION:
                1. "key_point": [320, 240] 
                - We provide integers as an example to force the AI to return PIXEL coordinates.
                - We need exact pixels (integers) to index directly into the Depth Sensor (PointCloud) array.

                2. "bbox": [0.2, 0.4, 0.8, 0.6]
                - We provide floats as an example to force the AI to return NORMALIZED coordinates (0.0 to 1.0).
                - Normalized values are "safer" because they are independent of image resolution. 
                    (e.g., "0.5" is always the center, whether the image is 640x480 or 1920x1080).
                """

            prompt = """
            Find humans. JSON.
            "engagement": "high", "medium", "low".
            "bbox": [ymin, xmin, ymax, xmax] (0-1).
            Format: { "humans": [ { "bbox": [0.2, 0.4, 0.8, 0.6], "engagement": "high" } ] }
            """
            
            try:
                resp = self.model.generate_content([prompt, pil_img])
                text = resp.text.strip().replace("```json", "").replace("```", "")
                if "'" in text: text = text.replace("'", '"')
                data = json.loads(text[text.find('{'):text.rfind('}')+1])
            except: return 

            humans = data.get('humans', [])
            current_detections = []
            
            # Use list to store visual debug info
            new_overlays = []
            
            frame_id = rgb_msg.header.frame_id 
            stamp = rgb_msg.header.stamp

            for i, human in enumerate(humans):
                bbox = human.get('bbox')
                eng = human.get('engagement', 'low').lower()
                
                if not bbox or len(bbox) != 4: continue
                
                ymin_n, xmin_n, ymax_n, xmax_n = bbox
                
                box_h = ymax_n - ymin_n; box_w = xmax_n - xmin_n
                ymin_n = max(0.0, ymin_n - (box_h * 0.05))
                ymax_n = min(1.0, ymax_n + (box_h * 0.05))
                xmin_n = max(0.0, xmin_n - (box_w * 0.1))
                xmax_n = min(1.0, xmax_n + (box_w * 0.1))

                center_x = (xmin_n + xmax_n) / 2.0
                center_y = (ymin_n + ymax_n) / 2.0
                
                u = int(center_x * img_w)
                v = int(center_y * img_h)
                u = max(0, min(u, img_w - 1))
                v = max(0, min(v, img_h - 1))

                final_x, final_y, final_z = None, None, None
                method = "NONE"

                if pc_msg:
                    pc_u = int(u * scale_x)
                    pc_v = int(v * scale_y)
                    pt_3d = self.get_xyz_smart_scan(pc_msg, pc_u, pc_v)
                    if pt_3d:
                        final_x, final_y, final_z = pt_3d
                        method = "PC_3D"

                if final_z is None:
                    px_h = (ymax_n - ymin_n) * img_h
                    if px_h > 10: 
                        final_z = (1.70 * self.fy) / px_h 
                        final_x = (u - self.cx) * final_z / self.fx
                        final_y = (v - self.cy) * final_z / self.fy
                        method = "VISUAL"

                # Store overlay info for the main thread to draw later
                # Format: (xmin_px, ymin_px, xmax_px, ymax_px, text_str)
                px_box = (int(xmin_n*img_w), int(ymin_n*img_h), int(xmax_n*img_w), int(ymax_n*img_h))
                depth_str = f"NaN"
                if final_z is not None:
                    depth_str = f"{final_z:.2f}m"
                
                new_overlays.append({
                    'box': px_box,
                    'center': (u, v),
                    'label': f"{depth_str} ({method})"
                })

                if final_z is not None:
                    map_pt = self.transform_point(final_x, final_y, final_z, frame_id, stamp)
                    if map_pt:
                        self.get_logger().info(f"Human {i + 1}: Eng={eng} Method={method} Depth={final_z:.2f}m")
                        current_detections.append({
                            'x': map_pt[0], 'y': map_pt[1], 'z': map_pt[2], 
                            'r': 0.40, 'eng': eng
                        })

            # Update Data
            self.update_trackers(current_detections)
            self.last_debug_overlays = new_overlays 

        except Exception as e:
            self.get_logger().error(f"Gemini Error: {e}")
        finally:
            self.gemini_busy = False 

    def update_trackers(self, detections):
        with self.tracker_lock:
            used_trackers = set()
            for det in detections:
                best_tracker = None
                min_dist = 999.9 
                for t in self.trackers:
                    if t in used_trackers: continue
                    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
                    z_meas = np.array([[det['x']], [det['y']]])
                    z_pred = H @ t.state
                    y_res = z_meas - z_pred
                    S = H @ t.P @ H.T + t.R
                    try:
                        # Mahalanobis distance to determine if this detection matches the tracker's prediction
                        d2 = y_res.T @ np.linalg.inv(S) @ y_res
                        d2_val = float(d2[0,0])
                    except: d2_val = 999.0
                    # 9.21 is the 99 % alse probability of  0.01 significance for 2 DOF (x and y).
                    if d2_val < 9.21 and d2_val < min_dist:
                        min_dist = d2_val
                        best_tracker = t
                if best_tracker:
                    best_tracker.update(det['x'], det['y'], det['z'], det['r'], det['eng'])
                    used_trackers.add(best_tracker)
                else:
                    new_tracker = HumanTracker(det['x'], det['y'], det['z'], det['r'], det['eng'])
                    self.trackers.append(new_tracker)
                    used_trackers.add(new_tracker)
            # for t in self.trackers:
            #     if t not in used_trackers:
            #         t.predict_only()

    def get_xyz_smart_scan(self, pc_msg, u, v):
        x_off, y_off, z_off = -1, -1, -1
        for f in pc_msg.fields:
            if f.name == 'x': x_off = f.offset
            elif f.name == 'y': y_off = f.offset
            elif f.name == 'z': z_off = f.offset
        if z_off == -1: return None

        width = pc_msg.width
        height = pc_msg.height
        radius = 15
        u_min = max(0, u - radius); u_max = min(width, u + radius)
        v_min = max(0, v - radius); v_max = min(height, v + radius)
        
        candidates = []
        point_step = pc_msg.point_step
        row_step = pc_msg.row_step

        for r in range(v_min, v_max, 3): 
            row_start = r * row_step
            for c in range(u_min, u_max, 3):
                offset = row_start + c * point_step
                if offset + z_off + 4 > len(pc_msg.data): continue
                try:
                    z = struct.unpack_from('f', pc_msg.data, offset + z_off)[0]
                    if np.isfinite(z) and 0.5 < z < 8.0:
                        x = struct.unpack_from('f', pc_msg.data, offset + x_off)[0]
                        y = struct.unpack_from('f', pc_msg.data, offset + y_off)[0]
                        candidates.append((x, y, z))
                except: pass

        if not candidates: return None
        candidates.sort(key=lambda p: p[2])
        return candidates[len(candidates) // 2]

    def transform_point(self, x, y, z, from_frame, time_stamp):
        pt = PointStamped() 
        pt.header.frame_id = from_frame; pt.header.stamp = time_stamp
        pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
        try:
            if self.tf_buffer.can_transform(self.target_frame, from_frame, time_stamp, rclpy.duration.Duration(seconds=0.5)):
                t_pt = self.tf_buffer.transform(pt, self.target_frame)
                return (t_pt.point.x, t_pt.point.y, t_pt.point.z)
        except: pass
        try:
            trans = self.tf_buffer.lookup_transform(self.target_frame, from_frame, rclpy.time.Time())
            pt.header.stamp = rclpy.time.Time().to_msg() 
            t_pt = do_transform_point(pt, trans)
            return (t_pt.point.x, t_pt.point.y, t_pt.point.z)
        except: pass
        return None

    def publish_obstacles(self):
        cloud_points = []
        marker_array = MarkerArray()
        delete_m = Marker(); delete_m.action = Marker.DELETEALL; marker_array.markers.append(delete_m)

        # 1. PREPARE LIVE IMAGE
        live_img = None
        with self.data_lock:
            if self.latest_rgb:
                try:
                    live_img = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'bgr8')
                except: pass

        # 2. DRAW OVERLAYS (Raw Detection from Gemini - Green)
        if live_img is not None:
            # We iterate over the stored debug info from the last successful Gemini run
            for overlay in self.last_debug_overlays:
                x1, y1, x2, y2 = overlay['box']
                cx, cy = overlay['center']
                label = overlay['label']
                
                # Green Box = Raw Input
                cv2.rectangle(live_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.drawMarker(live_img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(live_img, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        with self.tracker_lock:
            hits_trackers = [t for t in self.trackers if t.hits >= 1]

            grouped_trackers = set()
            pairs = []

            for i in range(len(hits_trackers)):
                for j in range(i + 1, len(hits_trackers)):
                    trac1 = hits_trackers[i]; trac2 = hits_trackers[j]
                    dx = trac1.state[0,0] - trac2.state[0,0]
                    dy = trac1.state[1,0] - trac2.state[1,0]
                    dist = math.sqrt(dx*dx + dy*dy)
                    is_engaged = (trac1.engagement in ['high', 'medium']) or (trac2.engagement in ['high', 'medium'])
                    if dist < 2.5 and is_engaged:
                        pairs.append((trac1, trac2, dist))
                        grouped_trackers.add(trac1); grouped_trackers.add(trac2)

            for tracker in hits_trackers:
                m = Marker(); m.header.frame_id = self.target_frame; m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "humans"; m.id = tracker.id; m.type = Marker.CYLINDER; m.action = Marker.ADD
                m.pose.position.x = tracker.state[0,0]
                m.pose.position.y = tracker.state[1,0]; m.pose.position.z = 0.9
                m.scale.x = 0.6; m.scale.y = 0.6; m.scale.z = 1.8
                m.color.r = 0.0; m.color.g = 1.0 if tracker in grouped_trackers else 0.0
                m.color.b = 1.0; m.color.a = 0.6
                marker_array.markers.append(m)
                
                self.add_cylinder_points(cloud_points, tracker.state[0,0], tracker.state[1,0], 0.35)

                # 3. DRAW TRACKER STATUS (EKF Output - Cyan)
                if live_img is not None:
                    # We can't easily draw 3D bbox on 2D image without camera matrix math here, 
                    # so we list active trackers at the top left.
                    txt = f"EKF ID:{tracker.id%100} Z:{tracker.z:.2f}m"
                    y_pos = 30 + (hits_trackers.index(tracker) * 30)
                    cv2.putText(live_img, txt, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            for (trac1, trac2, dist) in pairs:
                t1_x = trac1.state[0,0]; t1_y = trac1.state[1,0]
                t2_x = trac2.state[0,0]; t2_y = trac2.state[1,0]
                mid_x = (t1_x + t2_x) / 2.0; mid_y = (t1_y + t2_y) / 2.0
                angle = math.atan2(t2_y - t1_y, t2_x - t1_x)

                m = Marker(); m.header.frame_id = self.target_frame; m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "social_walls"; m.id = (trac1.id + trac2.id) & 0xFFFFFF; m.type = Marker.CUBE; m.action = Marker.ADD
                m.pose.position.x = mid_x; m.pose.position.y = mid_y; m.pose.position.z = 0.9
                q = self.quaternion_from_yaw(angle)
                m.pose.orientation.x = q[0]; m.pose.orientation.y = q[1]; m.pose.orientation.z = q[2]; m.pose.orientation.w = q[3]
                m.scale.x = dist; m.scale.y = 0.3; m.scale.z = 1.8; m.color.a = 0.8; m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0
                marker_array.markers.append(m)

                steps = int(dist / 0.08)
                for st in range(steps): 
                    ratio = st / float(steps) 
                    px = (1 - ratio) * t1_x + ratio * t2_x
                    py = (1 - ratio) * t1_y + ratio * t2_y
                    self.add_cylinder_points(cloud_points, px, py, 0.05)
                    perp = angle + math.pi/2
                    ox = 0.1 * math.cos(perp); oy = 0.1 * math.sin(perp)
                    self.add_cylinder_points(cloud_points, px+ox, py+oy, 0.05)
                    self.add_cylinder_points(cloud_points, px-ox, py-oy, 0.05)

        if cloud_points:
            self.publish_cloud(b''.join(cloud_points), len(cloud_points))
        else:
            self.publish_cloud(b'', 0)
            
        self.marker_pub.publish(marker_array)
        
        if live_img is not None:
            try:
                self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(live_img, 'bgr8'))
            except: pass

    def add_cylinder_points(self, cloud_points, cx, cy, radius):
        points = 8; layers = 8 
        for h_idx in range(layers):
            z = 0.2 + (h_idx * 0.2) 
            for i in range(points):
                angle = (2 * math.pi * i) / points
                px = cx + radius * math.cos(angle)
                py = cy + radius * math.sin(angle)
                cloud_points.append(struct.pack('fff', px, py, z))
                if i == 0: cloud_points.append(struct.pack('fff', cx, cy, z))

    def quaternion_from_yaw(self, yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def publish_cloud(self, data_bytes, num_points):
        cloud = PointCloud2()
        cloud.header.frame_id = self.target_frame
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1; cloud.width = num_points
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12; cloud.row_step = 12 * num_points
        cloud.data = data_bytes
        self.obstacle_pub.publish(cloud)

def main():
    rclpy.init()
    rclpy.spin(SocialNavigatorHybrid())
    rclpy.shutdown()

if __name__ == '__main__':
    main()