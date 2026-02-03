#!/usr/bin/env python3
"""
Social Navigation - Hybrid Tracker Using Depth + Visual Fallback 
=================================================
1. SENSORS: Hybrid Depth + Visual Fallback if Depth fails.
2. TRACKING: Uses HumanTracker class for Velocity Clamping in order to prevent teleporting obstacles by noisy or missing depth data.
3. SMOOTHING: Low-Pass Filter to remove jitter and ghosting obstacles.
author: Abolghasem Esmaeily 
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import google.generativeai as genai
from PIL import Image as PILImage
import json
import threading
import struct
import math
import time
import uuid 


class HumanTracker:
    def __init__(self, x, y, z, radius):
        # Generate a unique integer ID for Rviz
        self.id = uuid.uuid4().int & (1<<32)-1
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius
        self.last_update = time.time()
        
        # --- STABILITY PARAMETERS ---
        self.alpha = 0.2       # Smoothing: 20% New, 80% Old (Very Smooth) choose of alpha is based on system update rate and a moving person speed 
        self.max_step = 0.3    # Velocity Clamp: Max jump 30cm per frame

    def update(self, new_x, new_y, new_z, new_radius):
        self.last_update = time.time()
        
        # 1. VELOCITY CLAMPING (Anti-Teleport)
        dx = new_x - self.x
        dy = new_y - self.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist > self.max_step:
            # If jump is too big (e.g. 1.0m), limit it to max_step (0.3m)
            scale = self.max_step / dist
            new_x = self.x + dx * scale
            new_y = self.y + dy * scale
            # Dampen Z changes if X/Y was clamped
            new_z = self.z + (new_z - self.z) * scale

        # 2. EXPONENTIAL SMOOTHING (Low Pass Filter)
        self.x = self.x * (1 - self.alpha) + new_x * self.alpha
        self.y = self.y * (1 - self.alpha) + new_y * self.alpha
        self.z = self.z * (1 - self.alpha) + new_z * self.alpha
        
        # 3. Radius Update (Keep largest to be safe)
        self.radius = max(self.radius, new_radius)

# =========================================================================
# SOCIAL NAVIGATOR
# =========================================================================
class SocialNavigatorHybrid(Node):
    def __init__(self):
        super().__init__('social_navigator_hybrid')
        
        self.declare_parameter('gemini_api_key', '')
        self.api_key = self.get_parameter('gemini_api_key').value
        
        # Camera Topics and map frame
        self.rgb_topic = '/cam_1/color/image_raw'
        self.pc_topic = '/cam_1/depth/color/points' 
        self.target_frame = 'map' 

        if not self.api_key:
            self.get_logger().error("MISSING GEMINI API KEY, Please set a valid key.")
            return

        # Gemini SETUP
        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel('gemini-2.0-flash')
        
        self.bridge = CvBridge() # OpenCV Bridge for ROS image messages to OpenCV images 
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    
        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_profile_sensor_data)
        # in both them I used qos_profile_sensor_data for better performance with sensor data
        self.create_subscription(PointCloud2, self.pc_topic, self.pc_cb, qos_profile_sensor_data)
        
    
        self.obstacle_pub = self.create_publisher(PointCloud2, '/virtual_obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/human_markers', 10)

        self.debug_pub = self.create_publisher(Marker, '/debug_click_point', 10)

        # A generic camera model parameters (will be updated if CameraInfo received)
        self.fx = 554.25 # Approx for 640 width (Standard ROS)
        self.fy = 554.25
        self.cx = 320.5
        self.cy = 240.5
        self.calibrated = False

        self.create_subscription(CameraInfo, '/cam_1/color/camera_info', self.info_cb, 10)
        # STATE 
        self.latest_rgb = None
        self.latest_pc = None
        self.data_lock = threading.Lock() # ROS2 threading lock used to synchronize access to shared data, preventing race conditions, ensure data remains consistent and thread-safe.
        
        # TRACKING STATE (List of HumanTracker objects)
        self.trackers = [] 
        self.tracker_lock = threading.Lock()
        self.obstacle_ttl = 2.5 # is a threshold in seconds to remove old obstacles if not updated and it can be changed based on application needs
        # The choice of 2.5 is due to Gemini takes 1.5 seconds to process an image

        self.create_timer(1.0, self.analyze_loop) 
        self.create_timer(0.1, self.publish_obstacles) 

        self.get_logger().info("Social Navigation Hybrid Tracker use Depth point clouds and Bounding Boxes for human detection and tracking.")

    def rgb_cb(self, msg):
        with self.data_lock: self.latest_rgb = msg

    def pc_cb(self, msg):
        with self.data_lock: self.latest_pc = msg

 
    def analyze_loop(self):
        """A loop that processes the latest RGB and PointCloud data using Gemini model."""
        with self.data_lock:
            if self.latest_rgb is None:
                self.get_logger().warning("Waiting for camera...", throttle_duration_sec=2)
                return
            rgb_msg = self.latest_rgb
            pc_msg = self.latest_pc

        # Threaded processing to avoid blocking the main ROS thread callbacks
        threading.Thread(target=self.process_gemini, args=(rgb_msg, pc_msg)).start()

    def info_cb(self, msg):
        if not self.calibrated:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.calibrated = True
            #self.get_logger().info(f"Calibration Updated: fx={self.fx:.2f}, cx={self.cx:.2f}")

    def process_gemini(self, rgb_msg, pc_msg):
        # 1. SETUP LENS MATH (Handle Missing Calibration)
        try:
            img_w = rgb_msg.width
            img_h = rgb_msg.height

            # If it not received real calibration yet, we estimate it 
            if not self.calibrated:
                fov = 1.5184
                self.fx = img_w / (2 * math.tan(fov / 2)) # Estimate FX,FY, CY, CX 
                self.fy = self.fx                       
                self.cx = img_w / 2.0              
                self.cy = img_h / 2.0                     

            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB))
            
            scale_x, scale_y = 1.0, 1.0
            if pc_msg and pc_msg.width > 0:
                scale_x = pc_msg.width / img_w
                scale_y = pc_msg.height / img_h



            """ PROMPT EXPLANATION:
                1. "key_point": [320, 240] 
                - We provide integers as an example to force the AI to return PIXEL coordinates.
                - We need exact pixels (integers) to index directly into the Depth Sensor (PointCloud) array.

                2. "bbox": [0.2, 0.4, 0.8, 0.6]
                - We provide floats as an example to force the AI to return NORMALIZED coordinates (0.0 to 1.0).
                - Normalized values are "safer" because they are independent of image resolution. 
                    (e.g., "0.5" is always the center, whether the image is 640x480 or 1920x1080).
                """

            prompt = f"""
            Find humans. Return JSON.
            "engagement": "high", "medium", "low".
            "key_point": [x, y] (Center of chest).
            "bbox": [ymin, xmin, ymax, xmax] (0-1).
            Format: {{ "humans": [ {{ "key_point": [320, 240], "bbox": [0.2, 0.4, 0.8, 0.6], "engagement": "high" }} ] }}
            Image Size: {img_w}x{img_h}.
            """
            
            try:
                resp = self.model.generate_content([prompt, pil_img])
                text = resp.text.strip().replace("```json", "").replace("```", "")
                if "'" in text: text = text.replace("'", '"')
                data = json.loads(text[text.find('{'):text.rfind('}')+1])
            except: return 

            humans = data.get('humans', [])
            current_detections = []
            
            frame_id = pc_msg.header.frame_id if pc_msg else rgb_msg.header.frame_id
            stamp = pc_msg.header.stamp if pc_msg else rgb_msg.header.stamp

            for i, human in enumerate(humans):
                kp = human.get('key_point')
                bbox = human.get('bbox')
                eng = human.get('engagement', 'low').lower()
                
                if not kp: continue
                
                # Hybrid distance logic
                final_z = None
                method = "VISUAL_BACKUP"

                # PRIMARY: DEPTH SENSOR PointCloud
                if pc_msg:
                    rgb_x, rgb_y = kp[0], kp[1]
                    pc_x = int(rgb_x * scale_x)
                    pc_y = int(rgb_y * scale_y)
                    
                    pt_3d = self.get_xyz_smart_scan(pc_msg, pc_x, pc_y)
                    
                    if pt_3d:
                        final_z = pt_3d[2]
                        method = "DEPTH_SENSOR"

                # Visual Fallback (If Depth Failed / Ghost)
                if final_z is None and bbox and len(bbox) == 4:
                    method = "VISUAL_BACKUP (Ghost)"
                    ymin, _, ymax, _ = bbox
                    px_h = (ymax - ymin) * img_h
                    
                    # Uses self.fy instead of 'focal_length'
                    if px_h > 10: 
                         # We use 0.65m as approx visible torso height
                        final_z = (0.65 * self.fy) / px_h 

                if final_z:
                    # Uses self.fx and self.cx instead of 'focal_length' and 'x_center'
                    cx_pixel = kp[0]
                    final_x = (cx_pixel - self.cx) * final_z / self.fx
                    final_y = 0.0 
                    
                    map_pt = self.transform_point(final_x, final_y, final_z, frame_id, stamp)
                    
                    if map_pt:
                        radius = 0.85 if 'high' in eng else (0.60 if 'medium' in eng else 0.35)
                        self.get_logger().info(f"Human {i+1}: {eng.upper()} -> {final_z:.2f}m [{method}]")
                        
                        current_detections.append({
                            'x': map_pt[0], 'y': map_pt[1], 'z': map_pt[2], 'r': radius
                        })

            self.update_trackers(current_detections)

        except Exception as e:
            # This prints the error so you know exactly what line failed
            self.get_logger().error(f"Gemini Error: {e}")

    # =========================================
    # TRACKING & MATCHING LOGIC
    # =========================================
    def update_trackers(self, detections):
        with self.tracker_lock:
            now = time.time()
            used_trackers = set()
            
            # Match new detections to closest existing tracker
            for det in detections:
                best_tracker = None
                min_dist = 1.2 # Search radius (meters), can be ajusted, increase or deacrease for detection of fast or slow moving humans
                
                for t in self.trackers:
                    if t in used_trackers: continue
                    dist = math.sqrt((t.x - det['x'])**2 + (t.y - det['y'])**2)
                    if dist < min_dist:
                        min_dist = dist
                        best_tracker = t
                
                if best_tracker:
                    # Update Existing (Applies Velocity Clamp & Smoothing)
                    best_tracker.update(det['x'], det['y'], det['z'], det['r'])
                    used_trackers.add(best_tracker)
                else:
                    # Create New Tracker
                    new_t = HumanTracker(det['x'], det['y'], det['z'], det['r'])
                    self.trackers.append(new_t)
                    used_trackers.add(new_t)
            
            # Remove Old Trackers
            self.trackers = [t for t in self.trackers if (now - t.last_update) < self.obstacle_ttl]

    # =========================================
    # HELPERS 
    # =========================================
    def get_xyz_smart_scan(self, pc_msg, u, v):
        x_off, y_off, z_off = -1, -1, -1
        for f in pc_msg.fields:
            if f.name == 'x': x_off = f.offset
            elif f.name == 'y': y_off = f.offset
            elif f.name == 'z': z_off = f.offset
        if x_off == -1: return None


        """The chioce of 20 with/horison and 60 height/vertical is human aspect Ratio 1:3 Rule of thumb matches as standing human, we can adjusting for different scenario in general"""
        width_search = 20 
        height_search = 60
        u_min, u_max = max(0, u - width_search), min(pc_msg.width, u + width_search)
        v_min, v_max = max(0, v - height_search), min(pc_msg.height, v + height_search)
        
        candidates = []
        for row in range(v_min, v_max, 5): # Step by 5 pixels for speed up calculation, we can adjust it based on performance need
            offset_row = row * pc_msg.row_step
            for col in range(u_min, u_max, 5):
                off = offset_row + col * pc_msg.point_step
                try:
                    z = struct.unpack_from('f', pc_msg.data, off + z_off)[0]

                    # sorting out bad depth vlues, z > 1m is for Kinect V1 or early Asus Xtion, modern cameras Intel RealSense D435 or OAK-D can be around 0.2m to 0.3m. 
                    if np.isfinite(z) and z > 0.5 and z < 10.0:
                        x = struct.unpack_from('f', pc_msg.data, off + x_off)[0]
                        y = struct.unpack_from('f', pc_msg.data, off + y_off)[0]
                        candidates.append((z, x, y))
                except: pass
        if not candidates: return None
        candidates.sort(key=lambda p: p[0]) # Sort by Z distance
        idx = len(candidates) // 2 
        best = candidates[idx]
        return (best[1], best[2], best[0])

    def transform_point(self, x, y, z, from_frame, time_stamp):
        """Here safety check followed by the actual coordinate conversion. 
        It prevents your robot from crashing when it tries to do math on data that doesn't exist yet."""
        try:
            pt = PointStamped() 
            pt.header.frame_id = from_frame
            pt.header.stamp = time_stamp
            pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
            if self.tf_buffer.can_transform(self.target_frame, from_frame, time_stamp, rclpy.duration.Duration(seconds=0.5)):
                safety_checkout = self.tf_buffer.transform(pt, self.target_frame)
                return (safety_checkout.point.x, safety_checkout.point.y, safety_checkout.point.z)
        except: pass
        return None

    # =========================================
    # PUBLISHING
    # =========================================
    def publish_obstacles(self):
        cloud_points = []
        marker_array = MarkerArray()

        # old obstacles removal marker
        delete_m = Marker()
        delete_m.action = Marker.DELETEALL
        marker_array.markers.append(delete_m)

        with self.tracker_lock:
            if not self.trackers:
                self.publish_cloud(b'', 0)
                self.marker_pub.publish(marker_array)
                return

            for tracker in self.trackers:
                # Use smoothed values from tracker of human movement that obstacle slide smoothly on the map
                cx, cy = tracker.x, tracker.y
                
                m = Marker()
                m.header.frame_id = self.target_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "humans"
                m.id = tracker.id # Stable ID
                m.type = Marker.CYLINDER # the choice of cylinder is because of rotation invariant safe for moving humans
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, 0.9
                m.scale.x = tracker.radius * 2.0  # Visual Social Radius
                m.scale.y = tracker.radius * 2.0
                m.scale.z = 1.8
                m.color.a = 0.3 # Very Transparent
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0 # Cyan
                marker_array.markers.append(m)


                physical_radius = 0.30 
                points_per_layer = 36  # Density
                height_layers = 20     # From 0m to 2.0m
                
                for h_idx in range(height_layers):
                    z = h_idx * 0.1 # Every 10cm up

                # Create a dense grid/spiral of points
                    for angle_idx in range(points_per_layer):
                        angle = (2 * math.pi * angle_idx) / points_per_layer
                        
                        # We add random noise or multiple radii to make it "Solid"
                        # Ring 1 (Outer Shell)
                        px = cx + physical_radius * math.cos(angle)
                        py = cy + physical_radius * math.sin(angle)
                        cloud_points.append(struct.pack('fff', px, py, z))
                        
                        # Ring 2 (Inner Core) - Ensures no "hollow" center
                        px_inner = cx + (physical_radius * 0.5) * math.cos(angle)
                        py_inner = cy + (physical_radius * 0.5) * math.sin(angle)
                        cloud_points.append(struct.pack('fff', px_inner, py_inner, z))
                        
                        # Center Spine
                        if angle_idx == 0:
                            cloud_points.append(struct.pack('fff', cx, cy, z))

        # Publish the PointCloud
        if cloud_points:
            self.publish_cloud(b''.join(cloud_points), len(cloud_points))
            self.marker_pub.publish(marker_array)

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
        cloud.is_bigendian = False # It matches CPU and the struct.pack default behavior (otherwise robot would see obstacles kilometers away or garbage noise).
        cloud.point_step = 12
        cloud.row_step = 12 * num_points
        cloud.data = data_bytes
        self.obstacle_pub.publish(cloud)

def main():
    rclpy.init()
    rclpy.spin(SocialNavigatorHybrid())
    rclpy.shutdown()

if __name__ == '__main__':
    main()