#!/usr/bin/env python3
"""
Gemini Social Navigator - AI-powered human detection and social navigation
Enhanced with precise distance estimation using Depth Camera and LiDAR

Author: Abolghasem Esmaeily
Date: December 2025

Improvements over v1:
- Depth camera integration for precise distance measurement
- LiDAR integration for robust distance verification
- Bounding box support for precise angular position
- Fusion of multiple sensors for best estimate

Usage:
  ros2 run yahboom_rosmaster_navigation social_navigation.py \
    --ros-args -p gemini_api_key:="YOUR_API_KEY"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import google.generativeai as genai
from PIL import Image as PILImage
import json
import numpy as np
import threading
import time
import struct
import math
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class GeminiSocialNavigator(Node):
    """
    Social navigation using Gemini AI for human detection and engagement analysis.
    Enhanced with precise distance estimation using depth camera and LiDAR.
    
    Distance Estimation Methods:
    1. GEMINI_ONLY: Use Gemini's categorical estimate (original, least precise)
    2. DEPTH_CAMERA: Use depth image at human's pixel location (precise)
    3. LIDAR: Project to 2D and find closest LiDAR point (very precise)
    4. FUSION: Combine depth + LiDAR for best estimate (most robust)
    """
    
    def __init__(self):
        super().__init__('gemini_social_navigator')
        
        # ============ PARAMETERS ============
        self.declare_parameter('gemini_api_key', '')
        self.declare_parameter('camera_topic', '/cam_1/color/image_raw')
        self.declare_parameter('depth_topic', '/cam_1/depth/color/points')  # PointCloud2 from RGB-D
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('analysis_rate', 2.0)
        self.declare_parameter('obstacle_publish_rate', 10.0)
        self.declare_parameter('obstacle_ttl', 5.0)
        self.declare_parameter('detection_distance', 8.0)
        # Distance estimation method: 'gemini', 'depth', 'lidar', 'fusion'
        self.declare_parameter('distance_method', 'fusion')
        # Camera parameters (from URDF)
        self.declare_parameter('camera_fov_h', 1.2)  # Horizontal FOV in radians (~69°)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        api_key = self.get_parameter('gemini_api_key').value
        camera_topic = self.get_parameter('camera_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        analysis_rate = self.get_parameter('analysis_rate').value
        publish_rate = self.get_parameter('obstacle_publish_rate').value
        self.obstacle_ttl = self.get_parameter('obstacle_ttl').value
        self.detection_distance = self.get_parameter('detection_distance').value
        self.distance_method = self.get_parameter('distance_method').value
        self.camera_fov_h = self.get_parameter('camera_fov_h').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # ============ VALIDATE API KEY ============
        if not api_key:
            self.get_logger().error('=' * 60)
            self.get_logger().error('❌ GEMINI API KEY REQUIRED!')
            self.get_logger().error('Run with: -p gemini_api_key:="YOUR_KEY"')
            self.get_logger().error('Get key from: https://aistudio.google.com/app/apikey')
            self.get_logger().error('=' * 60)
            return
        
        # ============ INITIALIZE GEMINI ============
        genai.configure(api_key=api_key)
        try:
            self.model = genai.GenerativeModel('gemini-2.0-flash')
            self.get_logger().info('✓ Using Gemini 2.0 Flash')
        except Exception:
            try:
                self.model = genai.GenerativeModel('gemini-1.5-flash')
                self.get_logger().info('✓ Using Gemini 1.5 Flash')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize Gemini: {e}')
                return
        
        # ============ CV BRIDGE ============
        self.bridge = CvBridge()
        
        # ============ TF2 FOR ROBOT POSE ============
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ============ SUBSCRIBERS ============
        self.camera_sub = self.create_subscription(
            Image, camera_topic, self.camera_callback, 10)
        self.depth_sub = self.create_subscription(
            PointCloud2, depth_topic, self.depth_callback, 10)  # Changed to PointCloud2
        self.lidar_sub = self.create_subscription(
            LaserScan, lidar_topic, self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # ============ PUBLISHERS ============
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/virtual_obstacles', 10)
        
        # ============ STATE ============
        self.latest_image = None
        self.latest_depth = None
        self.latest_lidar = None
        self.image_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.lidar_lock = threading.Lock()
        self.analyzing = False
        self.robot_moving = False
        
        # Obstacle storage
        self.current_obstacles = []
        self.obstacles_lock = threading.Lock()
        
        # ============ TIMERS ============
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_obstacles)
        self.analysis_timer = self.create_timer(1.0 / analysis_rate, self.analyze_scene)
        
        # ============ STARTUP MESSAGE ============
        self.get_logger().info('=' * 70)
        self.get_logger().info('🤖 GEMINI SOCIAL NAVIGATOR v2.0 - ENHANCED DISTANCE ESTIMATION')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'📷 Camera topic: {camera_topic}')
        self.get_logger().info(f'📏 Depth topic: {depth_topic}')
        self.get_logger().info(f'📡 LiDAR topic: {lidar_topic}')
        self.get_logger().info(f'🎯 Distance method: {self.distance_method.upper()}')
        self.get_logger().info('')
        self.get_logger().info('Distance Estimation Methods:')
        self.get_logger().info('  gemini → Categorical (±1-2m accuracy)')
        self.get_logger().info('  depth  → Depth camera (±0.1m accuracy)')
        self.get_logger().info('  lidar  → LiDAR scan (±0.05m accuracy)')
        self.get_logger().info('  fusion → Depth + LiDAR combined (best)')
        self.get_logger().info('')
        self.get_logger().info('Engagement → Obstacle Radius:')
        self.get_logger().info('  HIGH (conversation)  → 1.0m radius')
        self.get_logger().info('  MEDIUM (standing)    → 0.6m radius')
        self.get_logger().info('  LOW (walking)        → 0.4m radius')
        self.get_logger().info('=' * 70)

    # ============ CALLBACKS ============
    
    def camera_callback(self, msg):
        """Store latest RGB camera image."""
        with self.image_lock:
            self.latest_image = msg
    
    def depth_callback(self, msg):
        """Store latest depth point cloud."""
        with self.depth_lock:
            self.latest_depth = msg
    
    def lidar_callback(self, msg):
        """Store latest LiDAR scan."""
        with self.lidar_lock:
            self.latest_lidar = msg
    
    def odom_callback(self, msg):
        """Track if robot is moving."""
        vel = msg.twist.twist
        speed = math.sqrt(vel.linear.x**2 + vel.linear.y**2)
        self.robot_moving = speed > 0.01

    # ============ ROBOT POSE ============
    
    def get_robot_pose(self):
        """Get robot pose in map frame using TF."""
        try:
            if not self.tf_buffer.can_transform('map', 'base_link', 
                                                 rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.1)):
                return None
            
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return float(t.x), float(t.y), float(yaw)
        except Exception:
            return None

    # ============ DISTANCE ESTIMATION METHODS ============
    
    def get_distance_from_gemini(self, distance_str):
        """
        Original method: Convert Gemini's categorical distance to meters.
        Accuracy: ±1-2 meters
        """
        if distance_str == 'near':
            return 1.5
        elif distance_str == 'medium':
            return 3.0
        else:
            return 5.0
    
    def get_distance_from_depth(self, bbox_center_x, bbox_center_y):
        """
        Get precise distance from depth PointCloud2 at human's angular position.
        Also returns refined angle based on where the closest cluster of points is.
        
        Args:
            bbox_center_x: Normalized x position (0-1) of human in image
            bbox_center_y: Normalized y position (0-1) of human in image
        
        Returns:
            (distance, refined_angle) tuple, or (None, None) if unavailable
        """
        with self.depth_lock:
            if self.latest_depth is None:
                return None, None
            cloud_msg = self.latest_depth
        
        try:
            # Parse PointCloud2 message
            points = self._parse_pointcloud2(cloud_msg)
            
            if points is None or len(points) == 0:
                self.get_logger().debug('Depth: No points parsed')
                return None, None
            
            # Calculate target angle from bbox position
            target_angle = (0.5 - bbox_center_x) * self.camera_fov_h
            
            # Filter points in the direction of the human (±25 degrees - wider)
            angle_tolerance = 0.44  # ~25 degrees
            
            valid_points = []  # Store (distance, angle, point) tuples
            for point in points:
                x, y, z = point[0], point[1], point[2]
                
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    continue
                
                # Skip points too close to camera or too far
                if abs(z) < 0.3 or abs(z) > self.detection_distance:
                    continue
                
                # Calculate horizontal angle (camera optical frame: z=forward, x=right)
                point_angle = math.atan2(-x, z)
                horizontal_dist = math.sqrt(x**2 + z**2)
                
                # Check if point is in the target direction
                if abs(point_angle - target_angle) < angle_tolerance:
                    valid_points.append((horizontal_dist, point_angle))
            
            if len(valid_points) == 0:
                self.get_logger().debug(f'Depth: No valid points in direction {math.degrees(target_angle):.1f}°')
                return None, None
            
            # Sort by distance
            valid_points.sort(key=lambda p: p[0])
            
            # Take the closest 25% of points and find their median angle
            num_closest = max(1, len(valid_points) // 4)
            closest_points = valid_points[:num_closest]
            
            # Distance: 25th percentile of all valid points
            all_distances = [p[0] for p in valid_points]
            distance = float(np.percentile(all_distances, 25))
            
            # Angle: median of the closest points' angles
            closest_angles = [p[1] for p in closest_points]
            refined_angle = float(np.median(closest_angles))
            
            self.get_logger().debug(
                f'Depth: Found {len(valid_points)} points, dist={distance:.2f}m, angle={math.degrees(refined_angle):.1f}°'
            )
            
            return distance, refined_angle
            
        except Exception as e:
            self.get_logger().warn(f'PointCloud depth extraction failed: {e}')
            return None, None
    
    def _parse_pointcloud2(self, cloud_msg):
        """
        Parse PointCloud2 message into numpy array of points.
        Returns array of shape (N, 3) with x, y, z coordinates.
        """
        try:
            # Get field offsets
            field_names = [f.name for f in cloud_msg.fields]
            
            if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
                return None
            
            x_offset = next(f.offset for f in cloud_msg.fields if f.name == 'x')
            y_offset = next(f.offset for f in cloud_msg.fields if f.name == 'y')
            z_offset = next(f.offset for f in cloud_msg.fields if f.name == 'z')
            
            point_step = cloud_msg.point_step
            data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
            
            # Calculate number of points
            num_points = len(cloud_msg.data) // point_step
            
            if num_points == 0:
                return None
            
            # Extract x, y, z for each point
            points = []
            for i in range(0, num_points, 10):  # Sample every 10th point for speed
                offset = i * point_step
                x = struct.unpack('f', data[offset + x_offset:offset + x_offset + 4])[0]
                y = struct.unpack('f', data[offset + y_offset:offset + y_offset + 4])[0]
                z = struct.unpack('f', data[offset + z_offset:offset + z_offset + 4])[0]
                points.append([x, y, z])
            
            return points
            
        except Exception as e:
            self.get_logger().warn(f'PointCloud2 parse error: {e}')
            return None
    
    def get_distance_from_lidar(self, angle_offset):
        """
        Get distance from LiDAR scan at the human's angular position.
        Also returns the refined angle where the closest object was found.
        
        Args:
            angle_offset: Angle from robot's forward direction (radians)
        
        Returns:
            (distance, refined_angle) tuple, or (None, None) if unavailable
        """
        with self.lidar_lock:
            if self.latest_lidar is None:
                return None, None
            lidar_msg = self.latest_lidar
        
        try:
            angle_min = lidar_msg.angle_min
            angle_max = lidar_msg.angle_max
            angle_increment = lidar_msg.angle_increment
            ranges = np.array(lidar_msg.ranges)
            
            target_angle = angle_offset
            
            if target_angle < angle_min:
                target_angle = angle_min
            elif target_angle > angle_max:
                target_angle = angle_max
            
            index = int((target_angle - angle_min) / angle_increment)
            index = max(0, min(index, len(ranges) - 1))
            
            # WIDER sample range (±15 degrees)
            angle_window = 0.26  # ~15 degrees in radians
            index_window = int(angle_window / angle_increment)
            
            start_idx = max(0, index - index_window)
            end_idx = min(len(ranges), index + index_window)
            
            # Get valid ranges in this window
            range_window = ranges[start_idx:end_idx]
            
            # Find indices of valid ranges
            valid_mask = (
                (range_window > lidar_msg.range_min) & 
                (range_window < lidar_msg.range_max) &
                np.isfinite(range_window)
            )
            
            if not np.any(valid_mask):
                return None, None
            
            valid_ranges = range_window[valid_mask]
            valid_indices = np.where(valid_mask)[0]
            
            # Use 25th percentile for distance
            percentile_25 = np.percentile(valid_ranges, 25)
            
            # Find the index closest to 25th percentile
            closest_idx = valid_indices[np.argmin(np.abs(valid_ranges - percentile_25))]
            
            distance = float(range_window[closest_idx])
            
            # Calculate the refined angle where this distance was found
            actual_index = start_idx + closest_idx
            refined_angle = angle_min + actual_index * angle_increment
            
            return distance, refined_angle
            
        except Exception as e:
            self.get_logger().warn(f'LiDAR extraction failed: {e}')
            return None, None
    
    def get_fused_distance(self, bbox_center_x, bbox_center_y, angle_offset, gemini_distance_str):
        """
        Fuse multiple distance estimates for best accuracy.
        Also returns refined angle - prefer depth camera angle over LiDAR.
        
        Returns:
            (distance, method_used, refined_angle) tuple
        """
        distances = {}
        depth_angle = None
        lidar_angle = None
        
        # Get Gemini estimate first (as reference)
        gemini_dist = self.get_distance_from_gemini(gemini_distance_str)
        distances['gemini'] = gemini_dist
        
        # Try depth camera (3D point cloud) - now returns angle too
        depth_result = self.get_distance_from_depth(bbox_center_x, bbox_center_y)
        depth_dist = depth_result[0] if depth_result[0] is not None else None
        depth_angle = depth_result[1] if depth_result[1] is not None else None
        
        if depth_dist is not None and 0.3 < depth_dist < self.detection_distance:
            distances['depth'] = depth_dist
        
        # Try LiDAR (returns distance AND refined angle)
        lidar_result = self.get_distance_from_lidar(angle_offset)
        lidar_dist = lidar_result[0] if lidar_result[0] is not None else None
        lidar_angle = lidar_result[1] if lidar_result[1] is not None else None
        
        if lidar_dist is not None and 0.3 < lidar_dist < self.detection_distance:
            distances['lidar'] = lidar_dist
        
        # Choose best refined angle: prefer DEPTH over LiDAR (depth camera sees in 3D, less occlusion)
        if depth_angle is not None:
            refined_angle = depth_angle
        elif lidar_angle is not None:
            refined_angle = lidar_angle
        else:
            refined_angle = None
        
        # Fusion logic with occlusion handling
        if 'lidar' in distances and 'depth' in distances:
            lidar_d = distances['lidar']
            depth_d = distances['depth']
            gemini_d = distances['gemini']
            
            lidar_much_larger_than_depth = lidar_d > depth_d * 1.5
            lidar_much_larger_than_gemini = lidar_d > gemini_d * 1.5
            
            if lidar_much_larger_than_depth and lidar_much_larger_than_gemini:
                final_dist = depth_d
                method = 'depth(lidar_occluded)'
            elif abs(lidar_d - depth_d) < 0.5:
                final_dist = (lidar_d * 0.6 + depth_d * 0.4)
                method = 'fusion(lidar+depth)'
            elif depth_d < lidar_d:
                final_dist = depth_d
                method = 'depth(closer)'
            else:
                final_dist = lidar_d
                method = 'lidar(closer)'
                
        elif 'depth' in distances:
            final_dist = distances['depth']
            method = 'depth'
        elif 'lidar' in distances:
            if distances['lidar'] > distances['gemini'] * 2:
                final_dist = distances['gemini']
                method = 'gemini(lidar_suspect)'
            else:
                final_dist = distances['lidar']
                method = 'lidar'
        else:
            final_dist = distances['gemini']
            method = 'gemini(fallback)'
        
        return final_dist, method, refined_angle

    # ============ ANGLE CALCULATION ============
    
    def get_angle_from_bbox(self, bbox_center_x):
        """
        Calculate precise angle from bounding box center position.
        
        Args:
            bbox_center_x: Normalized x position (0-1) in image
                          0 = left edge, 0.5 = center, 1 = right edge
        
        Returns:
            angle_offset in radians (positive = left, negative = right)
        """
        # Convert normalized position to angle
        # bbox_center_x = 0.5 means center (angle = 0)
        # bbox_center_x = 0 means left edge (angle = +fov/2)
        # bbox_center_x = 1 means right edge (angle = -fov/2)
        
        angle_offset = (0.5 - bbox_center_x) * self.camera_fov_h
        return angle_offset
    
    def get_angle_from_position(self, position_str):
        """
        Original method: Convert categorical position to angle.
        Less precise than bounding box method.
        """
        if position_str == 'left':
            return 0.5  # ~30 degrees
        elif position_str == 'right':
            return -0.5
        else:
            return 0.0

    # ============ GEMINI ANALYSIS ============
    
    def analyze_scene(self):
        """Main analysis loop - runs at analysis_rate Hz."""
        if self.analyzing:
            return
        
        with self.image_lock:
            if self.latest_image is None:
                return
            image_msg = self.latest_image
        
        pose = self.get_robot_pose()
        if pose is None:
            return
        
        self.analyzing = True
        threading.Thread(
            target=self._analyze_with_gemini,
            args=(image_msg, pose),
            daemon=True
        ).start()
    
    def _analyze_with_gemini(self, image_msg, robot_pose):
        """Background thread for Gemini analysis with enhanced distance estimation."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Enhanced prompt requesting bounding boxes for precise positioning
            prompt = """Analyze this robot camera image for social navigation.

Detect all humans and for each one provide:
1. Bounding box: [x_min, y_min, x_max, y_max] as normalized values 0-1
   (0,0 is top-left, 1,1 is bottom-right)
2. Position in image: "left", "center", or "right" (as backup)
3. Estimated distance: "near" (<2m), "medium" (2-5m), or "far" (>5m)
4. Engagement level:
   - "high": People in conversation, interacting, facing each other
   - "medium": Person standing still, looking around
   - "low": Person walking, passing through, not engaged

Return ONLY valid JSON (no markdown):
{
  "humans_detected": true/false,
  "humans": [
    {
      "bbox": [x_min, y_min, x_max, y_max],
      "position": "left/center/right",
      "distance": "near/medium/far",
      "engagement": "high/medium/low",
      "activity": "brief description"
    }
  ]
}

If no humans: {"humans_detected": false, "humans": []}
"""
            
            response = self.model.generate_content([prompt, pil_image])
            text = response.text.strip()
            
            if '```json' in text:
                text = text.split('```json')[1].split('```')[0].strip()
            elif '```' in text:
                text = text.split('```')[1].split('```')[0].strip()
            
            result = json.loads(text)
            
            if result.get('humans_detected', False):
                humans = result.get('humans', [])
                self.get_logger().info(f'👁️  Detected {len(humans)} human(s)')
                self._create_obstacles_from_detections(humans, robot_pose)
            else:
                self.get_logger().debug('No humans detected')
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'JSON parse error: {e}')
        except Exception as e:
            self.get_logger().error(f'Analysis error: {e}')
        finally:
            self.analyzing = False
    
    def _create_obstacles_from_detections(self, humans, robot_pose):
        """Convert Gemini detections to obstacles with precise distance estimation."""
        rx, ry, ryaw = robot_pose
        now = time.time()
        
        self.get_logger().info(f'    ROBOT: pos=({rx:.2f}, {ry:.2f}), heading={math.degrees(ryaw):.1f}°')
        
        new_obstacles = []
        
        for i, human in enumerate(humans):
            # Get engagement level → radius
            # REDUCED RADII for testing precision requirements
            engagement = human.get('engagement', 'medium')
            if engagement == 'high':
                radius = 1.0  # Conversation - was 1.2m, now 1.0m
            elif engagement == 'medium':
                radius = 0.6  # Standing - was 0.8m, now 0.6m
            else:
                radius = 0.4  # Walking - was 0.5m, now 0.4m
            
            # Get bounding box or fall back to position
            bbox = human.get('bbox', None)
            position_str = human.get('position', 'center')
            distance_str = human.get('distance', 'medium')
            
            # Calculate angle (prefer bbox if available)
            if bbox and len(bbox) == 4:
                bbox_center_x = (bbox[0] + bbox[2]) / 2
                bbox_center_y = (bbox[1] + bbox[3]) / 2
                angle_offset = self.get_angle_from_bbox(bbox_center_x)
                angle_method = 'bbox'
                self.get_logger().info(
                    f'    DEBUG: bbox={bbox}, center_x={bbox_center_x:.3f}, angle={math.degrees(angle_offset):.1f}°'
                )
            else:
                bbox_center_x = 0.5  # Default center
                bbox_center_y = 0.5
                angle_offset = self.get_angle_from_position(position_str)
                angle_method = 'categorical'
            
            # Get distance based on selected method
            if self.distance_method == 'gemini':
                distance = self.get_distance_from_gemini(distance_str)
                dist_method = 'gemini'
                final_angle = angle_offset
            elif self.distance_method == 'depth':
                distance = self.get_distance_from_depth(bbox_center_x, bbox_center_y)
                if distance is None:
                    distance = self.get_distance_from_gemini(distance_str)
                    dist_method = 'gemini(fallback)'
                else:
                    dist_method = 'depth'
                final_angle = angle_offset
            elif self.distance_method == 'lidar':
                lidar_dist, lidar_angle = self.get_distance_from_lidar(angle_offset)
                if lidar_dist is None:
                    distance = self.get_distance_from_gemini(distance_str)
                    dist_method = 'gemini(fallback)'
                    final_angle = angle_offset
                else:
                    distance = lidar_dist
                    dist_method = 'lidar'
                    final_angle = lidar_angle if lidar_angle is not None else angle_offset
            else:  # fusion
                distance, dist_method, refined_angle = self.get_fused_distance(
                    bbox_center_x, bbox_center_y, angle_offset, distance_str
                )
                # Use refined LiDAR angle if available, otherwise use bbox angle
                final_angle = refined_angle if refined_angle is not None else angle_offset
            
            # Calculate obstacle position in map frame using final_angle
            obstacle_angle = ryaw + final_angle
            ox = rx + distance * math.cos(obstacle_angle)
            oy = ry + distance * math.sin(obstacle_angle)
            
            obstacle = {
                'x': ox,
                'y': oy,
                'radius': radius,
                'engagement': engagement,
                'activity': human.get('activity', 'unknown'),
                'expires_at': now + self.obstacle_ttl
            }
            new_obstacles.append(obstacle)
            
            # Enhanced logging with distance method
            self.get_logger().info(
                f'  Human {i+1}: {engagement} engagement | '
                f'distance={distance:.2f}m ({dist_method}) | '
                f'bbox_angle={math.degrees(angle_offset):.1f}° final_angle={math.degrees(final_angle):.1f}° | '
                f'map_pos=({ox:.2f}, {oy:.2f})'
            )
            self.get_logger().info(
                f'    CALC: robot({rx:.2f},{ry:.2f}) + dist({distance:.2f}) @ angle({math.degrees(ryaw):.1f}° + {math.degrees(final_angle):.1f}° = {math.degrees(obstacle_angle):.1f}°)'
            )
        
        # Update obstacle list
        with self.obstacles_lock:
            self.current_obstacles = [
                obs for obs in self.current_obstacles
                if obs['expires_at'] > now
            ]
            
            for new_obs in new_obstacles:
                is_duplicate = False
                for existing in self.current_obstacles:
                    dist = math.sqrt(
                        (new_obs['x'] - existing['x'])**2 +
                        (new_obs['y'] - existing['y'])**2
                    )
                    if dist < 1.0:
                        existing['expires_at'] = new_obs['expires_at']
                        existing['radius'] = max(existing['radius'], new_obs['radius'])
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    self.current_obstacles.append(new_obs)

    # ============ OBSTACLE PUBLISHING ============
    
    def publish_obstacles(self):
        """Publish obstacles at high frequency (10 Hz)."""
        now = time.time()
        
        with self.obstacles_lock:
            self.current_obstacles = [
                obs for obs in self.current_obstacles
                if obs['expires_at'] > now
            ]
            
            all_points = []
            for obstacle in self.current_obstacles:
                points = self._generate_cylinder_points(
                    obstacle['x'],
                    obstacle['y'],
                    obstacle['radius']
                )
                all_points.extend(points)
        
        cloud = self._create_pointcloud2(all_points)
        self.obstacle_pub.publish(cloud)
    
    def _generate_cylinder_points(self, x_center, y_center, radius):
        """Generate dense cylinder points."""
        points = []
        height = 1.8
        num_angles = 36
        num_radii = max(5, int(radius / 0.05))
        num_heights = 10
        
        for angle_idx in range(num_angles):
            angle = 2 * math.pi * angle_idx / num_angles
            for r_idx in range(num_radii + 1):
                r = radius * r_idx / num_radii
                x = x_center + r * math.cos(angle)
                y = y_center + r * math.sin(angle)
                for h_idx in range(num_heights + 1):
                    z = height * h_idx / num_heights
                    points.append([x, y, z])
        
        return points
    
    def _create_pointcloud2(self, points):
        """Create PointCloud2 message."""
        cloud = PointCloud2()
        cloud.header.frame_id = "map"
        cloud.header.stamp = self.get_clock().now().to_msg()
        
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.height = 1
        cloud.width = len(points) if points else 0
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        
        if points:
            cloud.data = b''.join(
                struct.pack('fff', float(p[0]), float(p[1]), float(p[2]))
                for p in points
            )
        else:
            cloud.data = b''
        
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = GeminiSocialNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()