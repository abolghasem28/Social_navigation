#!/usr/bin/env python3
"""
Gemini Social Navigator - AI-powered human detection and social navigation
Uses Gemini to analyze camera feed and create appropriate virtual obstacles

Author: Abolghasem Esmaeily
Date: December 2025

Usage:
  ros2 run yahboom_rosmaster_navigation social_navigation.py \
  or run launch_gemini_detector.sh
    --ros-args -p gemini_api_key:="YOUR_API_KEY"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
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
from rclpy.time import Time
from rclpy.duration import Duration


class GeminiSocialNavigator(Node):
    """
    Social navigation using Gemini AI for human detection and engagement analysis.
    
    This node:
    1. Captures camera images
    2. Sends to Gemini for human detection + engagement analysis
    3. Creates virtual obstacles based on engagement level:
       - HIGH engagement (conversation): Large radius (1.2m) - robot should go around
       - MEDIUM engagement (standing): Medium radius (0.8m)
       - LOW engagement (walking): Small radius (0.5m) - robot can pass closer
    4. Publishes obstacles as PointCloud2 for Nav2
    """
    
    def __init__(self):
        super().__init__('gemini_social_navigator')
        
        # ============ PARAMETERS ============
        self.declare_parameter('gemini_api_key', '')
        self.declare_parameter('camera_topic', '/cam_1/color/image_raw')
        self.declare_parameter('analysis_rate', 2.0)  # Gemini analysis rate (Hz)
        self.declare_parameter('obstacle_publish_rate', 10.0)  # Obstacle publish rate (Hz)
        self.declare_parameter('obstacle_ttl', 5.0)  # Obstacle time-to-live (seconds)
        self.declare_parameter('detection_distance', 5.0)  # Max distance to detect humans (meters)
        
        api_key = self.get_parameter('gemini_api_key').value
        camera_topic = self.get_parameter('camera_topic').value
        analysis_rate = self.get_parameter('analysis_rate').value
        publish_rate = self.get_parameter('obstacle_publish_rate').value
        self.obstacle_ttl = self.get_parameter('obstacle_ttl').value
        self.detection_distance = self.get_parameter('detection_distance').value
        
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
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # ============ PUBLISHERS ============
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/virtual_obstacles', 10)
        
        # ============ STATE ============
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.analyzing = False
        self.robot_moving = False
        
        # Obstacle storage
        self.current_obstacles = []  # List of obstacle dicts: {x, y, radius, engagement, expires_at}
        self.obstacles_lock = threading.Lock()
        
        # ============ TIMERS ============
        # Fast timer for publishing obstacles (10 Hz)
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_obstacles)
        
        # Slower timer for Gemini analysis (2 Hz)
        self.analysis_timer = self.create_timer(1.0 / analysis_rate, self.analyze_scene)
        
        # ============ STARTUP MESSAGE ============
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 GEMINI SOCIAL NAVIGATOR STARTED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📷 Camera topic: {camera_topic}')
        self.get_logger().info(f'🔍 Analysis rate: {analysis_rate} Hz')
        self.get_logger().info(f'📡 Publish rate: {publish_rate} Hz')
        self.get_logger().info(f'⏱️  Obstacle TTL: {self.obstacle_ttl} seconds')
        self.get_logger().info('')
        self.get_logger().info('Engagement → Obstacle Radius:')
        self.get_logger().info('  HIGH (conversation)  → 1.2m radius')
        self.get_logger().info('  MEDIUM (standing)    → 0.8m radius')
        self.get_logger().info('  LOW (walking)        → 0.5m radius')
        self.get_logger().info('=' * 60)

    # ============ CALLBACKS ============
    
    def camera_callback(self, msg):
        """Store latest camera image."""
        with self.image_lock:
            self.latest_image = msg
    
    def odom_callback(self, msg):
        """Track if robot is moving."""
        vel = msg.twist.twist
        speed = math.sqrt(vel.linear.x**2 + vel.linear.y**2)
        self.robot_moving = speed > 0.01

    # ============ ROBOT POSE ============
    
    def get_robot_pose(self):
        """Get robot pose in map frame using TF."""
        try:
            # Use tf2 time of 0 to get latest available transform
            from builtin_interfaces.msg import Time as TimeMsg
            time_msg = TimeMsg(sec=0, nanosec=0)
            
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', 
                time_msg,
                timeout=Duration(seconds=1.0))
            t = tf.transform.translation
            q = tf.transform.rotation
            
            # Calculate yaw from quaternion
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return float(t.x), float(t.y), float(yaw)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    # ============ GEMINI ANALYSIS ============
    
    def analyze_scene(self):
        """Main analysis loop - runs at analysis_rate Hz."""
        
        # Skip if already analyzing
        if self.analyzing:
            return
        
        # Get latest image
        with self.image_lock:
            if self.latest_image is None:
                return
            image_msg = self.latest_image
        
        # Get robot pose
        pose = self.get_robot_pose()
        if pose is None:
            return
        
        # Start analysis in background thread
        self.analyzing = True
        threading.Thread(
            target=self._analyze_with_gemini,
            args=(image_msg, pose),
            daemon=True
        ).start()
    
    def _analyze_with_gemini(self, image_msg, robot_pose):
        """Background thread for Gemini analysis."""
        try:
            # Convert ROS image to PIL
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Gemini prompt
            prompt = """Analyze this robot camera image for social navigation.

Detect all humans and for each one provide:
1. Position in image: "left", "center", or "right"
2. Estimated distance: "near" (<2m), "medium" (2-5m), or "far" (>5m)
3. Engagement level based on their activity:
   - "high": People in conversation, interacting, facing each other
   - "medium": Person standing still, looking around
   - "low": Person walking, passing through, not engaged

Return ONLY valid JSON (no markdown, no explanation):
{
  "humans_detected": true/false,
  "humans": [
    {
      "position": "left/center/right",
      "distance": "near/medium/far",
      "engagement": "high/medium/low",
      "activity": "brief description"
    }
  ]
}

If no humans visible, return: {"humans_detected": false, "humans": []}
"""
            
            # Call Gemini
            response = self.model.generate_content([prompt, pil_image])
            text = response.text.strip()
            
            # Parse JSON
            if '```json' in text:
                text = text.split('```json')[1].split('```')[0].strip()
            elif '```' in text:
                text = text.split('```')[1].split('```')[0].strip()
            
            result = json.loads(text)
            
            # Process detections
            if result.get('humans_detected', False):
                humans = result.get('humans', [])
                self.get_logger().info(f'👁️  Detected {len(humans)} human(s)')
                
                # Create obstacles from detections
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
        """Convert Gemini detections to obstacles in map frame."""
        rx, ry, ryaw = robot_pose
        now = time.time()
        
        new_obstacles = []
        
        for i, human in enumerate(humans):
            # Get engagement level → radius
            engagement = human.get('engagement', 'medium')
            if engagement == 'high':
                radius = 1.2  # Conversation - large buffer
            elif engagement == 'medium':
                radius = 0.8  # Standing - medium buffer
            else:
                radius = 0.5  # Walking - small buffer
            
            # Get distance estimate
            distance_str = human.get('distance', 'medium')
            if distance_str == 'near':
                distance = 1.5
            elif distance_str == 'medium':
                distance = 3.0
            else:
                distance = 5.0
            
            # Get lateral position
            position = human.get('position', 'center')
            if position == 'left':
                angle_offset = 0.5  # 30 degrees left
            elif position == 'right':
                angle_offset = -0.5  # 30 degrees right
            else:
                angle_offset = 0.0  # Center
            
            # Calculate obstacle position in map frame
            obstacle_angle = ryaw + angle_offset
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
            
            self.get_logger().info(
                f'  Human {i+1}: {engagement} engagement, '
                f'pos=({ox:.1f}, {oy:.1f}), radius={radius}m'
            )
        
        # Update obstacle list (merge with existing, avoid duplicates)
        with self.obstacles_lock:
            # Remove expired obstacles
            self.current_obstacles = [
                obs for obs in self.current_obstacles
                if obs['expires_at'] > now
            ]
            
            # Add new obstacles (checking for nearby existing ones)
            for new_obs in new_obstacles:
                # Check if similar obstacle already exists
                is_duplicate = False
                for existing in self.current_obstacles:
                    dist = math.sqrt(
                        (new_obs['x'] - existing['x'])**2 +
                        (new_obs['y'] - existing['y'])**2
                    )
                    if dist < 1.0:  # Within 1m = same obstacle
                        # Update existing obstacle
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
            # Remove expired obstacles
            self.current_obstacles = [
                obs for obs in self.current_obstacles
                if obs['expires_at'] > now
            ]
            
            # Generate points for all obstacles
            all_points = []
            for obstacle in self.current_obstacles:
                points = self._generate_cylinder_points(
                    obstacle['x'],
                    obstacle['y'],
                    obstacle['radius']
                )
                all_points.extend(points)
        
        # Create and publish PointCloud2
        cloud = self._create_pointcloud2(all_points)
        self.obstacle_pub.publish(cloud)
    
    def _generate_cylinder_points(self, x_center, y_center, radius):
        """Generate dense cylinder points (same as obstacle_manager)."""
        points = []
        height = 1.8  # Human height
        
        # Dense sampling for solid detection
        num_angles = 36  # Every 10 degrees
        num_radii = max(5, int(radius / 0.05))  # Every 5cm
        num_heights = 10  # 10 height levels
        
        for angle_idx in range(num_angles):
            angle = 2 * math.pi * angle_idx / num_angles
            
            # From center to edge
            for r_idx in range(num_radii + 1):
                r = radius * r_idx / num_radii
                
                x = x_center + r * math.cos(angle)
                y = y_center + r * math.sin(angle)
                
                # Multiple heights
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