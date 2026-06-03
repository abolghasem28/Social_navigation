#!/usr/bin/env python3
"""
NODE 2: Social Navigator (Step 3 & 4)
=====================================
Handles EKF tracking, zero-shot semantic analysis via Gemini, 
and dynamic Costmap generation for complex social scenarios.

In terminal 1:
ros2 run tf2_ros static_transform_publisher -0.10 0 0.052 0 -1.5708 0 lio_gripper_interface_link camera_link

In terminal 2:
ros2 run tf2_ros static_transform_publisher 0 0 0.5 0 0 0 LIO_base_link camera_link

In terminal 3:
python3 /home/aesmaeily/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_navigation/scripts/yolo_detector.py

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
import ast
import json
import threading
from concurrent.futures import ThreadPoolExecutor
import struct
import math
import time
import uuid 
import os

# EKF STABILIZER
class HumanTracker:
    def __init__(
        self,
        x,
        y,
        z,
        image_order=None,
        detection_seq=0,
        detection_confidence=0.0,
        box_confidence=0.0,
        left_shoulder_conf=0.0,
        right_shoulder_conf=0.0,
    ):
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

        # High Measurement Noise: Don't trust the camera 
        #self.R = np.eye(2) * 0.5  

        # For fast moving humans, we can set a higher velocity to be more responsive, like below.
        self.Q = np.diag([0.03, 0.03, 0.1, 0.1])
         
        # Low Measurement Noise: Trust more the camera between 0.1-0.5, Faster movement
        self.R = np.eye(2) * 0.6

        self.z = z
        self.image_order = image_order
        self.social_edges = {}
        #self.target_ids = []
        self.last_update = time.time()
        self.last_detection_seq = detection_seq
        self.last_detection_confidence = float(detection_confidence)
        self.last_box_confidence = float(box_confidence)
        self.last_left_shoulder_conf = float(left_shoulder_conf)
        self.last_right_shoulder_conf = float(right_shoulder_conf)
        self.last_visible_confirm_time = 0.0
        self.last_visible_pose = None
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
        self.state[2, 0] *= 0.95
        self.state[3, 0] *= 0.95 

    def update(
        self,
        measured_x,
        measured_y,
        measured_z,
        image_order=None,
        detection_seq=None,
        detection_confidence=None,
        box_confidence=None,
        left_shoulder_conf=None,
        right_shoulder_conf=None,
    ):
        self.last_update = time.time()
        if image_order is not None:
            self.image_order = image_order
        if detection_seq is not None:
            self.last_detection_seq = detection_seq
        if detection_confidence is not None:
            self.last_detection_confidence = float(detection_confidence)
        if box_confidence is not None:
            self.last_box_confidence = float(box_confidence)
        if left_shoulder_conf is not None:
            self.last_left_shoulder_conf = float(left_shoulder_conf)
        if right_shoulder_conf is not None:
            self.last_right_shoulder_conf = float(right_shoulder_conf)
        
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
        self.declare_parameter('gemini_timeout_sec', 20.0)
        self.declare_parameter('gemini_max_retries', 1)
        self.declare_parameter('gemini_retry_delay_sec', 2.0)
        self.declare_parameter('tracker_ttl_sec', 5.0)
        self.declare_parameter('social_obstacle_ttl_sec', 5.0)
        self.declare_parameter('red_wall_hold_sec', 30.0)
        self.declare_parameter('visible_human_conf_threshold', 0.6)
        self.declare_parameter('visible_human_grace_sec', 0.5)
        self.api_key = self.get_parameter('gemini_api_key').value


        if not self.api_key:
            self.api_key = os.getenv('GEMINI_API_KEY', '')
            if not self.api_key:
                self.get_logger().error("MISSING GEMINI API KEY.")
                return

        self.api_key = str(self.api_key).strip()
        if not self.api_key:
            self.get_logger().error("MISSING GEMINI API KEY.")
            return

        self.gemini_timeout_sec = float(self.get_parameter('gemini_timeout_sec').value)
        self.gemini_max_retries = max(0, int(self.get_parameter('gemini_max_retries').value))
        self.gemini_retry_delay_sec = max(0.0, float(self.get_parameter('gemini_retry_delay_sec').value))
        self.tracker_ttl_sec = max(0.0, float(self.get_parameter('tracker_ttl_sec').value))
        self.social_obstacle_ttl_sec = max(
            0.0, float(self.get_parameter('social_obstacle_ttl_sec').value)
        )
        self.red_wall_hold_sec = max(
            0.0, float(self.get_parameter('red_wall_hold_sec').value)
        )
        self.visible_human_conf_threshold = float(
            self.get_parameter('visible_human_conf_threshold').value
        )
        self.visible_human_grace_sec = max(
            0.0, float(self.get_parameter('visible_human_grace_sec').value)
        )
        self.transform_cache_ttl_sec = max(
            5.0,
            self.tracker_ttl_sec,
            self.social_obstacle_ttl_sec,
            self.red_wall_hold_sec,
        )
            
        # TOPICS
        self.rgb_topic = '/annotated_image'  # From YOLO Detector for bouding box and IDs
        self.pose_topic = '/detected_humans'
        #self.rgb_topic = '/camera/camera/color/image_raw'
        self.map_frame = 'map'
        self.target_frame = 'LIO_base_link' 
        self.obstacle_cache_frame = self.map_frame

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
        self.gemini_request_counter = 0
        self.last_gemini_wait_reason = None
        self.pose_wait_status = "No pose messages received yet."
        self.last_pose_wait_status_logged = None
        self.thread_executor = ThreadPoolExecutor(max_workers=1)
        self.social_state_lock = threading.RLock()
        self.tf_cache_lock = threading.RLock()
        self.persisted_walls = {}
        self.cached_transforms = {}
        self.pose_sequence = 0

        self.create_timer(0.1, self.analyse_loop)
        # self.create_timer(1.0, self.log_map_to_base_transform)
        self.create_timer(1.0, self.trigger_gemini)
        
        self.get_logger().info("Node 2 Started. Waiting for data to generate report...")

    def pose_cb(self, msg):
        self.pose_sequence += 1
        pose_sequence = self.pose_sequence
        frame_id = msg.header.frame_id
        detections = []
        transform_error = None

        if not frame_id:
            self.set_pose_wait_status(
                f"Received PoseArray on {self.pose_topic} without a header.frame_id.",
                warn=True,
            )
            self.update_ekf([], pose_sequence)
            return

        if not msg.poses:
            self.set_pose_wait_status(
                f"Received an empty PoseArray on {self.pose_topic}; waiting for a human with valid depth."
            )
            self.update_ekf([], pose_sequence)
            return

        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            pose_time = rclpy.time.Time()
        else:
            pose_time = rclpy.time.Time.from_msg(msg.header.stamp)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                pose_time,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as exc:
            self.set_pose_wait_status(
                (
                    f"Received {len(msg.poses)} pose(s) on {self.pose_topic}, but TF could not "
                    f"transform from '{frame_id}' to '{self.target_frame}' at the pose timestamp: {exc}"
                ),
                warn=True,
            )
            self.update_ekf([], pose_sequence)
            return

        for image_order, pose in enumerate(msg.poses):
            pt = PointStamped()
            pt.header.frame_id = frame_id
            pt.header.stamp = msg.header.stamp
            pt.point.x, pt.point.y, pt.point.z = pose.position.x, pose.position.y, pose.position.z
            try:
                detection_confidence = float(pose.orientation.w)
                box_confidence = float(pose.orientation.x)
                left_shoulder_conf = float(pose.orientation.y)
                right_shoulder_conf = float(pose.orientation.z)
                if (
                    detection_confidence == 0.0
                    and box_confidence == 0.0
                    and left_shoulder_conf == 0.0
                    and right_shoulder_conf == 0.0
                ):
                    detection_confidence = 1.0
                    box_confidence = 1.0

                out = do_transform_point(pt, transform)
                detections.append({
                    'x': out.point.x,
                    'y': out.point.y,
                    'z': out.point.z,
                    'image_order': image_order,
                    'detection_confidence': detection_confidence if detection_confidence > 0.0 else box_confidence,
                    'box_confidence': box_confidence,
                    'left_shoulder_conf': left_shoulder_conf,
                    'right_shoulder_conf': right_shoulder_conf,
                })
            except Exception as exc:
                if transform_error is None:
                    transform_error = exc

        if detections:
            self.clear_pose_wait_status()
        else:
            status = (
                f"Received {len(msg.poses)} pose(s) on {self.pose_topic}, but none could be "
                f"transformed into '{self.target_frame}'."
            )
            if transform_error is not None:
                status = f"{status} Example TF error: {transform_error}"
            self.set_pose_wait_status(status, warn=True)

        self.update_ekf(detections, pose_sequence)

    def log_map_to_base_transform(self):
        try:
            transform = self.lookup_transform_cached(self.map_frame, self.target_frame)
            t = transform.transform.translation
            r = transform.transform.rotation
            self.get_logger().info(
                f"TF {self.map_frame} -> {self.target_frame} | "
                f"xyz=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) | "
                f"xyzw=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})"
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not get TF {self.map_frame} -> {self.target_frame}: {e}"
            )

    def rgb_cb(self, msg):
        self.latest_rgb = msg

    def set_pose_wait_status(self, status, warn=False):
        self.pose_wait_status = status
        if status != self.last_pose_wait_status_logged:
            if warn:
                self.get_logger().warn(status)
            else:
                self.get_logger().info(status)
            self.last_pose_wait_status_logged = status

    def clear_pose_wait_status(self):
        self.pose_wait_status = None
        self.last_pose_wait_status_logged = None

    def log_gemini_wait_reason(self, reason):
        if reason != self.last_gemini_wait_reason:
            self.get_logger().info(reason)
            self.last_gemini_wait_reason = reason

    def get_trackers_in_image_order(self):
        # Old logic kept for reference. Rebuilding left/right order from robot-frame Y
        # can disagree with the actual IDs drawn in the annotated image.
        # return sorted(self.trackers, key=lambda t: float(t.state[1, 0]), reverse=True)
        return sorted(
            self.trackers,
            key=lambda t: (
                t.image_order if t.image_order is not None else 10**9,
                float(t.state[1, 0]),
            ),
        )

    def lookup_transform_cached(self, target_frame, source_frame, timeout_sec=0.2):
        cache_key = (target_frame, source_frame)

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            with self.tf_cache_lock:
                self.cached_transforms[cache_key] = {
                    'transform': transform,
                    'updated_at': time.time(),
                }
            return transform
        except Exception as exc:
            with self.tf_cache_lock:
                cached = self.cached_transforms.get(cache_key)

            if cached is not None:
                age = time.time() - cached['updated_at']
                if age <= self.transform_cache_ttl_sec:
                    return cached['transform']

            raise exc

    def choose_publish_frame(self, active_persisted_walls=None):
        if active_persisted_walls and any(
            wall.get('latched') and wall.get('frame_id') == self.map_frame
            for wall in active_persisted_walls.values()
        ):
            return self.map_frame

        if self.obstacle_cache_frame == self.target_frame:
            return self.target_frame

        try:
            self.lookup_transform_cached(
                self.obstacle_cache_frame,
                self.target_frame,
                timeout_sec=0.05,
            )
            return self.obstacle_cache_frame
        except Exception:
            return self.target_frame

    def transform_point(self, x, y, z, from_frame, to_frame):
        if from_frame == to_frame:
            return (float(x), float(y), float(z))

        pt = PointStamped()
        pt.header.frame_id = from_frame
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)

        try:
            transform = self.lookup_transform_cached(to_frame, from_frame, timeout_sec=0.2)
            out = do_transform_point(pt, transform)
            return (float(out.point.x), float(out.point.y), float(out.point.z))
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to transform point from {from_frame} to {to_frame}: {exc}"
            )
            return None

    def wall_marker_id(self, pair_key):
        return ((pair_key[0] * 1000003) ^ pair_key[1]) & 0xFFFFFF

    def freeze_wall(self, pair_key, tracker_a, tracker_b, can_cross, now=None):
        if self.red_wall_hold_sec <= 0.0:
            return False

        if now is None:
            now = time.time()

        with self.social_state_lock:
            existing = self.persisted_walls.get(pair_key)
            if (
                existing is not None
                and existing.get('latched')
                and existing.get('expires_at', 0.0) > now
            ):
                return True

        pose_a = tracker_a.last_visible_pose or (
            float(tracker_a.state[0, 0]),
            float(tracker_a.state[1, 0]),
            float(tracker_a.z),
        )
        pose_b = tracker_b.last_visible_pose or (
            float(tracker_b.state[0, 0]),
            float(tracker_b.state[1, 0]),
            float(tracker_b.z),
        )

        p1 = self.transform_point(pose_a[0], pose_a[1], pose_a[2], self.target_frame, self.map_frame)
        p2 = self.transform_point(pose_b[0], pose_b[1], pose_b[2], self.target_frame, self.map_frame)
        if p1 is None or p2 is None:
            return False

        with self.social_state_lock:
            existing = self.persisted_walls.get(pair_key)
            if (
                existing is not None
                and existing.get('latched')
                and existing.get('expires_at', 0.0) > now
            ):
                return True

            is_blocked = float(can_cross) < 0.5
            self.persisted_walls[pair_key] = {
                'can_cross': float(can_cross),
                'expires_at': now + self.red_wall_hold_sec,
                'frame_id': self.map_frame,
                'latched': True,
                'start': (float(p1[0]), float(p1[1])),
                'end': (float(p2[0]), float(p2[1])),
                'thickness': 0.4 if is_blocked else 0.05,
            }

        return True

    def refresh_persisted_wall(self, pair_key, can_cross, now=None):
        if now is None:
            now = time.time()

        with self.social_state_lock:
            existing = self.persisted_walls.get(pair_key)
            if (
                existing is not None
                and existing.get('latched')
                and existing.get('expires_at', 0.0) > now
            ):
                return

            if self.social_obstacle_ttl_sec <= 0.0:
                self.persisted_walls.pop(pair_key, None)
                return

            self.persisted_walls[pair_key] = {
                'can_cross': float(can_cross),
                'expires_at': now + self.social_obstacle_ttl_sec,
            }

    def get_active_persisted_walls(self):
        now = time.time()

        with self.social_state_lock:
            expired_keys = [
                pair_key
                for pair_key, wall in self.persisted_walls.items()
                if wall['expires_at'] <= now
            ]
            for pair_key in expired_keys:
                del self.persisted_walls[pair_key]

            return {
                pair_key: dict(wall)
                for pair_key, wall in self.persisted_walls.items()
            }

    def parse_social_edge(self, edge_state):
        if isinstance(edge_state, dict):
            try:
                can_cross = float(edge_state['can_cross'])
                updated_at = float(edge_state['updated_at'])
            except (KeyError, TypeError, ValueError):
                return None
            return can_cross, updated_at

        try:
            return float(edge_state), 0.0
        except (TypeError, ValueError):
            return None

    def tracker_is_recently_visible(self, tracker, now=None):
        if tracker.last_visible_pose is None:
            return False

        if now is None:
            now = time.time()

        return (now - tracker.last_visible_confirm_time) <= self.visible_human_grace_sec

    def apply_social_links(self, social_links):
        # Old logic kept for reference:
        # sorted_trackers = sorted(self.trackers, key=lambda t: float(t.state[1, 0]), reverse=True)
        sorted_trackers = self.get_trackers_in_image_order()
        update_time = time.time()
        pending_edges = []
        pending_walls = []

        for link in social_links:
            pair = link.get('pair', [])
            can_cross = link.get('robot_can_cross', 1.0)
            reason = link.get('reason', 'No reason given')

            if len(pair) != 2:
                self.get_logger().warn(f"Ignored malformed pair: {pair}")
                continue

            try:
                id_A, id_B = self.normalize_pair_ids(pair, len(sorted_trackers))
            except ValueError as exc:
                self.get_logger().warn(str(exc))
                continue

            try:
                prob_cross = float(can_cross)
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f"Ignored pair {pair} because robot_can_cross is not numeric: {can_cross}"
                )
                continue

            self.get_logger().info(
                f"Mapped Pair [{id_A}, {id_B}] | Can Cross: {prob_cross:.2f} | Reason: {reason}"
            )

            if 0 <= id_A < len(sorted_trackers) and 0 <= id_B < len(sorted_trackers):
                tracker_A = sorted_trackers[id_A]
                tracker_B = sorted_trackers[id_B]
                pair_key = tuple(sorted((tracker_A.id, tracker_B.id)))
                both_visible = (
                    self.tracker_is_recently_visible(tracker_A, update_time)
                    and self.tracker_is_recently_visible(tracker_B, update_time)
                )
                if both_visible:
                    pending_edges.append((tracker_A, tracker_B, prob_cross, update_time))
                    pending_walls.append((pair_key, prob_cross, tracker_A, tracker_B))

                self.get_logger().info(
                    f"Edge [{id_A}-{id_B}] | Tracker IDs [{tracker_A.id}-{tracker_B.id}] "
                    f"| Cross Prob: {prob_cross:.2f} | Reason: {reason}"
                )
            else:
                self.get_logger().warn(
                    f"Ignored out-of-bounds pair: {pair} for {len(sorted_trackers)} trackers."
                )

        with self.social_state_lock:
            for tracker in sorted_trackers:
                tracker.social_edges = {}

            for tracker_A, tracker_B, prob_cross, edge_time in pending_edges:
                edge_state = {
                    'can_cross': prob_cross,
                    'updated_at': edge_time,
                }
                tracker_A.social_edges[tracker_B.id] = edge_state
                tracker_B.social_edges[tracker_A.id] = edge_state

        for pair_key, prob_cross, tracker_A, tracker_B in pending_walls:
            if self.freeze_wall(
                pair_key,
                tracker_A,
                tracker_B,
                prob_cross,
                now=update_time,
            ):
                continue

            self.refresh_persisted_wall(pair_key, prob_cross, now=update_time)

    def extract_gemini_text(self, response):
        diagnostics = []

        try:
            text = response.text
            if text:
                return text.strip(), ""
        except Exception as exc:
            diagnostics.append(f"response.text unavailable: {exc}")

        candidates = getattr(response, 'candidates', None) or []
        parts_text = []
        finish_reasons = []

        for candidate in candidates:
            finish_reason = getattr(candidate, 'finish_reason', None)
            if finish_reason is not None:
                finish_reasons.append(str(finish_reason))

            content = getattr(candidate, 'content', None)
            parts = getattr(content, 'parts', None) or []
            for part in parts:
                part_text = getattr(part, 'text', None)
                if part_text:
                    parts_text.append(part_text)

        if parts_text:
            return "\n".join(parts_text).strip(), ""

        prompt_feedback = getattr(response, 'prompt_feedback', None)
        if prompt_feedback:
            diagnostics.append(f"prompt_feedback={prompt_feedback}")
        if finish_reasons:
            diagnostics.append(f"finish_reasons={finish_reasons}")

        return "", "; ".join(diagnostics) if diagnostics else "Gemini returned no text parts."

    def parse_gemini_json(self, text):
        candidates = []
        stripped = text.strip()

        if stripped:
            candidates.append(stripped)

        start_idx = stripped.find('{')
        end_idx = stripped.rfind('}')
        if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
            json_slice = stripped[start_idx:end_idx + 1]
            if json_slice not in candidates:
                candidates.append(json_slice)

        errors = []
        for candidate in candidates:
            try:
                return json.loads(candidate)
            except json.JSONDecodeError as exc:
                errors.append(f"json.loads failed: {exc}")

            try:
                parsed = ast.literal_eval(candidate)
                if isinstance(parsed, dict):
                    return parsed
            except (ValueError, SyntaxError) as exc:
                errors.append(f"literal_eval failed: {exc}")

        raise ValueError("Unable to parse Gemini JSON. " + " | ".join(errors[:2]))

    def response_to_social_links(self, response, request_id, attempt, elapsed):
        text, diagnostics = self.extract_gemini_text(response)
        if not text:
            raise ValueError(diagnostics or "empty Gemini response")

        self.get_logger().info(
            f"RAW GEMINI OUTPUT (request {request_id}, attempt {attempt}, {elapsed:.1f}s):\n{text}"
        )

        data = self.parse_gemini_json(text)
        if not isinstance(data, dict):
            raise ValueError(f"Gemini returned {type(data).__name__}, expected a JSON object.")

        social_links = data.get('social_links', [])
        if not isinstance(social_links, list):
            raise ValueError("Gemini JSON must contain a list in 'social_links'.")

        return social_links

    def normalize_pair_ids(self, pair, tracker_count):
        if len(pair) != 2:
            raise ValueError(f"Expected 2 ids, got {pair}")

        ids = [int(pair[0]), int(pair[1])]

        if all(1 <= idx <= tracker_count for idx in ids):
            return ids[0] - 1, ids[1] - 1

        if all(0 <= idx < tracker_count for idx in ids):
            return ids[0], ids[1]

        raise ValueError(f"Pair {pair} is out of bounds for {tracker_count} trackers")

    def update_ekf(self, detections, detection_seq=None):
        for t in self.trackers: 
            t.predict()

        used_trackers = set()
        for det in detections:
            best_t = None
            min_dist = 5.0 
            
            for t in self.trackers:
                if t in used_trackers: 
                    continue
                dist = math.sqrt((t.state[0, 0]-det['x'])**2 + (t.state[1, 0]-det['y'])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_t = t
            
            if best_t:
                best_t.update(
                    det['x'],
                    det['y'],
                    det['z'],
                    det.get('image_order'),
                    detection_seq=detection_seq,
                    detection_confidence=det.get('detection_confidence'),
                    box_confidence=det.get('box_confidence'),
                    left_shoulder_conf=det.get('left_shoulder_conf'),
                    right_shoulder_conf=det.get('right_shoulder_conf'),
                )
                if best_t.last_detection_confidence >= self.visible_human_conf_threshold:
                    best_t.last_visible_confirm_time = best_t.last_update
                    best_t.last_visible_pose = (
                        float(best_t.state[0, 0]),
                        float(best_t.state[1, 0]),
                        float(best_t.z),
                    )
                used_trackers.add(best_t)
                
                # PRINT REPORT EVERY 100 FRAMES
                if best_t.frame_count == 100:
                    self.print_stability_report(best_t)
                    best_t.history_raw = []
                    best_t.history_ekf = []
                    best_t.frame_count = 0
            else:
                self.trackers.append(
                    HumanTracker(
                        det['x'],
                        det['y'],
                        det['z'],
                        det.get('image_order'),
                        detection_seq=detection_seq if detection_seq is not None else 0,
                        detection_confidence=det.get('detection_confidence', 0.0),
                        box_confidence=det.get('box_confidence', 0.0),
                        left_shoulder_conf=det.get('left_shoulder_conf', 0.0),
                        right_shoulder_conf=det.get('right_shoulder_conf', 0.0),
                    )
                )
                if self.trackers[-1].last_detection_confidence >= self.visible_human_conf_threshold:
                    self.trackers[-1].last_visible_confirm_time = self.trackers[-1].last_update
                    self.trackers[-1].last_visible_pose = (
                        float(self.trackers[-1].state[0, 0]),
                        float(self.trackers[-1].state[1, 0]),
                        float(self.trackers[-1].z),
                    )

        #for t in self.trackers:
         #   if t not in used_trackers: t.predict()
        
        # Keep tracker memory alive for the configured TTL.
        now = time.time()
        self.trackers = [t for t in self.trackers if (now - t.last_update) < self.tracker_ttl_sec]

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
        #print(f"  Z: ±{std_ekf[2]:.4f} m Depth ")
        print("-" * 50)
        #print("This proves (Depth) is noisy and (EKF) fixes it.")
        print("="*50 + "\n")

    def trigger_gemini(self):
        if self.gemini_busy:
            return

        if self.latest_rgb is None:
            self.log_gemini_wait_reason(
                f"Gemini waiting for annotated images on {self.rgb_topic}."
            )
            return

        if not self.trackers:
            wait_reason = f"Gemini waiting for tracked humans on {self.pose_topic}."
            if self.pose_wait_status:
                wait_reason = f"{wait_reason} {self.pose_wait_status}"
            self.log_gemini_wait_reason(wait_reason)
            return

        self.last_gemini_wait_reason = None
        self.gemini_busy = True
        img_copy = self.latest_rgb
        self.gemini_request_counter += 1
        request_id = self.gemini_request_counter
        tracker_count = len(self.trackers)
        self.get_logger().info(
            f"Starting Gemini request {request_id} with {tracker_count} tracked humans."
        )
        self.thread_executor.submit(self.process_gemini, request_id, img_copy)

    def process_gemini(self, request_id, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
            pil_img = pil_img.resize((640, 480)) 

            #self.get_logger().info(f"Triggering Gemini... Humans in memory: {len(self.trackers)}")



            prompt = """
                You are the vision system for a social navigation robot. Evaluate the invisible social boundaries between humans marked with numbered bounding boxes.

                RULES:
                1. If 0 person is detected, return exactly: {"social_links": []}
                2. If 1 person is detected, return exactly: {"social_links": [], "person": [1]}
                3. Use the exact IDs drawn in the image.
                4. If 2 or more people are detected, evaluate EVERY pair combination (e.g., [1,2], [2,3], [1,3]).

                You MUST return a valid JSON object matching this exact structure:
                {
                "social_links": [
                {
                "pair": [1, 2],
                "engagement": "high",
                "robot_can_cross": 0.1,
                "reason": "Individuals are standing close and facing each other, indicating active conversation."
                }
                ]
                }
                """

            try:
                generation_config = genai.GenerationConfig(
                    response_mime_type="application/json"
                )
            except Exception:
                generation_config = {"response_mime_type": "application/json"}

            max_attempts = self.gemini_max_retries + 1
            social_links = None
            last_issue = ""

            for attempt in range(1, max_attempts + 1):
                start_time = time.time()

                try:
                    response = self.model.generate_content(
                        [prompt, pil_img],
                        generation_config=generation_config,
                        request_options={"timeout": self.gemini_timeout_sec},
                    )
                    elapsed = time.time() - start_time
                    social_links = self.response_to_social_links(
                        response, request_id, attempt, elapsed
                    )
                    break
                except Exception as exc:
                    elapsed = time.time() - start_time
                    last_issue = str(exc)

                    if attempt == max_attempts:
                        raise

                    self.get_logger().warn(
                        f"Gemini request {request_id} failed on attempt {attempt}/{max_attempts} "
                        f"after {elapsed:.1f}s: {exc}. Retrying in {self.gemini_retry_delay_sec:.1f}s."
                    )

                if attempt < max_attempts and self.gemini_retry_delay_sec > 0.0:
                    time.sleep(self.gemini_retry_delay_sec)

            if social_links is None:
                self.get_logger().warn(
                    f"Gemini request {request_id} ended without a usable response. "
                    f"Last issue: {last_issue}"
                )
                return

            self.apply_social_links(social_links)

            self.get_logger().info(
                f"Gemini request {request_id} produced {len(social_links)} social links."
            )
        
        except Exception as e:
            self.get_logger().error(f"Gemini request {request_id} error: {e}")
        finally: 
            self.gemini_busy = False

    def analyse_loop(self):
        # Generate visualization markers and point cloud for obstacles
        marker_array = MarkerArray()
        cloud_points = []
        now = time.time()
        active_persisted_walls = self.get_active_persisted_walls()
        publish_frame = self.choose_publish_frame(active_persisted_walls)
        
        delete_m = Marker()
        delete_m.action = Marker.DELETEALL
        marker_array.markers.append(delete_m)
        # Old logic kept for reference:
        # sorted_trackers = sorted(self.trackers, key=lambda t: float(t.state[1, 0]), reverse=True)
        sorted_trackers = self.get_trackers_in_image_order()
        tracker_by_id = {t.id: t for t in sorted_trackers}
        tracker_positions = {}
        visible_tracker_positions = {}
        drawn_pairs = set()

        # Single cylinder for each human
        for t in sorted_trackers:
            pos = self.transform_point(
                float(t.state[0, 0]),
                float(t.state[1, 0]),
                0.0,
                self.target_frame,
                publish_frame,
            )
            if pos is None:
                continue

            tracker_positions[t.id] = pos
            if t.last_visible_pose is None:
                continue

            if (now - t.last_visible_confirm_time) > self.visible_human_grace_sec:
                continue

            visible_pos = self.transform_point(
                t.last_visible_pose[0],
                t.last_visible_pose[1],
                t.last_visible_pose[2],
                self.target_frame,
                publish_frame,
            )
            if visible_pos is None:
                continue

            visible_tracker_positions[t.id] = visible_pos
            m = Marker()
            m.header.frame_id = publish_frame; m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "humans"; m.id = t.id; m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = visible_pos[0]; m.pose.position.y = visible_pos[1]; m.pose.position.z = 0.9

            # Standard personal space for all humans
            m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 1.8 
            m.color.r = 0.0; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.8
            marker_array.markers.append(m)
            self.add_cylinder_points(cloud_points, visible_pos[0], visible_pos[1], 0.25)

        for pair_key, wall in active_persisted_walls.items():
            if wall.get('latched'):
                start = wall.get('start')
                end = wall.get('end')
                if start is None or end is None:
                    continue

                drawn_pairs.add(pair_key)
                can_cross = float(wall.get('can_cross', 1.0))
                is_blocked = can_cross < 0.5
                self.draw_wall_segment(
                    marker_array,
                    cloud_points if is_blocked else [],
                    start[0],
                    start[1],
                    end[0],
                    end[1],
                    color=(1.0, 0.0, 0.0) if is_blocked else (0.0, 1.0, 0.0),
                    thickness=float(wall.get('thickness', 0.4 if is_blocked else 0.05)),
                    frame_id=wall.get('frame_id', self.map_frame),
                    marker_id=self.wall_marker_id(pair_key),
                )
                continue

            tracker_a = tracker_by_id.get(pair_key[0])
            tracker_b = tracker_by_id.get(pair_key[1])
            if tracker_a is None or tracker_b is None:
                continue

            if not (
                self.tracker_is_recently_visible(tracker_a, now)
                and self.tracker_is_recently_visible(tracker_b, now)
            ):
                continue

            p1 = visible_tracker_positions.get(tracker_a.id)
            p2 = visible_tracker_positions.get(tracker_b.id)
            if p1 is None or p2 is None:
                continue

            drawn_pairs.add(pair_key)
            can_cross = float(wall.get('can_cross', 1.0))

            if can_cross < 0.5:
                self.draw_wall_segment(
                    marker_array,
                    cloud_points,
                    p1[0],
                    p1[1],
                    p2[0],
                    p2[1],
                    color=(1.0, 0.0, 0.0),
                    thickness=0.4,
                    frame_id=publish_frame,
                    marker_id=self.wall_marker_id(pair_key),
                )
            else:
                self.draw_wall_segment(
                    marker_array,
                    [],
                    p1[0],
                    p1[1],
                    p2[0],
                    p2[1],
                    color=(0.0, 1.0, 0.0),
                    thickness=0.05,
                    frame_id=publish_frame,
                    marker_id=self.wall_marker_id(pair_key),
                )

        self.marker_pub.publish(marker_array)
        if cloud_points: 
            self.publish_cloud(b''.join(cloud_points), len(cloud_points), publish_frame)
        else: 
            self.publish_cloud(b'', 0, publish_frame)


    def draw_wall_segment(
        self,
        marker_array,
        cloud_points,
        x1,
        y1,
        x2,
        y2,
        color,
        thickness=0.2,
        frame_id=None,
        marker_id=0,
    ):
        if frame_id is None:
            frame_id = self.target_frame

        wall_m = Marker()
        wall_m.header.frame_id = frame_id; wall_m.header.stamp = self.get_clock().now().to_msg()
        wall_m.ns = "semantic_walls"
        wall_m.id = marker_id
        wall_m.type = Marker.CUBE; wall_m.action = Marker.ADD

        dx = float(x2) - float(x1)
        dy = float(y2) - float(y1)
        wall_m.pose.position.x = float(x1) + (dx / 2.0)
        wall_m.pose.position.y = float(y1) + (dy / 2.0)
        wall_m.pose.position.z = 0.9 
        
        dist = math.sqrt(dx*dx + dy*dy)
        if dist <= 1e-6:
            return

        angle = math.atan2(dy, dx)
        wall_m.pose.orientation.z = math.sin(angle / 2.0)
        wall_m.pose.orientation.w = math.cos(angle / 2.0)

        wall_m.scale.x = dist
        wall_m.scale.y = thickness
        wall_m.scale.z = 1.8
        wall_m.color.r, wall_m.color.g, wall_m.color.b = color
        wall_m.color.a = 0.5
        marker_array.markers.append(wall_m)

        # Only generate obstacle points if the cloud_points list is provided (i.e., for blocked pairs)
        if cloud_points is not None:
            steps = max(1, int(dist / 0.1))
            for s in range(steps + 1):
                r = s / float(steps)
                px = float(x1) + (r * dx)
                py = float(y1) + (r * dy)
                self.add_cylinder_points(cloud_points, px, py, thickness / 2.0)

    def draw_wall(self, marker_array, cloud_points, t1, t2, dx, dy, color, thickness=0.2, is_frustum=False):
        pair_key = tuple(sorted((t1.id, t2.id if t2 else t1.id)))
        self.draw_wall_segment(
            marker_array,
            cloud_points,
            float(t1.state[0, 0]),
            float(t1.state[1, 0]),
            float(t1.state[0, 0]) + float(dx),
            float(t1.state[1, 0]) + float(dy),
            color=color,
            thickness=thickness,
            frame_id=self.target_frame,
            marker_id=self.wall_marker_id(pair_key),
        )



    # def transform_point(self, x, y, z, from_frame, time_stamp):
    #     if self.target_frame == from_frame: 
    #         return (x,y,z)
    #     pt = PointStamped()
    #     pt.header.frame_id = from_frame
    #     pt.header.stamp = self.get_clock().now().to_msg()
    #     pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
    #     try:
    #         out = self.tf_buffer.transform(pt, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.2))
    #         return (out.point.x, out.point.y, out.point.z)
    #     except Exception as e:
    #         self.get_logger().warn(f"Real TF Error: {e}")
    #         return None

    def add_cylinder_points(self, cloud_points, cx, cy, radius):
        for h_idx in range(5):
            z = 0.2 + (h_idx * 0.3)
            for i in range(8):
                angle = (2 * math.pi * i) / 8
                cloud_points.append(struct.pack('fff', cx + radius * math.cos(angle), cy + radius * math.sin(angle), z))

    def publish_cloud(self, data_bytes, num_points, frame_id=None):
        if frame_id is None:
            frame_id = self.target_frame

        cloud = PointCloud2()
        cloud.header.frame_id = frame_id
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
