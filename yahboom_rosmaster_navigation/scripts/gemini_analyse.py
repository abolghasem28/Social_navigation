#!/usr/bin/env python3
"""
Gemini Static Analyzer (Step 3)
===============================
1. CAPTURES: Take raw frames from the RealSense camera.
2. ANALYZES: Sends each frame to Gemini to detect humans.
3. OUTPUTS: 
   - Draws Bounding Boxes (Green).
   - Draws Keypoints (Red Dot on Chest).
   - Prints Confidence scores.
In order to run this code, ensure you have:
- A valid Gemini API key set in your environment (export GEMINI_API_KEY="your_key_here").

In termin 1:
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true

In terminal 2:
ros2 run yahboom_rosmaster_navigation gemini_analyse.py

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import google.generativeai as genai
from PIL import Image as PILImage
import json
import time
import os

class GeminiStaticTest(Node):
    def __init__(self):
        super().__init__('gemini_static_test')
        
        # --- CONFIGURATION ---
        self.target_frame_count = 20
        self.save_dir = "/home/aesmaeily/ros2_ws/src/test_images/gemini_analyse"
        self.api_key = os.getenv('GEMINI_API_KEY')
        self.rgb_topic = '/camera/camera/color/image_raw'

        if not self.api_key:
            self.get_logger().error("MISSING GEMINI API KEY. Please export it.")
            return

        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel('gemini-2.0-flash')
        self.bridge = CvBridge()

        if not os.path.exists(self.save_dir):
            try:
                os.makedirs(self.save_dir)
                print(f"\n[INFO] Created directory: {self.save_dir}")
            except OSError as e:
                self.get_logger().error(f"Could not create directory {self.save_dir}: {e}")
                return
        else:
             print(f"\n[INFO] Saving report images to: {self.save_dir}\n")

        self.create_subscription(Image, self.rgb_topic, self.image_cb, 10)
        self.frame_count = 0
        self.is_processing = False
        
        print("--> Be ready FRONT OF THE ROBOT NOW.")
        print("--> Starting capture in 4 seconds...")
        time.sleep(6.0)

    def image_cb(self, msg):
        if self.is_processing: return
        if self.frame_count >= self.target_frame_count:
            print(f"\n[DONE] Captured {self.target_frame_count} images. Exiting.")
            rclpy.shutdown()
            return

        self.is_processing = True
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
            img_w, img_h = pil_img.size

            print(f"[{self.frame_count+1}/{self.target_frame_count}] Sending to Gemini...", end="", flush=True)

            prompt = """
            Analyze this image for humans. Return JSON.
            Fields required:
            - "bbox": [ymin, xmin, ymax, xmax] (Normalized 0.0-1.0)
            - "confidence": float (Estimated certainty 0.0-1.0)
            - "keypoint": [x, y] (Pixel coordinates for center of chest/torso)
            
            Format: { "humans": [ { "bbox": [0.1, 0.2, 0.5, 0.6], "confidence": 0.95, "keypoint": [320, 240] } ] }
            """
            
            start_time = time.time()
            response = self.model.generate_content([prompt, pil_img])
            print(f" Done ({time.time() - start_time:.2f}s)")

            text = response.text.strip().replace("```json", "").replace("```", "")
            if "'" in text: text = text.replace("'", '"')
            
            try:
                data = json.loads(text[text.find('{'):text.rfind('}')+1])
                humans = data.get('humans', [])
            except:
                print("  -> JSON Parse Failed")
                humans = []

            annotated_img = cv_img.copy()
            
            for i, h in enumerate(humans):
                bbox = h.get('bbox')
                conf = h.get('confidence', 0.0)
                kp = h.get('keypoint')

                # 1. Draw Bounding Box
                if bbox:
                    ymin, xmin, ymax, xmax = bbox
                    pt1 = (int(xmin * img_w), int(ymin * img_h))
                    pt2 = (int(xmax * img_w), int(ymax * img_h))
                    cv2.rectangle(annotated_img, pt1, pt2, (0, 255, 0), 2)

                    # Prepare Text strings
                    conf_text = f"Human {i}: Conf={conf:.2f}"
                    box_text = f"Box=[{ymin:.3f}, {xmin:.3f}, {ymax:.3f}, {xmax:.3f}]"
                    kp_text = f"Keypoint: {kp[0]}, {kp[1]}" if kp else "Keypoint: N/A"

                    # Write Text on Image (Top-Left corner of box)
                    # Line 1: Confidence
                    cv2.putText(annotated_img, conf_text, (pt1[0], pt1[1]-40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    # Line 2: Box Coords
                    cv2.putText(annotated_img, box_text, (pt1[0], pt1[1]-15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(annotated_img, kp_text, (pt1[0], pt1[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                                
                    print(f"  -> {conf_text}, {box_text}", {"keypoint": kp_text})

                # 2. Draw Keypoint
                if kp and len(kp) == 2:
                    cx, cy = int(kp[0]), int(kp[1])
                    cv2.circle(annotated_img, (cx, cy), 5, (0, 0, 255), -1) 
                    
                    # Write Keypoint Text
                    kp_text = f"Keypoint: {cx}, {cy}"
                    cv2.putText(annotated_img, kp_text, (cx + 10, cy), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    
                    print(f"  -> {kp_text}")

            # Save Images
            raw_path = os.path.join(self.save_dir, f"frame_{self.frame_count:02d}_raw.jpg")
            debug_path = os.path.join(self.save_dir, f"frame_{self.frame_count:02d}_debug.jpg")
            
            cv2.imwrite(raw_path, cv_img)
            cv2.imwrite(debug_path, annotated_img)
            
            self.frame_count += 1
            time.sleep(0.5) 

        except Exception as e:
            print(f"\n[ERROR] {e}")
        
        finally:
            self.is_processing = False

def main():
    rclpy.init()
    node = GeminiStaticTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()