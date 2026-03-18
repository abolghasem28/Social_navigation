#!/usr/bin/env python3
import os
import glob
import json
import time
import pandas as pd
import google.generativeai as genai
from PIL import Image

# ==========================================
# 1. CONFIGURATION
# ==========================================
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", '')
genai.configure(api_key=GEMINI_API_KEY)
model = genai.GenerativeModel('gemini-2.0-flash')

IMAGE_FOLDER = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/dev_set"
OUTPUT_CSV = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/dev_set/dev_csv_results/VLM7_predictions.csv"

# ==========================================
# 2. PROMPTS
# ==========================================
PROMPT_ANNOTATED = """
You are the vision system for a social navigation robot analyzing humans marked with numbered bounding boxes.
The image has bounding boxes that identify people with IDs starting from 1.

People in the scene may be involved in different types of activity, some proximate (physically 
close, directly engaging with each other) and some remote (spatially offset but still part of 
the same shared activity). The robot must not disturb any ongoing activity, regardless of 
whether the people involved are facing each other or physically close.

STEP 1 — Classify the scene type BEFORE evaluating any pairs:
- FULL-GROUP: All people share one collective activity. → ALL pairs are Blocked (robot_can_cross ≤ 0.2).
  Once classified as FULL-GROUP, do NOT re-evaluate individual pairs, lack of direct 
  face-to-face contact between two specific members does NOT make them Open.
- PARTIAL-GROUP: Some people share an activity, others do not. Evaluate each pair 
  individually based on whether they are actively engaged with each other.

STEP 2 — Evaluate every unique pair according to the scene type above.
Use the spatial arrangement and proximity of bounding boxes to identify group clusters.

RULES:
- A social boundary exists if the two individuals are part of the same ongoing activity,
  whether they are directly interacting or jointly engaged from a distance.
- If there is 0 or 1 person in the image, return an empty array: {"social_links": []}
- Evaluate every single unique combination of IDs as a separate pair (e.g., [1,2], [1,3], [2,3]).

Return a strict JSON object with one array named "social_links". For each pair, provide:
1. "pair": A list of exactly two human IDs in ascending order (e.g., [1, 2]).
2. "engagement": ["low", "medium", "high"].
3. "robot_can_cross": Float probability (0.0 to 1.0). 0.0 = absolutely cannot cross, 1.0 = completely safe.
4. "reason": One short sentence explaining why.
"""

PROMPT_RAW = """
You are the vision system for a social navigation robot analyzing a camera image.
Silently identify the humans from LEFT to RIGHT and assign them IDs starting from 1.

People in the scene may be involved in different types of activity, some proximate (physically 
close, directly engaging with each other) and some remote (spatially offset but still part of 
the same shared activity). The robot must not disturb any ongoing activity, regardless of 
whether the people involved are facing each other or physically close.

STEP 1 — Classify the scene type BEFORE evaluating any pairs:
- FULL-GROUP: All people share one collective activity. → ALL pairs are Blocked (robot_can_cross ≤ 0.2).
  Once classified as FULL-GROUP, do NOT re-evaluate individual pairs, lack of direct 
  face-to-face contact between two specific members does NOT make them Open.
- PARTIAL-GROUP: Some people share an activity, others do not. Evaluate each pair 
  individually based on whether they are actively engaged with each other.

STEP 2 — Evaluate every unique pair according to the scene type above.

RULES:
- A social boundary exists if the two individuals are part of the same ongoing activity,
  whether they are directly interacting or jointly engaged from a distance.
- If there is 0 or 1 person in the image, return an empty array: {"social_links": []}
- Evaluate every single unique combination of IDs as a separate pair.

Return a strict JSON object with one array named "social_links". For each pair, provide:
1. "pair": A list of exactly two human IDs in ascending order (e.g., [1, 2]).
2. "engagement": ["low", "medium", "high"].
3. "robot_can_cross": Float probability (0.0 to 1.0). 0.0 = absolutely cannot cross, 1.0 = completely safe.
4. "reason": One short sentence explaining why.
"""

PROMPT_DEPTH = """
You are the vision system for a social navigation robot analyzing a depth map.
First, identify the distinct human silhouettes from LEFT to RIGHT. Assign them IDs starting from 1 (the leftmost person is ID 1).
Evaluate every unique pair of humans in the scene for invisible social boundaries.

RULES:
- A social boundary exists if the spatial orientation and proximity suggest the two individuals are interacting.
- If there is 0 or 1 person in the image, return an empty array: {"social_links": []}
- Evaluate every single unique combination of IDs as a separate pair.

Return a strict JSON object with one array named "social_links". For each pair, provide:
1. "pair": A list of exactly two human IDs in ascending order (e.g., [1, 2]).
2. "engagement": ["low", "medium", "high"].
3. "robot_can_cross": Float probability (0.0 to 1.0).  0.0 = absolutely cannot cross, 1.0 = completely safe.
4. "reason": One short sentence explaining why.
"""

def safe_gemini_call(prompt, img, retries=4):
    for attempt in range(retries):
        try:
            resp = model.generate_content(
                [prompt, img],
                generation_config={
                    "response_mime_type": "application/json",
                    "temperature": 0.1,
                    "max_output_tokens": 1500
                }
            )
            return resp
        except Exception as e:
            wait_time = 2 ** attempt
            print(f"     [API LIMIT] Attempt {attempt+1} failed ({e}). Retrying in {wait_time}s...")
            time.sleep(wait_time)
    return None

def run_inference(run_id):
    raw_paths = sorted(glob.glob(os.path.join(IMAGE_FOLDER, "*_raw.jpg")))
    print(f"      STARTING ITERATION {run_id}/10")
    results_list = []

    for raw_path in raw_paths:
        base_filename = os.path.basename(raw_path)
        print(f"Processing Scene: {base_filename}...")
        
        annotated_path = raw_path.replace("_raw.jpg", "_annotated.jpg")
        depth_path = raw_path.replace("_raw.jpg", "_depth.png")
        
        if not os.path.exists(annotated_path):
            print(f"     [WARNING] Missing {annotated_path}. Skipping.")
            continue
        if not os.path.exists(depth_path):
            print(f"     [WARNING] Missing {depth_path}. Skipping.")
            continue

        try:
            raw_img = Image.open(raw_path)
            annotated_img = Image.open(annotated_path)
            depth_img = Image.open(depth_path)

            eval_cases = [
                ("Raw", PROMPT_RAW, raw_img),
                ("Annotated", PROMPT_ANNOTATED, annotated_img),
                ("Depth", PROMPT_DEPTH, depth_img)
            ]

            for image_type, current_prompt, img_to_send in eval_cases:
                print(f"  Evaluating {image_type} image...")

                resp = safe_gemini_call(current_prompt, img_to_send)
                if not resp:
                    print(f"     [FAILED] Could not get response for {image_type}.")
                    continue
                
                try:
                    text = resp.text.strip().replace("```json", "").replace("```", "")
                    start_idx = text.find('{')
                    end_idx = text.rfind('}')
                    if start_idx == -1 or end_idx == -1: 
                        raise ValueError("No JSON bounds found.")
                    json_str = text[start_idx:end_idx+1].replace("'", '"')
                    data = json.loads(json_str)
                except Exception as e:
                    print(f"     [WARNING] Gemini completely broke JSON on {image_type}: {e}")
                    continue
                    
                social_links = data.get('social_links', [])
                
                for link in social_links:
                    raw_eng = link.get("engagement", "low")
                    if isinstance(raw_eng, list): 
                        raw_eng = raw_eng[0] if len(raw_eng) > 0 else "low"
                        
                    raw_cross = link.get("robot_can_cross", 1.0)
                    if isinstance(raw_cross, list): 
                        raw_cross = raw_cross[0] if len(raw_cross) > 0 else 1.0
                        
                    try:
                        raw_cross = max(0.0, min(1.0, float(raw_cross)))
                    except (ValueError, TypeError):
                        raw_cross = 1.0 
                        
                    if image_type == 'Raw':
                        final_filename = base_filename
                    elif image_type == 'Annotated':
                        final_filename = base_filename.replace('_raw.jpg', '_annotated.jpg')
                    elif image_type == 'Depth':
                        final_filename = base_filename.replace('_raw.jpg', '_depth.png')
                        
                    clean_pair = str(link.get("pair", [])).replace(' ', '')

                    results_list.append({
                        "Run_ID": run_id,
                        "Filename": final_filename,
                        "Image_Type": image_type,
                        "Pair": clean_pair, 
                        "Predicted_Engagement": str(raw_eng).lower(),
                        "Predicted_Crossability": raw_cross,
                        "Reason": str(link.get("reason", ""))
                    })

                time.sleep(1.0) 
                
        except Exception as e:
            print(f"     [FAILED] Error on {base_filename}: {e}")
            results_list.append({"Filename": base_filename, "Image_Type": "ERROR", "Predicted_Crossability": -1.0})

    if results_list:
        df = pd.DataFrame(results_list)
        output_file = OUTPUT_CSV.replace('.csv', f'_run_{run_id}.csv')
        df.to_csv(output_file, index=False)
        print(f"--- Iteration {run_id} complete. Saved: {output_file} ---")

if __name__ == "__main__":
    TOTAL_RUNS = 1
    #for i in range(1, TOTAL_RUNS + 1):
    for i in range(TOTAL_RUNS):
        run_inference(i)
        if i < TOTAL_RUNS:
            print("Cooling down API before next run...")
            time.sleep(10)