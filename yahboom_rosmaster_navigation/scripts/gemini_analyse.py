#!/usr/bin/env python3
import os
import glob
import json
import time
import pandas as pd
import google.generativeai as genai
from PIL import Image
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix
import os


GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", '')
genai.configure(api_key=GEMINI_API_KEY)

model = genai.GenerativeModel('gemini-2.0-flash')

IMAGE_FOLDER = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images"
OUTPUT_CSV = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/ablation_results.csv"
OUTPUT_MATRIX_ANNOTATED = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/matrix_annotated.png"
OUTPUT_MATRIX_RAW = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/matrix_raw.png"
# This Grayscale comparison is for a future ablation study
OUTPUT_MATRIX_GRAY = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/matrix_grayscale.png"

# Zero-Shot Prompt for Social State Classification
PROMPT_ANNOTATED = """
Analyze this image for social navigation robotics. You will see humans with numbered bounding boxes (ID 1, 2...).
Rely on zero-shot visual reasoning to classify their social state.

Return a JSON array named "humans". For each human ID, provide:
1. "id": The integer ID from the image's green bounding box.
2. "state": MUST be one of: ["ignoring", "conversation", "photography", "sharedtask", "playing", "presenting", "parallelwalking"].
3. "target_ids": A list of IDs they are interacting with. If none, return [].
4. "gaze": Direction of attention ["left", "right", "forward", "backward", "at_target"].

Example: {"humans": [{"id": 1, "state": "photography", "target_ids": [2], "gaze": "at_target"}]}
"""

# Prompt B: For RAW images without bounding boxes
PROMPT_RAW = """
Analyze this raw camera image for social navigation robotics. Rely on zero-shot visual reasoning to classify the social state of the humans.

First, identify the humans in the image from LEFT to RIGHT. Assign them IDs starting from 1 (e.g., the leftmost person is ID 1). 
Return a JSON array named "humans". For each human, provide:
1. "id": The integer ID you assigned them based on their left-to-right position.
2. "state": MUST be one of: ["ignoring", "conversation", "photography", "sharedtask", "playing", "presenting", "parallelwalking"].
3. "target_ids": A list of IDs they are interacting with. If none, return [].
4. "gaze": Direction of attention ["left", "right", "forward", "backward", "at_target"].

Example: {"humans": [{"id": 1, "state": "photography", "target_ids": [2], "gaze": "at_target"}]}
"""


def run_evaluation():
    image_paths = sorted(glob.glob(os.path.join(IMAGE_FOLDER, "*.jpg")))
    print(f"Found {len(image_paths)} images to evaluate.\n")
    
    results_list = []

    for img_path in image_paths:
        filename = os.path.basename(img_path)
        print(f"Processing: {filename}...")
        
        try:
            pil_img = Image.open(img_path)
            eval_cases = []
            
            if "_raw" in filename:
                eval_cases.append(("Raw", PROMPT_RAW, pil_img))
            else:
                # For annotated images, we run the normal color version AND a grayscale version
                eval_cases.append(("Annotated", PROMPT_ANNOTATED, pil_img))
                
                # Convert to Grayscale ('L' (Luminance) or Destroys All Color) then back to 'RGB' channels so Gemini accepts it
                gray_img = pil_img.convert('L').convert('RGB')
                eval_cases.append(("Grayscale", PROMPT_ANNOTATED, gray_img))

            for image_type, current_prompt, img_to_send in eval_cases:
                print(f"  -> Requesting Gemini analysis for: {image_type}...")
                resp = model.generate_content([current_prompt, img_to_send])
                
                text = resp.text.strip().replace("```json", "").replace("```", "")
                if "'" in text: text = text.replace("'", '"')
                
                start_idx = text.find('{')
                end_idx = text.rfind('}') + 1
                
                if start_idx == -1 or end_idx == 0:
                    raise ValueError("No JSON block found in response.")
                    
                data = json.loads(text[start_idx:end_idx])
                humans = data.get('humans', [])
                
                for h in humans:
                    results_list.append({
                        "Filename": filename,
                        "Image_Type": image_type,
                        "Human_ID": h.get("id"),
                        "State": h.get("state"),
                        "Target_IDs": str(h.get("target_ids", [])),
                        "Gaze": h.get("gaze")
                    })
                    
                print(f"     [Success] Found {len(humans)} humans.")
                time.sleep(2.0) # Respect API rate limits
                
        except Exception as e:
            print(f"     [FAILED] Error: {e}")
            results_list.append({
                "Filename": filename,
                "Image_Type": "ERROR",
                "Human_ID": "ERROR",
                "State": "ERROR",
                "Target_IDs": "",
                "Gaze": ""
            })

   
    if results_list:
        df = pd.DataFrame(results_list)
        df.to_csv(OUTPUT_CSV, index=False)
        print(f"\nSaved CSV to: {OUTPUT_CSV}")

        df_clean = df[df['State'] != 'ERROR'].copy()
        df_clean['True_State'] = df_clean['Filename'].apply(lambda x: x.split('_')[0].lower())
        df_clean['Predicted_State'] = df_clean['State'].astype(str).str.lower()
        
        def plot_matrix(dataframe, title, output_path):
            if dataframe.empty: return
            labels = sorted(list(set(dataframe['True_State'].unique()) | set(dataframe['Predicted_State'].unique())))
            cm = confusion_matrix(dataframe['True_State'], dataframe['Predicted_State'], labels=labels)
            
            plt.figure(figsize=(10, 8))
            sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', xticklabels=labels, yticklabels=labels, linewidths=.5)
            plt.title(title, fontsize=16, pad=20)
            plt.xlabel('Predicted State (Gemini)', fontsize=14, labelpad=10)
            plt.ylabel('True State (Ground Truth)', fontsize=14, labelpad=10)
            plt.xticks(rotation=45, ha='right', fontsize=12)
            plt.yticks(rotation=0, fontsize=12)
            plt.tight_layout()
            plt.savefig(output_path, dpi=300)
            plt.close()

        print("\nGenerating Annotated Matrix...")
        plot_matrix(df_clean[df_clean['Image_Type'] == 'Annotated'], 'Classification: ANNOTATED Images', OUTPUT_MATRIX_ANNOTATED)
        
        print("Generating Raw Matrix...")
        plot_matrix(df_clean[df_clean['Image_Type'] == 'Raw'], 'Classification: RAW Images', OUTPUT_MATRIX_RAW)

        print("Generating Grayscale Matrix...")
        plot_matrix(df_clean[df_clean['Image_Type'] == 'Grayscale'], 'Classification: GRAYSCALE Images', OUTPUT_MATRIX_GRAY)

        print("\nEvaluation Complete! All 3 matrices generated successfully.")


if __name__ == "__main__":
    run_evaluation()