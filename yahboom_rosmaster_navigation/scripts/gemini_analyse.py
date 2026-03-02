#!/usr/bin/env python3
import os
import glob
import json
import time
import pandas as pd
import numpy as np
import google.generativeai as genai
from PIL import Image
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix, roc_curve, auc, brier_score_loss
from sklearn.calibration import calibration_curve

# ==========================================
# 1. CONFIGURATION
# ==========================================
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", '')
genai.configure(api_key=GEMINI_API_KEY)
model = genai.GenerativeModel('gemini-2.0-flash')

IMAGE_FOLDER = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images"
OUTPUT_CSV = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/ablation_results.csv"

OUTPUT_MATRIX_RAW = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/matrix_decision_raw.png"
OUTPUT_MATRIX_ANNOTATED = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/matrix_decision_annotated.png"
OUTPUT_MATRIX_DEPTH = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/matrix_decision_depth.png"

# Stats & Bar Charts
STATS_CSV_PAIR = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/accuracy_stats_pair.csv"
STATS_CSV_SCENE = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/accuracy_stats_scene.csv"
OUTPUT_PLOT_PAIR = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/accuracy_chart_pair.png"
OUTPUT_PLOT_SCENE = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/accuracy_chart_scene.png"

# Base paths for dynamic pair/scene plot generation
BASE_ROC = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/roc_curve"
BASE_DIST = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/prob_distributions"
BASE_CALIB = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/calibration_curve"

GT_MAPPING = {
    'conversation': {'engagement': 'high', 'decision': 'Blocked'},
    'photography': {'engagement': 'high', 'decision': 'Blocked'},
    'presenting': {'engagement': 'high', 'decision': 'Blocked'},
    'playing': {'engagement': 'high', 'decision': 'Blocked'},
    'sharedtask': {'engagement': 'high', 'decision': 'Blocked'},
    'parallelwalking': {'engagement': 'medium', 'decision': 'Blocked'}, 
    'ignoring': {'engagement': 'low', 'decision': 'Open'},
    'proxemics': {'engagement': 'low', 'decision': 'Open'}
}

# ==========================================
# 2. PROMPTS
# ==========================================
PROMPT_ANNOTATED = """
You are the vision system for a social navigation robot. Look at the humans marked with numbered bounding boxes (ID 1, 2, 3...).
Evaluate every unique pair of humans in the scene for invisible social boundaries.

RULES:
- If there is 0 or 1 person in the image, return an empty array: {"social_links": []}
- If people are in a group of 3 or more, evaluate every single combination as a separate pair (e.g., [1,2], [2,3], [1,3]).

Return a strict JSON object with one array named "social_links". For each pair, provide:
1. "pair": A list of exactly two human IDs (e.g., [1, 2]).
2. "engagement": ["low", "medium", "high"].
3. "robot_can_cross": Float probability (0.0 to 1.0). 0.0 = absolutely cannot cross, 1.0 = completely safe.
4. "reason": One short sentence explaining why.
"""

PROMPT_RAW = """
You are the vision system for a social navigation robot analyzing a raw, unmarked camera image.

First, silently identify the humans from LEFT to RIGHT. Assign them IDs starting from 1 (the leftmost person is ID 1, next is 2, etc.).
Evaluate every unique pair of humans in the scene for invisible social boundaries.

RULES:
- If there is 0 or 1 person in the image, return an empty array: {"social_links": []}
- If people are in a group of 3 or more, evaluate every single combination as a separate pair.
- MAXIMUM CAP: Never evaluate more than 10 pairs total. Focus only on the most prominent, clear humans.

Return a strict JSON object with one array named "social_links". For each pair, provide:
1. "pair": A list of exactly two human IDs (e.g., [1, 2]).
2. "engagement": ["low", "medium", "high"].
3. "robot_can_cross": Float probability (0.0 to 1.0). 0.0 = absolutely cannot cross, 1.0 = completely safe.
4. "reason": One short sentence explaining why.
"""
PROMPT_DEPTH = """
You are the vision system for a social navigation robot analyzing a depth map. Brighter/different colors represent physical distance.
Identify the humans from LEFT to RIGHT. Assign them IDs starting from 1.
Evaluate every unique pair of humans in the scene for invisible social boundaries.

RULES:
- If there is 0 or 1 person in the image, return an empty array: {"social_links": []}
- If people are in a group of 3 or more, evaluate every single combination as a separate pair.
- MAXIMUM CAP: Never evaluate more than 10 pairs total. Focus only on the most prominent, clear humans.

Return a strict JSON object with one array named "social_links". For each pair, provide:
1. "pair": A list of exactly two human IDs (e.g., [1, 2]).
2. "engagement": ["low", "medium", "high"].
3. "robot_can_cross": Float probability (0.0 to 1.0). 0.0 = absolutely cannot cross, 1.0 = completely safe.
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

def export_statistics_report(dataframe, output_path):
    report_data = []
    for image_type in dataframe['Image_Type'].unique():
        subset = dataframe[dataframe['Image_Type'] == image_type]
        if subset.empty: continue
        
        total_predictions = len(subset)
        correct_predictions = len(subset[subset['True_Decision'] == subset['Predicted_Decision']])
        overall_acc = (correct_predictions / total_predictions) * 100 if total_predictions > 0 else 0
        
        report_data.append({
            'Image_Type': image_type, 
            'Category': 'OVERALL_NAVIGATION_DECISION',
            'Total_Evaluated': total_predictions, 
            'Correct_Predictions': correct_predictions,
            'Accuracy_Percentage': round(overall_acc, 1)
        })
        
        scenarios = sorted(subset['Scenario'].unique())
        for scenario in scenarios:
            scen_subset = subset[subset['Scenario'] == scenario]
            scen_total = len(scen_subset)
            scen_correct = len(scen_subset[scen_subset['True_Decision'] == scen_subset['Predicted_Decision']])
            scen_acc = (scen_correct / scen_total) * 100 if scen_total > 0 else 0.0
            
            report_data.append({
                'Image_Type': image_type, 
                'Category': f"Scenario: {scenario.capitalize()}",
                'Total_Evaluated': scen_total, 
                'Correct_Predictions': scen_correct,
                'Accuracy_Percentage': round(scen_acc, 1)
            })
            
    pd.DataFrame(report_data).to_csv(output_path, index=False)
    print(f"---> Statistical CSV saved to: {output_path}")

def plot_scenario_breakdown(dataframe, title, output_path):
    if dataframe.empty: return
    crosstab = pd.crosstab(dataframe['Scenario'], dataframe['Predicted_Decision'], normalize='index')
    crosstab_percentage = crosstab * 100
    if 'Blocked' not in crosstab_percentage.columns: crosstab_percentage['Blocked'] = 0.0
    if 'Open' not in crosstab_percentage.columns: crosstab_percentage['Open'] = 0.0
    crosstab_percentage = crosstab_percentage[['Blocked', 'Open']] 
    
    plt.figure(figsize=(10, 8))
    sns.heatmap(crosstab_percentage, annot=True, fmt='.1f', cmap='Blues', linewidths=.5)
    plt.title(title + " (%)", fontsize=16, pad=20)
    plt.xlabel('Predicted Navigation Decision', fontsize=14, labelpad=10)
    plt.ylabel('Ground Truth Scenario', fontsize=14, labelpad=10)
    plt.yticks(rotation=0, fontsize=12)
    plt.xticks(fontsize=12)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"---> Generated Scenario Breakdown Matrix: {output_path}")



def plot_roc_auc(dataframe, title_suffix, output_path):
    if dataframe.empty: return
    plt.figure(figsize=(8, 8))
    sns.set_theme(style="whitegrid")
    colors = {'Raw': '#1f77b4', 'Annotated': '#2ca02c', 'Depth': '#9467bd'}
    
    for image_type in ['Raw', 'Annotated', 'Depth']:
        subset = dataframe[dataframe['Image_Type'] == image_type]
        if subset.empty: continue
        
        # Hazard Detection: Blocked is the positive class (1)
        y_true = (subset['True_Decision'] == 'Blocked').astype(int)
        y_prob = 1.0 - subset['Predicted_Cross_Prob']
        
        fpr, tpr, _ = roc_curve(y_true, y_prob)
        roc_auc = auc(fpr, tpr)
        
        plt.plot(fpr, tpr, color=colors.get(image_type, 'black'), lw=2, 
                 label=f'{image_type} (AUC = {roc_auc:.3f})')

    plt.plot([0, 1], [0, 1], color='gray', lw=1, linestyle='--')
    plt.xlim([0.0, 1.0])
    plt.ylim([0.0, 1.05])
    plt.xlabel('False Positive Rate (Safe classified as Hazard)', fontsize=12)
    plt.ylabel('True Positive Rate (Hazard correctly detected)', fontsize=12)
    plt.title(f'ROC Curve: Hazard Detection ({title_suffix})', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc="lower right", fontsize=12)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"---> Generated ROC Curve: {output_path}")

def plot_probability_distributions(dataframe, title_suffix, output_path):
    if dataframe.empty: return
    image_types = [t for t in ['Raw', 'Annotated', 'Depth'] if t in dataframe['Image_Type'].unique()]
    fig, axes = plt.subplots(len(image_types), 1, figsize=(10, 4 * len(image_types)), sharex=True)
    if len(image_types) == 1: axes = [axes]
    
    for ax, image_type in zip(axes, image_types):
        subset = dataframe[dataframe['Image_Type'] == image_type]
        sns.kdeplot(data=subset, x='Predicted_Cross_Prob', hue='True_Decision', 
                    fill=True, common_norm=False, palette={'Blocked': '#d62728', 'Open': '#1f77b4'}, 
                    alpha=0.5, ax=ax)
        ax.set_title(f'Probability Distribution: {image_type} Image', fontsize=14)
        ax.set_ylabel('Density', fontsize=12)
        ax.axvline(0.5, color='black', linestyle='--', alpha=0.5)
        
    plt.xlabel('VLM Predicted Probability of Safe Crossing', fontsize=12)
    plt.xlim(0, 1)
    fig.suptitle(f'Distributions ({title_suffix})', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"---> Generated Distribution Plots: {output_path}")

def plot_calibration_and_brier(dataframe, title_suffix, output_path):
    if dataframe.empty: return
    plt.figure(figsize=(8, 8))
    sns.set_theme(style="whitegrid")
    colors = {'Raw': '#1f77b4', 'Annotated': '#2ca02c', 'Depth': '#9467bd'}
    
    for image_type in ['Raw', 'Annotated', 'Depth']:
        subset = dataframe[dataframe['Image_Type'] == image_type]
        if subset.empty: continue
        
        # Hazard Detection: Blocked is the positive class (1)
        y_true = (subset['True_Decision'] == 'Blocked').astype(int)
        y_prob = 1.0 - subset['Predicted_Cross_Prob']
        
        brier = brier_score_loss(y_true, y_prob)
        prob_true, prob_pred = calibration_curve(y_true, y_prob, n_bins=10)
        plt.plot(prob_pred, prob_true, marker='o', color=colors.get(image_type, 'black'), 
                 label=f'{image_type} (Brier = {brier:.3f})')

    plt.plot([0, 1], [0, 1], linestyle='--', color='gray', label='Perfect Calibration')
    plt.xlabel('Mean Predicted Hazard Probability', fontsize=12)
    plt.ylabel('Fraction of Actual Hazards', fontsize=12)
    plt.title(f'Calibration Curve & Reliability ({title_suffix})', fontsize=16, fontweight='bold', pad=15)
    plt.legend(loc="upper left", fontsize=12)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"---> Generated Calibration Curve: {output_path}")


def plot_accuracy_comparison(dataframe, title_suffix, output_path):
    if dataframe.empty: return
    data = []
    for image_type in ['Raw', 'Annotated', 'Depth']:
        subset = dataframe[dataframe['Image_Type'] == image_type]
        if subset.empty: continue
        
        total = len(subset)
        correct = len(subset[subset['True_Decision'] == subset['Predicted_Decision']])
        acc = (correct / total) * 100 if total > 0 else 0
        data.append({'Image Type': image_type, 'Navigation Accuracy (%)': acc})
            
    df_plot = pd.DataFrame(data)
    plt.figure(figsize=(8, 6))
    sns.set_theme(style="whitegrid")
    chart = sns.barplot(
        data=df_plot, x='Image Type', y='Navigation Accuracy (%)', 
        hue='Image Type', palette=['#1f77b4', '#7f7f7f', '#ff7f0e'], legend=False
    )
    for p in chart.patches:
        height = p.get_height()
        chart.annotate(f'{height:.1f}%', (p.get_x() + p.get_width() / 2., height), 
                       ha='center', va='bottom', fontsize=12, xytext=(0, 5), textcoords='offset points')
            
    plt.title(f'Robot Navigation Decision Accuracy ({title_suffix})', fontsize=16, pad=20, fontweight='bold')
    plt.ylim(0, 105)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"---> Accuracy Bar Chart saved to: {output_path}")

def run_evaluation():
    raw_paths = sorted(glob.glob(os.path.join(IMAGE_FOLDER, "*_raw.jpg")))
    print(f"Found {len(raw_paths)} unique scenes to evaluate.\n")
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

                    results_list.append({
                        "Filename": base_filename, 
                        "Image_Type": image_type,
                        "Pair": str(link.get("pair", [])), 
                        "Predicted_Engagement": str(raw_eng).lower(),
                        "Predicted_Cross_Prob": raw_cross,
                        "Reason": str(link.get("reason", ""))
                    })

                time.sleep(1.0) 
                
        except Exception as e:
            print(f"     [FAILED] Error on {base_filename}: {e}")
            results_list.append({"Filename": base_filename, "Image_Type": "ERROR", "Predicted_Cross_Prob": -1.0})

    if results_list:
        df = pd.DataFrame(results_list)
        df.to_csv(OUTPUT_CSV, index=False)
        print(f"\nSaved raw CSV to: {OUTPUT_CSV}")

        df_clean = df[df['Image_Type'] != 'ERROR'].copy()
        if df_clean.empty: return

        df_clean['Scenario'] = df_clean['Filename'].apply(lambda x: x.split('_')[0].lower())
        df_clean['True_Engagement'] = df_clean['Scenario'].map(lambda x: GT_MAPPING.get(x, {}).get('engagement', 'low'))
        df_clean['True_Decision'] = df_clean['Scenario'].map(lambda x: GT_MAPPING.get(x, {}).get('decision', 'Open'))
        df_clean['Predicted_Decision'] = df_clean['Predicted_Cross_Prob'].apply(lambda x: 'Blocked' if x < 0.5 else 'Open')

        # Group by the image and calculate min, mean, and 25th percentile
        scene_df = df_clean.groupby(['Filename', 'Image_Type', 'Scenario', 'True_Engagement', 'True_Decision']).agg(
            Min_Cross_Prob=('Predicted_Cross_Prob', 'min'),
            Mean_Cross_Prob=('Predicted_Cross_Prob', 'mean'),
            Q25_Cross_Prob=('Predicted_Cross_Prob', lambda x: x.quantile(0.25))
        ).reset_index()
        
        # Use Q25 for the final prediction to avoid the single-hallucination minimum trap
        scene_df['Predicted_Cross_Prob'] = scene_df['Q25_Cross_Prob']
        scene_df['Predicted_Decision'] = scene_df['Predicted_Cross_Prob'].apply(lambda x: 'Blocked' if x < 0.5 else 'Open')

        print("\n===========================================")
        print("         CLASS BALANCE AUDIT")
        print("===========================================")
        print("Pair-Level True Decisions (Percentage):")
        print(df_clean['True_Decision'].value_counts(normalize=True) * 100)
        print("\nScene-Level True Decisions (Percentage):")
        print(scene_df['True_Decision'].value_counts(normalize=True) * 100)
        print("===========================================\n")

      
        print("\n--- GENERATING VISUALIZATIONS (PAIR LEVEL) ---")
        plot_accuracy_comparison(df_clean, "Pair Level", OUTPUT_PLOT_PAIR)
        export_statistics_report(df_clean, STATS_CSV_PAIR)
        plot_roc_auc(df_clean, "Pair Level", f"{BASE_ROC}_pair.png")
        plot_probability_distributions(df_clean, "Pair Level", f"{BASE_DIST}_pair.png")
        plot_calibration_and_brier(df_clean, "Pair Level", f"{BASE_CALIB}_pair.png")

        print("\n--- GENERATING VISUALIZATIONS (SCENE LEVEL) ---")
        df_raw = scene_df[scene_df['Image_Type'] == 'Raw']
        if not df_raw.empty:
            plot_scenario_breakdown(df_raw, 'Navigation Safety per Scenario (Raw, Scene Level)', OUTPUT_MATRIX_RAW)

        df_annotated = scene_df[scene_df['Image_Type'] == 'Annotated']
        if not df_annotated.empty:
            plot_scenario_breakdown(df_annotated, 'Navigation Safety per Scenario (Annotated, Scene Level)', OUTPUT_MATRIX_ANNOTATED)
            
        df_depth = scene_df[scene_df['Image_Type'] == 'Depth']
        if not df_depth.empty:
            plot_scenario_breakdown(df_depth, 'Navigation Safety per Scenario (Depth, Scene Level)', OUTPUT_MATRIX_DEPTH)

        plot_accuracy_comparison(scene_df, "Scene Level", OUTPUT_PLOT_SCENE)
        export_statistics_report(scene_df, STATS_CSV_SCENE)
        plot_roc_auc(scene_df, "Scene Level", f"{BASE_ROC}_scene.png")
        plot_probability_distributions(scene_df, "Scene Level", f"{BASE_DIST}_scene.png")
        plot_calibration_and_brier(scene_df, "Scene Level", f"{BASE_CALIB}_scene.png")
        
        print("\n--- PIPELINE COMPLETE! ---")

if __name__ == "__main__":
    run_evaluation()