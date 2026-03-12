#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix


VLM_RESULTS_DIR = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/vlm_csvs"
GROUND_TRUTH_CSV = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/ground_truth_template.csv"
OUTPUT_MARGED_CSV = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/merged_results.csv"

OUTPUT_MATRIX_RAW = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/matrix_decision_raw.png"
OUTPUT_MATRIX_ANNOTATED = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/matrix_decision_annotated.png"
OUTPUT_MATRIX_DEPTH = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/matrix_decision_depth.png"
STATS_CSV_PAIR = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/accuracy_stats_pair.csv"
OUTPUT_PLOT_PAIR = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/accuracy_chart_pair.png"

OUTPUT_PLOT_DIST = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/distribution_box_swarm.png"

def export_statistics_report(dataframe, output_path):
    report_data = []
    for image_type in dataframe['Image_Type'].unique():
        subset = dataframe[dataframe['Image_Type'] == image_type]
        if subset.empty: continue
        
        total_predictions = len(subset)
        correct_predictions = len(subset[subset['Actual_Status'] == subset['Predicted_Decision']])
        overall_acc = (correct_predictions / total_predictions) * 100 if total_predictions > 0 else 0
        
        report_data.append({
            'Image_Type': image_type, 
            'Category': 'OVERALL_PAIR_DECISION',
            'Total_Evaluated': total_predictions, 
            'Correct_Predictions': correct_predictions,
            'Accuracy_Percentage': round(overall_acc, 1)
        })
            
    pd.DataFrame(report_data).to_csv(output_path, index=False)
    print(f"---> Statistical CSV saved to: {output_path}")

def plot_accuracy_comparison(dataframe, output_path):
    if dataframe.empty: return
    data = []
    for image_type in ['Raw', 'Annotated', 'Depth']:
        subset = dataframe[dataframe['Image_Type'] == image_type]
        if subset.empty: continue
        
        total = len(subset)
        correct = len(subset[subset['Actual_Status'] == subset['Predicted_Decision']])
        acc = (correct / total) * 100 if total > 0 else 0
        data.append({'Image Type': image_type, 'Navigation Accuracy (%)': acc})
            
    df_plot = pd.DataFrame(data)
    plt.figure(figsize=(8, 6))
    sns.set_theme(style="whitegrid")
    chart = sns.barplot(
        data=df_plot, x='Image Type', y='Navigation Accuracy (%)', 
        hue='Image Type', palette=["#cb5ee6", "#52B169", "#8b8783"], legend=False
    )
    for p in chart.patches:
        height = p.get_height()
        chart.annotate(f'{height:.1f}%', (p.get_x() + p.get_width() / 2., height), 
                       ha='center', va='bottom', fontsize=12, xytext=(0, 5), textcoords='offset points')
            
    plt.title('Pair-Level Navigation Decision Accuracy', fontsize=16, pad=20, fontweight='bold')
    plt.ylim(0, 105)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"---> Accuracy Bar Chart saved to: {output_path}")

def plot_confusion_matrix(dataframe, title, output_path):
    if dataframe.empty: return
    
    y_true = dataframe['Actual_Status'].astype(str)
    y_pred = dataframe['Predicted_Decision'].astype(str)
    
    labels = ['Blocked', 'Open']
    
    if len(set(y_true).intersection(set(labels))) == 0:
        print(f"     [WARNING] Skipping {title}. No 'Blocked' or 'Open' labels found.")
        return

    cm = confusion_matrix(y_true, y_pred, labels=labels)
    cm_percentage = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis] * 100
    cm_percentage = np.nan_to_num(cm_percentage) 
    
    plt.figure(figsize=(8, 6))
    ax = plt.gca()
    
    sns.heatmap(cm_percentage, annot=True, fmt='.1f', cbar=False, 
                xticklabels=labels, yticklabels=labels, 
                annot_kws={'size': 16, 'weight': 'bold'})
    
    
    custom_colors = [
        ["#eca6a6", "#f0f00d"],  
        ["#eeee09", "#16cd16"]   
    ]
              
    # 3. Inject the colors behind the text (zorder=0 pushes it to the back layer)
    for i in range(2):
        for j in range(2):
            ax.add_patch(plt.Rectangle((j, i), 1, 1, fill=True, facecolor=custom_colors[i][j], zorder=0))

    plt.title(title + " (%)", fontsize=16, pad=20, fontweight='bold')
    plt.xlabel('VLM Predicted Decision', fontsize=14, labelpad=10)
    plt.ylabel('Ground Truth (Actual)', fontsize=14, labelpad=10)
    
    # Increase text sizes for readability
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12, rotation=0)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"---> Generated Custom Color Confusion Matrix: {output_path}")

def plot_distribution_box_swarm(dataframe, output_path):
    if dataframe.empty: return
    
    # 1. Clean the probability data to ensure it is numeric
    dataframe['Predicted_Crossability'] = pd.to_numeric(dataframe['Predicted_Crossability'], errors='coerce')
    df_plot = dataframe.dropna(subset=['Predicted_Crossability']).copy()
    
    plt.figure(figsize=(10, 7))
    sns.set_theme(style="whitegrid")
    
    box_palette = {"Raw": "#7a378a", "Annotated": "#207736", "Depth": "#4E4A46"}
    
    # FIX 1: Add hue='Image_Type' and legend=False for the new Seaborn update
    sns.boxplot(
        data=df_plot, 
        x='Image_Type', 
        y='Predicted_Crossability',
        hue='Image_Type', 
        legend=False, 
        palette=box_palette,
        showfliers=False, 
        boxprops=dict(alpha=0.4),
        order=['Raw', 'Annotated', 'Depth']
    )
    
    # FIX 2: Change swarmplot to stripplot with jitter=True to handle massive data
    sns.stripplot(
        data=df_plot, 
        x='Image_Type', 
        y='Predicted_Crossability',
        hue='Actual_Status',
        palette={'Blocked': "#a22222", 'Open': "#0C870C"}, 
        dodge=True, 
        jitter=True, 
        size=4, 
        alpha=0.6, 
        order=['Raw', 'Annotated', 'Depth']
    )
    
    plt.title('The results of 10 times running of VLM Social Navigation Decision', fontsize=16, fontweight='bold', pad=15)
    plt.xlabel('VLM Input Modality of RAW Annotated and Depth', fontsize=14, labelpad=10)
    plt.ylabel('Predicted of Crossabilities', fontsize=14, labelpad=10)
    plt.ylim(-0.05, 1.05)
    
    # Fix the legend
    plt.legend(title='Ground Truth (Actual)', fontsize=12, title_fontsize=12, loc='upper left', bbox_to_anchor=(1.02, 1))
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"---> Generated Box/Strip Plot: {output_path}")

def run_evaluation():
    if not os.path.exists(GROUND_TRUTH_CSV):
        print(f"ERROR: Ground Truth file not found at {GROUND_TRUTH_CSV}")
        return
        
    # NEW MERGE LOGIC: Find all CSVs in the directory and concatenate them
    if not os.path.exists(VLM_RESULTS_DIR):
        print(f"ERROR: VLM results directory not found at {VLM_RESULTS_DIR}")
        return
        
    all_csv_files = glob.glob(os.path.join(VLM_RESULTS_DIR, "*.csv"))
    if not all_csv_files:
        print(f"ERROR: No CSV files found in {VLM_RESULTS_DIR}")
        return
        
    print(f"Found {len(all_csv_files)} VLM prediction CSV files. Merging...")
    df_list = [pd.read_csv(f) for f in all_csv_files]
    df_vlm = pd.concat(df_list, ignore_index=True)

    df_clean = df_vlm[df_vlm['Image_Type'] != 'ERROR'].copy()
    if df_clean.empty: 
        print("ERROR: VLM results are empty or full of errors.")
        return

    # 1. Convert probability to binary decision
    df_clean['Predicted_Decision'] = df_clean['Predicted_Crossability'].apply(lambda x: 'Blocked' if float(x) <= 0.5 else 'Open')

    # 2. Self-Healing Data Cleaner for Ground Truth
    gt_df = pd.read_csv(GROUND_TRUTH_CSV)
    gt_df['Actual_Status'] = gt_df['Actual_Status'].astype(str).str.strip().str.title()
    gt_df['Actual_Status'] = gt_df['Actual_Status'].replace({'0': 'Blocked', '0.0': 'Blocked', '1': 'Open', '1.0': 'Open'})

    # 3. Aggressive String Cleaning for Merging
    gt_df['Pair'] = gt_df['Pair'].astype(str).str.replace(' ', '').str.replace("'", "").str.replace('"', "")
    df_clean['Pair'] = df_clean['Pair'].astype(str).str.replace(' ', '').str.replace("'", "").str.replace('"', "")
    
    gt_df['Image_Type'] = gt_df['Image_Type'].astype(str).str.strip().str.title()
    df_clean['Image_Type'] = df_clean['Image_Type'].astype(str).str.strip().str.title()

    # 4. Extract Base Scene
    gt_df['Base_Scene'] = gt_df['Filename'].astype(str).apply(
        lambda x: x.replace('_raw.jpg', '').replace('_annotated.jpg', '').replace('_depth.png', '')
    )
    df_clean['Base_Scene'] = df_clean['Filename'].astype(str).apply(
        lambda x: x.replace('_raw.jpg', '').replace('_annotated.jpg', '').replace('_depth.png', '')
    )

    # 5. Execute Merge
    gt_lookup = gt_df[['Base_Scene', 'Pair', 'Actual_Status', 'Level_Engagement']].drop_duplicates(subset=['Base_Scene', 'Pair'])
    df_merged = pd.merge(df_clean, gt_lookup, on=['Base_Scene', 'Pair'], how='inner')

    df_merged.to_csv(OUTPUT_MARGED_CSV, index=False)
    print(f"---> Merged VLM predictions saved to: {OUTPUT_MARGED_CSV}")

    print("===========================================")
    print("         CLASS BALANCE AUDIT (PAIR LEVEL)")
    print("===========================================")
    print("Actual True Decisions (Percentage) from CSV:")
    print(df_merged['Actual_Status'].value_counts(normalize=True) * 100)
    print("Total Pairs Evaluated:", len(df_merged))
    print("===========================================\n")

    print("\n--- GENERATING VISUALIZATIONS ---")
    plot_accuracy_comparison(df_merged, OUTPUT_PLOT_PAIR)
    export_statistics_report(df_merged, STATS_CSV_PAIR)
    
    for img_type, output_path in [
        ('Raw', OUTPUT_MATRIX_RAW),
        ('Annotated', OUTPUT_MATRIX_ANNOTATED),
        ('Depth', OUTPUT_MATRIX_DEPTH)
    ]:
        subset = df_merged[df_merged['Image_Type'] == img_type]
        if not subset.empty:
            plot_confusion_matrix(subset, f'Confusion Matrix ({img_type})', output_path)
            
    # CALL NEW BOXPLOT FUNCTION
    plot_distribution_box_swarm(df_merged, OUTPUT_PLOT_DIST)
            
    print("\n--- PIPELINE COMPLETE! ---")

if __name__ == "__main__":
    run_evaluation()