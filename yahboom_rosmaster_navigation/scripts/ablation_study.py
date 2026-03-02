import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import os


filename_ablation = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/ablation_results.csv"
file_name_pair = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/accuracy_stats_pair.csv"
output_dir = "/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/thesis_evaluation/"

def generate_strategic_dashboard():
    # 1. DATA ACQUISITION
    # Load the results and overall accuracy stats
    ablation_df = pd.read_csv(filename_ablation)
    accuracy_pair = pd.read_csv(file_name_pair)

    # 2. FEATURE ENGINEERING (Extracting the Signal)
    # Extract the scenario type from the filename (e.g., 'playing_123.jpg' -> 'Playing')
    ablation_df['Scenario'] = ablation_df['Filename'].str.split('_').str[0].str.capitalize()
    
    # Ground Truth Logic: 'Ignoring' is safe to cross (1.0). All others are blocked (0.0).
    ablation_df['Truth'] = ablation_df['Scenario'].apply(lambda x: 1.0 if x == 'Ignoring' else 0.0)

    # 3. METRIC CALCULATION
    # Certainty (Boldness): Distance from the 0.5 'I don't know' point, scaled 0 to 1.
    ablation_df['Certainty'] = (ablation_df['Predicted_Cross_Prob'] - 0.5).abs() * 2
    
    # Error: Distance from actual ground truth
    ablation_df['Error'] = (ablation_df['Predicted_Cross_Prob'] - ablation_df['Truth']).abs()
    
    # A "Lie" (The Black Swan): Certainty > 0.6 AND Error > 0.5
    ablation_df['Is_Lie'] = (ablation_df['Certainty'] > 0.6) & (ablation_df['Error'] > 0.5)
    
    # Identification of "Bold" predictions
    ablation_df['Is_Bold'] = ablation_df['Certainty'] > 0.6

    # --- VISUALIZATION 1: ACCURACY vs. BOLDNESS ---
    # Compare truth (accuracy) against model ego (boldness)
    boldness_stats = ablation_df.groupby('Image_Type')['Is_Bold'].mean().reset_index()
    boldness_stats['Boldness_Pct'] = boldness_stats['Is_Bold'] * 100
    
    overall_acc = accuracy_pair[accuracy_pair['Category'] == 'OVERALL_NAVIGATION_DECISION'][['Image_Type', 'Accuracy_Percentage']]
    diag_df = pd.merge(boldness_stats, overall_acc, on='Image_Type')

    plt.figure(figsize=(10, 6))
    x_axis = np.arange(len(diag_df['Image_Type']))
    plt.bar(x_axis - 0.2, diag_df['Accuracy_Percentage'], 0.4, label='Accuracy % (Truth)', color='#3498db')
    plt.bar(x_axis + 0.2, diag_df['Boldness_Pct'], 0.4, label='Boldness % (Confidence)', color='#e67e22')
    plt.xticks(x_axis, diag_df['Image_Type'])
    plt.title('Strategic Diagnostic: Accuracy vs. Boldness')
    plt.ylabel('Percentage')
    plt.legend()
    plt.grid(axis='y', linestyle='--', alpha=0.3)
    plt.savefig(os.path.join(output_dir, 'diagnostic_accuracy_confidence.png'))
    plt.close()

    # --- VISUALIZATION 2: SUCCESS HEATMAP ---
    # Identifying where each model type wins
    pivot_acc = accuracy_pair.pivot(index='Category', columns='Image_Type', values='Accuracy_Percentage')
    pivot_scenarios = pivot_acc.drop('OVERALL_NAVIGATION_DECISION', errors='ignore')
    pivot_scenarios = pivot_scenarios[['Depth', 'Raw', 'Annotated']] # Logical order

    plt.figure(figsize=(12, 8))
    sns.heatmap(pivot_scenarios, annot=True, cmap='RdYlGn', fmt='.1f')
    plt.title('Strategic Success Map: Accuracy by Scenario (%)')
    plt.savefig(os.path.join(output_dir, 'strategic_success_heatmap.png'))
    plt.close()

    # --- VISUALIZATION 3: THE FAILURE MAP (LIE RATE) ---
    # Highlighting where the model is dangerously overconfident
    lie_stats = ablation_df.groupby(['Scenario', 'Image_Type'])['Is_Lie'].mean().reset_index()
    lie_stats['Lie_Rate'] = lie_stats['Is_Lie'] * 100
    lie_map = lie_stats.pivot(index='Scenario', columns='Image_Type', values='Lie_Rate').fillna(0)
    lie_map = lie_map[['Depth', 'Raw', 'Annotated']]

    plt.figure(figsize=(12, 8))
    sns.heatmap(lie_map, annot=True, cmap='OrRd', fmt='.1f')
    plt.title('The Failure Map: Confidently Wrong Rate (%)')
    plt.savefig(os.path.join(output_dir, 'failure_lie_map.png'))
    plt.close()

    print("Strategic Intelligence Report Generated.")
    print(f"Outputs saved to: {output_dir}")

# Run the system
if __name__ == "__main__":
    generate_strategic_dashboard()