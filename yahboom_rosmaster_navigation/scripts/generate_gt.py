import os
import csv
from itertools import combinations

def generate_annotation_template(image_dir, output_file):
    """
    Scans a directory for .jpg and .png images and generates a CSV template
    for pair-level social boundary annotation.
    """
    # Define the 4 individuals and generate the 6 unique pairs
    people = [1, 2, 3, 4]
    pairs = [f"[{p1}, {p2}]" for p1, p2 in combinations(people, 2)]
    
    valid_extensions = ('.jpg', '.jpeg', '.png')
    
    # Check if directory exists
    if not os.path.exists(image_dir):
        print(f"Error: Directory '{image_dir}' not found.")
        return

    # Count processed files for verification
    processed_count = 0

    with open(output_file, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['Filename', 'Image_Type', 'Pair','Level_Engagement', 'Actual_Status'])

        # Iterate through files in the target directory
        files = sorted([f for f in os.listdir(image_dir) if f.lower().endswith(valid_extensions)])
        
        for filename in files:
            # Determine Image_Type from filename
            img_type = "Unknown"
            if "raw" in filename.lower():
                img_type = "Raw"
            elif "annotated" in filename.lower():
                img_type = "Annotated"
            elif "depth" in filename.lower():
                img_type = "Depth"

            # Write rows for all potential pairs
            for pair in pairs:
                writer.writerow([filename, img_type, pair, '', ''])
            
            processed_count += 1
                
    print(f"Execution complete.")
    print(f"Processed {processed_count} images.")
    print(f"Generated {processed_count * 6} rows in '{output_file}'.")


IMAGE_DIRECTORY = '/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test' 
OUTPUT_CSV_NAME = '/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/ground_truth_template.csv'

generate_annotation_template(IMAGE_DIRECTORY, OUTPUT_CSV_NAME)