import os
import csv
from itertools import combinations


"""
This script generates a CSV template, 
Use base base_id to group images by scenaris if it is identical it compares the priority operational sequence of Raw -> Annotated -> Depth or whatever we want to define
If both base_id and priority are identical it uses filename as the last sorting key to ensure consistent order."""


def generate_annotation_template(image_dir, output_file):
    """
    Generates a CSV template grouped by image scenario with the sequence:
    Raw -> Annotated -> Depth.
    """
    people = [1, 2, 3, 4]
    pairs = [f"[{p1}, {p2}]" for p1, p2 in combinations(people, 2)]
    valid_extensions = ('.jpg', '.jpeg', '.png')
    
    if not os.path.exists(image_dir):
        print(f"Error: Directory '{image_dir}' not found.")
        return

    # Define the internal sequence priority
    type_priority = {"raw": 1, "annotated": 2, "depth": 3}

    def get_sort_key(filename):
        fname_lower = filename.lower()
        # 1. Determine Type and Priority
        if "raw" in fname_lower:
            img_type, priority = "Raw", type_priority["raw"]
        elif "annotated" in fname_lower:
            img_type, priority = "Annotated", type_priority["annotated"]
        elif "depth" in fname_lower:
            img_type, priority = "Depth", type_priority["depth"]
        else:
            img_type, priority = "Unknown", 4

        parts = filename.rsplit('_', 1)
        if len(parts) > 1:
            base_id = parts[0]
        else:
            base_id = filename
        return (base_id, priority, filename)

    processed_count = 0
    with open(output_file, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['Filename', 'Image_Type', 'Pair', 'Level_Engagement', 'Actual_Status'])

        # Get and sort files using the composite key
        all_files = [f for f in os.listdir(image_dir) if f.lower().endswith(valid_extensions)]
        sorted_files = sorted(all_files, key=get_sort_key)
        
        for filename in sorted_files:
            fname_lower = filename.lower()
            img_type = "Raw" if "raw" in fname_lower else "Annotated" if "annotated" in fname_lower else "Depth" if "depth" in fname_lower else "Unknown"

            for pair in pairs:
                writer.writerow([filename, img_type, pair, '', ''])
            processed_count += 1
                
    print("Execution complete." )
    print(f"Processed {processed_count} images.")
    print(f"Total rows: {processed_count * 6}. Order: Scene -> Raw -> Annotated -> Depth.")

# Paths
IMAGE_DIRECTORY = '/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/new_annotated' 
OUTPUT_CSV_NAME = '/home/aesmaeily/ros2_ws/src/yahboom_rosmaster/dataset_images/generate_labels/eval_test/new_annotated/ground_truth_template1.csv'

generate_annotation_template(IMAGE_DIRECTORY, OUTPUT_CSV_NAME)