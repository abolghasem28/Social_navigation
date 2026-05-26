import os
from collections import defaultdict

# Set your base directory path here
base_dir = "/path/to/your/images/folder"  # Change this to your actual path

# Define your scenario keywords
scenarios = [
    "conversation",
    "ignoring", 
    "photography",
    "playing",
    "presenting",
    "proxemics",
    "sharedtask"
]

# Define location keywords (optional, remove if not needed)
locations = ["robotics", "corridor", "1stfloor", "kitchen"]

def count_images(directory, extensions={'.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG'}):
    """Count images per scenario and location"""
    
    scenario_count = defaultdict(int)
    scenario_location_count = defaultdict(lambda: defaultdict(int))
    total_count = 0
    
    for root, dirs, files in os.walk(directory):
        for file in files:
            if any(file.lower().endswith(ext) for ext in extensions):
                total_count += 1
                file_lower = file.lower()
                
                # Check which scenario this file belongs to
                for scenario in scenarios:
                    if scenario in file_lower:
                        scenario_count[scenario] += 1
                        
                        # Check for location in path or filename
                        path_lower = root.lower() + file_lower
                        location_found = False
                        for location in locations:
                            if location in path_lower:
                                scenario_location_count[scenario][location] += 1
                                location_found = True
                                break
                        if not location_found:
                            scenario_location_count[scenario]["unknown"] += 1
                        break
    
    return scenario_count, scenario_location_count, total_count

# Run the script
if __name__ == "__main__":
    # Method 1: Ask user for path
    user_path = input("Enter the path to your images folder: ").strip()
    
    if os.path.exists(user_path):
        scenario_counts, location_counts, total = count_images(user_path)
        
        print("\n" + "="*60)
        print("IMAGE COUNT PER SCENARIO")
        print("="*60)
        
        for scenario in scenarios:
            count = scenario_counts.get(scenario, 0)
            print(f"{scenario.capitalize():15} : {count} images")
            
            # Print location breakdown if available
            if location_counts[scenario]:
                print(f"   Locations:")
                for loc, loc_count in location_counts[scenario].items():
                    print(f"      - {loc.capitalize()}: {loc_count}")
        
        print("\n" + "="*60)
        print(f"TOTAL IMAGES: {total}")
        print("="*60)
        
    else:
        print(f"Error: Path '{user_path}' does not exist.")