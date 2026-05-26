import os
import glob
import cv2
from ultralytics import YOLO

# 1. CONFIGURATION
# Point this to where your dataset_logger.py saved the raw images
DATASET_DIR = "/media/abolghasem/51AC-250D/test_set"

# Load YOLOv8 (The 'n' stands for nano - it's fast and will download automatically the first time)
print("Loading YOLO11-Pose Model...")
model = YOLO('yolo11s-pose.pt') 

def process_offline_dataset():
    # Find all the raw images you recorded in the lab
    raw_images = sorted(glob.glob(os.path.join(DATASET_DIR, "*_raw.jpg")))
    
    if not raw_images:
        print(f"No raw images found in {DATASET_DIR}")
        return

    print(f"Found {len(raw_images)} raw images to process.\n")

    for img_path in raw_images:
        filename = os.path.basename(img_path)
        print(f"Annotating: {filename}")
        
        # Read the image using OpenCV
        img = cv2.imread(img_path)
        if img is None:
            continue

        annotated_img = img.copy()
        # Run YOLO inference
        results = model(img, conf=0.60, classes=[0], verbose=False)
        
        # Extract the bounding boxes
        human_boxes = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Class 0 in YOLO is always "person"
                if int(box.cls[0]) == 0:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    human_boxes.append([int(x1), int(y1), int(x2), int(y2)])
        
        # THE LEVERAGE: Sort humans from Left to Right (based on the x1 coordinate)
        # This guarantees ID 1 is always the leftmost person, exactly matching your Gemini prompt!
        human_boxes = sorted(human_boxes, key=lambda x: x[0])

        # Draw the boxes and IDs
        for idx, (x1, y1, x2, y2) in enumerate(human_boxes):
            human_id = idx + 1 # Start IDs at 1
            
            # Draw a thick Green box (Color: BGR -> 0, 255, 0)
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), thickness=1)
            
            # Draw the ID label background and text
            label = f"ID: {human_id}"
            (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated_img, (x1, y1 - text_h - 10), (x1 + text_w, y1), (0, 255, 0), -1)
            cv2.putText(annotated_img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Save the new annotated image right next to the raw one
        annotated_filename = img_path.replace("_raw.jpg", "_annotated.jpg")
        cv2.imwrite(annotated_filename, annotated_img)
        print(f"  -> Saved {len(human_boxes)} humans to {os.path.basename(annotated_filename)}")

if __name__ == "__main__":
    process_offline_dataset()
    print("\nAnnotation complete! You are ready to run Boy!")