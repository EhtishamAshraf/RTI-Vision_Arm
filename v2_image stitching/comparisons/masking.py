"""
Code to generate binary overlap masks from saved overlap preview images.
    These masks are later used for quantitative evaluation.
"""

import cv2
import numpy as np
from pathlib import Path

# Function to extract overlap mask from overlap preview image
def extract_overlap_mask(overlap_img_path, save_path):
    img = cv2.imread(str(overlap_img_path))
    if img is None:
        print(f"Could not read {overlap_img_path}")
        return
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Defining red color ranges in HSV
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Creating mask for red pixels
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Finding the Bbox region from red pixels
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print(f"No red bounding box found in {overlap_img_path}")
        return

    # Assuming largest contour is the overlap box
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))

    # Creating binary mask with overlap region
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    mask[y:y+h, x:x+w] = 255

    cv2.imwrite(str(save_path), mask)
    print(f"Mask saved: {save_path}")


def main(overlap_folder, mask_folder):
    overlap_folder = Path(overlap_folder)
    mask_folder = Path(mask_folder)
    mask_folder.mkdir(parents=True, exist_ok=True)

    for overlap_img in overlap_folder.glob("overlap_*.jpg"):
        mask_path = mask_folder / overlap_img.name.replace("overlap_", "mask_")
        extract_overlap_mask(overlap_img, mask_path)

if __name__ == "__main__":
    overlap_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\overlap_folder"
    mask_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\simple_mask"
    main(overlap_folder, mask_folder)
