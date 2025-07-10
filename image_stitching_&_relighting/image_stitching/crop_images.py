"""Crop images in a folder by specified pixel values."""

import cv2
import os

def crop_images(input_folder, output_folder, crop_top, crop_bottom, crop_left, crop_right):
    output_folder.mkdir(parents=True, exist_ok=True)

    for filename in sorted(os.listdir(input_folder)):
        if not filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            continue

        img_path = input_folder / filename
        img = cv2.imread(str(img_path))

        if img is None:
            print(f"Skipping {filename}, image could not be read.")
            continue

        h, w = img.shape[:2]
        cropped_img = img[
            crop_top : h - crop_bottom,
            crop_left : w - crop_right
        ]

        output_path = output_folder / filename
        cv2.imwrite(str(output_path), cropped_img)
        print(f"Cropped and saved: {filename}")

    print("Done cropping images.")
