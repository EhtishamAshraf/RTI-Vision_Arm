"""
Code to compare the sharpness of overlapping regions in simple stitching (no blending) and multiband blending. 
    It calculates gradient magnitudes (using Sobel filters) inside the overlap masks.
"""

import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Function to calculate gradient magnitude
def compute_gradient_map(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    grad_mag = cv2.magnitude(grad_x, grad_y)
    return grad_mag

# Function to analyse pairs
def analyze_pair(simple_path, multiband_path, mask_path):
    simple_img = cv2.imread(str(simple_path))
    multiband_img = cv2.imread(str(multiband_path))
    mask_img = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)

    if simple_img is None or multiband_img is None or mask_img is None:
        return None

    if multiband_img.shape != simple_img.shape:
        multiband_img = cv2.resize(multiband_img, (simple_img.shape[1], simple_img.shape[0]))
    if mask_img.shape != simple_img.shape[:2]:
        mask_img = cv2.resize(mask_img, (simple_img.shape[1], simple_img.shape[0]))

    grad_simple = compute_gradient_map(simple_img)
    grad_multiband = compute_gradient_map(multiband_img)

    grad_simple_vals = grad_simple[mask_img > 0].flatten()
    grad_multiband_vals = grad_multiband[mask_img > 0].flatten()

    return {
        "Image": simple_path.name,
        "Simple_Mean": np.mean(grad_simple_vals),
        "Multiband_Mean": np.mean(grad_multiband_vals)
    }

def process_dataset(simple_folder, multiband_folder, mask_folder):
    simple_folder = Path(simple_folder)
    multiband_folder = Path(multiband_folder)
    mask_folder = Path(mask_folder)

    summaries = []
    for i, simple_img_path in enumerate(sorted(simple_folder.glob("*.jpg"))):
        mask_path = mask_folder / f"mask_simple_{simple_img_path.name}"
        multiband_img_path = multiband_folder / simple_img_path.name
        summary = analyze_pair(simple_img_path, multiband_img_path, mask_path)
        if summary:
            summary["Index"] = i+1
            summaries.append(summary)

    return pd.DataFrame(summaries)

def main():
    # Paths
    nonlight_simple = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\A1234_B1234"
    nonlight_multiband = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\multiband_blending\A1234_B1234"
    light_simple = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\lightcorrected\without_blending\results"
    light_multiband = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\lightcorrected\multiband\results"
    mask_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\simple_mask"
    output_folder = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\combined_gradient_analysis")
    output_folder.mkdir(parents=True, exist_ok=True)

    # Processing both datasets
    df_nonlight = process_dataset(nonlight_simple, nonlight_multiband, mask_folder)
    df_light = process_dataset(light_simple, light_multiband, mask_folder)

    # Plot - Simple stitching
    plt.figure(figsize=(18,6))
    x = np.arange(len(df_nonlight))
    plt.bar(x - 0.2, df_nonlight["Simple_Mean"], width=0.4, label="Simple (Non-Light)", color="lightgreen")
    plt.bar(x + 0.2, df_light["Simple_Mean"], width=0.4, label="Simple (Light Corrected)", color="blue")

    overall_nonlight_simple = df_nonlight["Simple_Mean"].mean()
    overall_light_simple = df_light["Simple_Mean"].mean()

    tick_step = max(1, len(df_nonlight)//30)
    plt.xticks(x[::tick_step], df_nonlight["Index"][::tick_step])
    plt.xlabel("Image Index")
    plt.ylabel("Mean Gradient Magnitude in Overlap")
    plt.title("Simple (No Blend): Gradient Comparison")
    plt.legend()
    plt.text(0.95, 0.95,
             f"Means:\nNon-Light: {overall_nonlight_simple:.2f}\nLight-Corrected: {overall_light_simple:.2f}",
             horizontalalignment='right', verticalalignment='top',
             transform=plt.gca().transAxes,
             bbox=dict(facecolor='white', alpha=0.6, edgecolor='black'))
    plt.tight_layout()
    plt.savefig(output_folder / "simple_comparison.png", dpi=200)
    plt.close()

    # Plot - Multiband stitching
    plt.figure(figsize=(18,6))
    plt.bar(x - 0.2, df_nonlight["Multiband_Mean"], width=0.4, label="Multiband (Non-Light)", color="lightgreen")
    plt.bar(x + 0.2, df_light["Multiband_Mean"], width=0.4, label="Multiband (Light Corrected)", color="blue")

    overall_nonlight_multiband = df_nonlight["Multiband_Mean"].mean()
    overall_light_multiband = df_light["Multiband_Mean"].mean()

    plt.xticks(x[::tick_step], df_nonlight["Index"][::tick_step])
    plt.xlabel("Image Index")
    plt.ylabel("Mean Gradient Magnitude in Overlap")
    plt.title("Multiband Blend: Gradient Comparison")
    plt.legend()
    plt.text(0.95, 0.95,
             f"Means:\nNon-Light: {overall_nonlight_multiband:.2f}\nLight-Corrected: {overall_light_multiband:.2f}",
             horizontalalignment='right', verticalalignment='top',
             transform=plt.gca().transAxes,
             bbox=dict(facecolor='white', alpha=0.6, edgecolor='black'))
    plt.tight_layout()
    plt.savefig(output_folder / "multiband_comparison.png", dpi=200)
    plt.close()

    print(f"Plots saved in {output_folder}")

if __name__ == "__main__":
    main()
