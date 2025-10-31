"""
Code to compare the sharpness of overlapping regions in simple stitching (no blending) and multiband blending. 
    It calculates gradient magnitudes (using Sobel filters) inside the overlap masks.
"""

import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Function to calculate the gradient magnitude
def compute_gradient_map(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    grad_mag = cv2.magnitude(grad_x, grad_y)
    return grad_mag

# Function to analyse the image pairs (multiband, unblended)
def analyze_pair(simple_path, multiband_path, mask_path):
    simple_img = cv2.imread(str(simple_path))
    multiband_img = cv2.imread(str(multiband_path))
    mask_img = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)

    if simple_img is None or multiband_img is None or mask_img is None:
        print(f"Missing file: {simple_path}, {multiband_path}, or {mask_path}")
        return None

    # Resizing
    if multiband_img.shape != simple_img.shape:
        multiband_img = cv2.resize(multiband_img, (simple_img.shape[1], simple_img.shape[0]))
    if mask_img.shape != simple_img.shape[:2]:
        mask_img = cv2.resize(mask_img, (simple_img.shape[1], simple_img.shape[0]))

    # Gradients
    grad_simple = compute_gradient_map(simple_img)
    grad_multiband = compute_gradient_map(multiband_img)
    grad_diff = np.abs(grad_simple - grad_multiband)

    # Extracting gradient values within the overlap region for simple, multiband, and their difference
    grad_simple_vals = grad_simple[mask_img > 0].flatten()
    grad_multiband_vals = grad_multiband[mask_img > 0].flatten()
    grad_diff_vals = grad_diff[mask_img > 0].flatten()

    return {
        "Image": simple_path.name,
        "Simple_Mean": np.mean(grad_simple_vals),
        "Multiband_Mean": np.mean(grad_multiband_vals),
        "Mean_Difference": np.mean(grad_diff_vals)
    }

def main(simple_folder, multiband_folder, mask_folder, output_folder):
    simple_folder = Path(simple_folder)
    multiband_folder = Path(multiband_folder)
    mask_folder = Path(mask_folder)
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    # Comparing images and collecting results
    summaries = []
    for i, simple_img_path in enumerate(sorted(simple_folder.glob("*.jpg"))):
        mask_path = mask_folder / f"mask_simple_{simple_img_path.name}"
        multiband_img_path = multiband_folder / simple_img_path.name
        summary = analyze_pair(simple_img_path, multiband_img_path, mask_path)
        if summary:
            summary["Index"] = i+1 
            summaries.append(summary)

    if not summaries:
        print("No valid image pairs found.")
        return

    df = pd.DataFrame(summaries)
    df.to_csv(output_folder / "gradient_summary.csv", index=False)
    print(f"Summary saved: {output_folder / 'gradient_summary.csv'}")

    overall_simple_mean = df["Simple_Mean"].mean()
    overall_multiband_mean = df["Multiband_Mean"].mean()

    plt.figure(figsize=(18,6)) 
    x = range(len(df))
    plt.bar([i-0.15 for i in x], df["Simple_Mean"], width=0.3, label='Simple (No Blend)')
    plt.bar([i+0.15 for i in x], df["Multiband_Mean"], width=0.3, label='Multiband Blend')

    tick_step = max(1, len(df)//30)
    plt.xticks(x[::tick_step], df["Index"][::tick_step])

    plt.xlabel("Image Index")
    plt.ylabel("Mean Gradient Magnitude in Overlap")
    plt.title("Comparison of Overlap Gradients: Simple vs Multiband [Light Correction Applied]")
    plt.legend()

    plt.text(0.95, 0.95,
             f"Dataset Mean:\nSimple: {overall_simple_mean:.2f}\nMultiband: {overall_multiband_mean:.2f}",
             horizontalalignment='right', verticalalignment='top',
             transform=plt.gca().transAxes,
             bbox=dict(facecolor='white', alpha=0.6, edgecolor='black'))

    plt.tight_layout()
    plt.savefig(output_folder / "bar_chart_comparison.png", dpi=200)
    plt.close()
    print(f"Bar chart saved: {output_folder / 'bar_chart_comparison.png'}")

    plt.figure(figsize=(7,7))
    data = [df["Simple_Mean"], df["Multiband_Mean"]]
    plt.boxplot(data, labels=["Simple (No Blend)", "Multiband Blend"])
    plt.ylabel("Mean Gradient Magnitude in Overlap")
    plt.title("Distribution of Gradient Magnitudes")

    plt.text(1.9, max(df["Simple_Mean"].max(), df["Multiband_Mean"].max())*0.95,
             f"Means:\nSimple: {overall_simple_mean:.2f}\nMultiband: {overall_multiband_mean:.2f}",
             horizontalalignment='right',
             bbox=dict(facecolor='white', alpha=0.6, edgecolor='black'))

    plt.tight_layout()
    plt.savefig(output_folder / "boxplot_comparison.png", dpi=200)
    plt.close()
    print(f"Boxplot saved: {output_folder / 'boxplot_comparison.png'}")

if __name__ == "__main__":
    # simple_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\A1234_B1234"
    # multiband_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\multiband_blending\A1234_B1234"
    # mask_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\simple_mask"
    # output_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\nonlightcorrected\gradient_analysis"

    simple_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\lightcorrected\without_blending\results"
    multiband_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\lightcorrected\multiband\results"
    mask_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\simple_mask"
    output_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\lightcorrected\gradient_analysis"
    main(simple_folder, multiband_folder, mask_folder, output_folder)