"""
This script compares stitched images from simple stitching (no blending) and multiband blending using 
frequency domain analysis. It computes the ratio of high-frequency energy (sharp details) within overlap regions, 
based on FFT (2D Fast Fourier Transform). 
"""

import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Function to compute FFT and high-frequency ratio
def compute_fft_ratio(image, mask, low_freq_fraction=0.1):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if mask.shape != gray.shape:
        mask = cv2.resize(mask, (gray.shape[1], gray.shape[0]))

    gray_masked = cv2.bitwise_and(gray, gray, mask=mask)

    f = np.fft.fft2(gray_masked)
    fshift = np.fft.fftshift(f)
    magnitude = np.abs(fshift)

    h, w = magnitude.shape
    cy, cx = h // 2, w // 2
    low_h, low_w = int(h * low_freq_fraction), int(w * low_freq_fraction)

    mask_low = np.zeros_like(magnitude, dtype=np.uint8)
    mask_low[cy - low_h:cy + low_h, cx - low_w:cx + low_w] = 1

    low_energy = np.sum(magnitude * mask_low)
    total_energy = np.sum(magnitude)

    hf_ratio = (total_energy - low_energy) / (total_energy + 1e-8)
    return hf_ratio

# Analyze single pair
def analyze_pair(simple_path, multiband_path, mask_path):
    simple_img = cv2.imread(str(simple_path))
    multiband_img = cv2.imread(str(multiband_path))
    mask_img = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)

    if simple_img is None or multiband_img is None or mask_img is None:
        print(f"Missing file: {simple_path.name}")
        return None

    if multiband_img.shape != simple_img.shape:
        multiband_img = cv2.resize(multiband_img, (simple_img.shape[1], simple_img.shape[0]))
    if mask_img.shape != simple_img.shape[:2]:
        mask_img = cv2.resize(mask_img, (simple_img.shape[1], simple_img.shape[0]))

    fft_simple = compute_fft_ratio(simple_img, mask_img)
    fft_multiband = compute_fft_ratio(multiband_img, mask_img)

    return {"FFT_Simple": fft_simple, "FFT_Multiband": fft_multiband}

# Process dataset 
def process_dataset(simple_folder, multiband_folder, mask_folder):
    simple_folder = Path(simple_folder)
    multiband_folder = Path(multiband_folder)
    mask_folder = Path(mask_folder)

    simple_vals, multiband_vals = [], []
    for simple_img_path in sorted(simple_folder.glob("*.jpg")):
        multiband_img_path = multiband_folder / simple_img_path.name
        mask_path = mask_folder / f"mask_simple_{simple_img_path.name}"
        summary = analyze_pair(simple_img_path, multiband_img_path, mask_path)
        if summary:
            simple_vals.append(summary["FFT_Simple"])
            multiband_vals.append(summary["FFT_Multiband"])

    return np.array(simple_vals), np.array(multiband_vals)

# Main function to compare and plot
def main(nonlight_simple, nonlight_multiband, light_simple, light_multiband, mask_folder, output_folder):
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    # Process both datasets
    simple_nonlight, multiband_nonlight = process_dataset(nonlight_simple, nonlight_multiband, mask_folder)
    simple_light, multiband_light = process_dataset(light_simple, light_multiband, mask_folder)

    plt.figure(figsize=(10,6))

    def plot_cdf(values, label, color):
        sorted_vals = np.sort(values)
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        plt.plot(sorted_vals, cdf, label=label, color=color)

    plot_cdf(multiband_light, "Multiband (Light Corrected)", "blue")
    plot_cdf(multiband_nonlight, "Multiband (Non Light Corrected)", "darkgreen")
    plot_cdf(simple_light, "Simple (Light Corrected)", "skyblue")
    plot_cdf(simple_nonlight, "Simple (Non Light Corrected)", "lightgreen")

    plt.xlabel("High-Frequency Ratio")
    plt.ylabel("Cumulative Probability")
    plt.title("CDF of High-Frequency Ratios in Overlap\nLight Corrected vs Non Light Corrected")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(output_folder / "fft_cdf_comparison.png", dpi=200)
    plt.close()

    print("CDF plot saved at:", output_folder / "fft_cdf_comparison.png")


if __name__ == "__main__":
    # Non-light corrected
    nonlight_simple = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\A1234_B1234"
    nonlight_multiband = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\multiband_blending\A1234_B1234"
    
    # Light corrected
    light_simple = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\lightcorrected\without_blending\results"
    light_multiband = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\lightcorrected\multiband\results"
    
    # Masks + Output
    mask_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\simple_mask"
    output_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\fft_analysis_comparison"

    main(nonlight_simple, nonlight_multiband, light_simple, light_multiband, mask_folder, output_folder)
