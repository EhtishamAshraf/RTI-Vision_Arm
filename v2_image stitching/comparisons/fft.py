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

    # Creating a low frequency window
    mask_low = np.zeros_like(magnitude, dtype=np.uint8)
    mask_low[cy - low_h:cy + low_h, cx - low_w:cx + low_w] = 1

    low_energy = np.sum(magnitude * mask_low)
    total_energy = np.sum(magnitude)

    hf_ratio = (total_energy - low_energy) / (total_energy + 1e-8)
    return hf_ratio

# Function to analyze image pairs
def analyze_pair(simple_path, multiband_path, mask_path):
    simple_img = cv2.imread(str(simple_path))
    multiband_img = cv2.imread(str(multiband_path))
    mask_img = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)

    if simple_img is None or multiband_img is None or mask_img is None:
        print(f"Missing file: {simple_path}, {multiband_path}, {mask_path}")
        return None

    if multiband_img.shape != simple_img.shape:
        multiband_img = cv2.resize(multiband_img, (simple_img.shape[1], simple_img.shape[0]))
    if mask_img.shape != simple_img.shape[:2]:
        mask_img = cv2.resize(mask_img, (simple_img.shape[1], simple_img.shape[0]))

    # FFT metrics
    fft_simple = compute_fft_ratio(simple_img, mask_img)
    fft_multiband = compute_fft_ratio(multiband_img, mask_img)

    return {
        "Image": simple_path.name,
        "FFT_Simple": fft_simple,
        "FFT_Multiband": fft_multiband
    }

def main(simple_folder, multiband_folder, mask_folder, output_folder):
    simple_folder = Path(simple_folder)
    multiband_folder = Path(multiband_folder)
    mask_folder = Path(mask_folder)
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    summaries = []
    for idx, simple_img_path in enumerate(simple_folder.glob("*.jpg"), start=1):
        multiband_img_path = multiband_folder / simple_img_path.name
        mask_path = mask_folder / f"mask_simple_{simple_img_path.name}"
        print(f"Analyzing {simple_img_path.name}...")

        summary = analyze_pair(simple_img_path, multiband_img_path, mask_path)
        if summary:
            summary["ID"] = idx 
            summaries.append(summary)

    if not summaries:
        print("No valid image pairs found.")
        return

    df = pd.DataFrame(summaries)
    df.to_csv(output_folder / "fft_summary.csv", index=False)
    print(f"FFT summary saved: {output_folder / 'fft_summary.csv'}")

    plt.figure(figsize=(8,6))
    for col, label in [("FFT_Simple", "Simple"), ("FFT_Multiband", "Multiband")]:
        sorted_vals = np.sort(df[col])
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        plt.plot(sorted_vals, cdf, label=label)

    plt.xlabel("High-Frequency Ratio")
    plt.ylabel("Cumulative Probability")
    plt.title("CDF of High-Frequency Ratios in Overlap [Without Light Correction]")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(output_folder / "fft_cdf.png", dpi=200)
    plt.close()
    print(f"CDF plot saved: {output_folder / 'fft_cdf.png'}")

    plt.figure(figsize=(12,6))
    plt.plot(df["ID"], df["FFT_Simple"], label="Simple", marker="o", alpha=0.7)
    plt.plot(df["ID"], df["FFT_Multiband"], label="Multiband", marker="s", alpha=0.7)
    plt.xlabel("Image ID")
    plt.ylabel("High-Frequency Ratio")
    plt.title("Line Plot of High-Frequency Ratios Across Images")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(output_folder / "fft_lineplot.png", dpi=200)
    plt.close()
    print(f"Line plot saved: {output_folder / 'fft_lineplot.png'}")

if __name__ == "__main__":
    simple_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\A1234_B1234"
    multiband_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\multiband_blending\A1234_B1234"
    mask_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\simple_mask"
    output_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\nonlightcorrected\fft_analysis"
    
    # simple_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\lightcorrected\without_blending\results"
    # multiband_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\lightcorrected\multiband\results"
    # mask_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\simple_mask"
    # output_folder = r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\comparisons\lightcorrected\fft_analysis"
    main(simple_folder, multiband_folder, mask_folder, output_folder)
