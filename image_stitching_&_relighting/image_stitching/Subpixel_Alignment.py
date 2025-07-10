# """
# Python script for aligning two images using the Enhanced Correlation Coefficient (ECC) method.
# -   This script reads two images, computes the alignment using ECC, save the aligned images and plot the comparison.
# -   It also computes the Mean Squared Error (MSE) and SSIM before and after alignment.
# -   This script is designed to work with grayscale images and uses OpenCV for image processing.
# """


import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
from skimage.metrics import structural_similarity as ssim

from paths import get_alignment_paths

valid_exts = ('.jpg', '.jpeg', '.png')

# ---------------------- Helper Functions ----------------------

def get_image_list(folder):
    return sorted([f for f in os.listdir(folder) if f.lower().endswith(valid_exts)])

def compute_mse(img1, img2):
    return mean_squared_error(img1.flatten(), img2.flatten())

def get_gradient(image):
    grad_x = cv2.Sobel(image, cv2.CV_32F, 1, 0)
    grad_y = cv2.Sobel(image, cv2.CV_32F, 0, 1)
    grad = cv2.addWeighted(cv2.convertScaleAbs(grad_x), 0.5,
                           cv2.convertScaleAbs(grad_y), 0.5, 0)
    return grad

def align_images(reference, target, use_gradient=True):
    warp_mode = cv2.MOTION_EUCLIDEAN
    warp_matrix = np.eye(2, 3, dtype=np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1000, 1e-6)

    ref_input = get_gradient(reference).astype(np.float32) if use_gradient else np.float32(reference)
    tgt_input = get_gradient(target).astype(np.float32) if use_gradient else np.float32(target)

    try:
        cc, warp_matrix = cv2.findTransformECC(ref_input, tgt_input, warp_matrix, warp_mode, criteria)
        aligned = cv2.warpAffine(target, warp_matrix, (reference.shape[1], reference.shape[0]),
                                 flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
        return aligned, True, warp_matrix, cc
    except cv2.error as e:
        print(f"ECC alignment failed: {e}")
        return target, False, warp_matrix, None

# ---------------------- Main Function ----------------------

def align_folder(folder_path, save_dir):
    image_files = get_image_list(folder_path)
    print(f"\n[INFO] Found {len(image_files)} images in {folder_path}")
    
    images = []
    for fname in image_files:
        img_path = os.path.join(folder_path, fname)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            images.append(img)
        else:
            print(f"Failed to read: {fname}")

    if len(images) == 0:
        print("No images to process.")
        return

    reference = images[0]
    mse_before_list = []
    mse_after_list = []
    ssim_before_list = []
    ssim_after_list = []

    for i, target in enumerate(images):
        target_name = image_files[i]

        if i == 0:
            mse_before_list.append(0)
            mse_after_list.append(0)
            ssim_before_list.append(1.0)
            ssim_after_list.append(1.0)
            print(f"\n[{i}] Image: {target_name}")
            print("    Skipped alignment (reference image)")
            aligned = reference
        else:
            mse_before = compute_mse(reference, target)
            aligned, success, warp_matrix, cc = align_images(reference, target)
            mse_after = compute_mse(reference, aligned)

            ssim_b = ssim(reference, target)
            ssim_a = ssim(reference, aligned)

            mse_before_list.append(mse_before)
            mse_after_list.append(mse_after)
            ssim_before_list.append(ssim_b)
            ssim_after_list.append(ssim_a)

            print(f"\n[{i}] Image: {target_name}")
            print(f"    MSE Before: {mse_before:.2f}")
            print(f"    MSE After:  {mse_after:.2f}")
            print(f"    SSIM Before: {ssim_b:.4f}")
            print(f"    SSIM After:  {ssim_a:.4f}")

            if success:
                print(f"    Correlation Coefficient (ECC): {cc:.4f}")
            else:
                print("    Alignment failed.")

        # Save aligned image
        aligned_name = target_name
        aligned_path = os.path.join(save_dir, aligned_name)
        cv2.imwrite(aligned_path, aligned)
        print(f"    Saved aligned image: {aligned_path}")

    # Plot MSE
    mean_mse_before = np.mean(mse_before_list)
    mean_mse_after = np.mean(mse_after_list)

    plt.figure(figsize=(12, 6))
    plt.plot(mse_before_list, 'r-o', label='MSE Before Alignment')
    plt.plot(mse_after_list, 'g-o', label='MSE After Alignment')
    plt.title("MSE Before vs After Alignment", fontsize=14)
    plt.xlabel("Image Index")
    plt.ylabel("MSE")
    plt.legend()
    plt.grid(True)
    textstr = f"Mean MSE Before: {mean_mse_before:.2f}\nMean MSE After: {mean_mse_after:.2f}"
    plt.gcf().text(0.72, 0.75, textstr, fontsize=12,
                   bbox=dict(boxstyle="round,pad=0.5", fc="#f0f0f0", ec="gray", alpha=0.95))
    plt.tight_layout()
    plot_dir = os.path.join(save_dir, "analysis")
    os.makedirs(plot_dir, exist_ok=True)
    plt.savefig(os.path.join(plot_dir, "ECC-mse_comparison_plot.png"), dpi=300)
    plt.close()

    # Plot SSIM
    mean_ssim_before = np.mean(ssim_before_list)
    mean_ssim_after = np.mean(ssim_after_list)

    plt.figure(figsize=(12, 6))
    plt.plot(ssim_before_list, 'b-o', label='SSIM Before Alignment')
    plt.plot(ssim_after_list, 'm-o', label='SSIM After Alignment')
    plt.title("SSIM Before vs After Alignment", fontsize=14)
    plt.xlabel("Image Index")
    plt.ylabel("SSIM")
    plt.legend()
    plt.grid(True)
    textstr = f"Mean SSIM Before: {mean_ssim_before:.4f}\nMean SSIM After: {mean_ssim_after:.4f}"
    plt.gcf().text(0.72, 0.75, textstr, fontsize=12,
                   bbox=dict(boxstyle="round,pad=0.5", fc="#f0f0f0", ec="gray", alpha=0.95))
    plt.tight_layout()

    plt.savefig(os.path.join(plot_dir, "ECC-ssim_comparison_plot.png"), dpi=300)
    plt.close()

    print("--" * 20)
    print(f"Mean MSE Before Alignment: {mean_mse_before:.2f}")
    print(f"Mean MSE After Alignment:  {mean_mse_after:.2f}")
    print(f"Mean SSIM Before Alignment: {mean_ssim_before:.4f}")
    print(f"Mean SSIM After Alignment:  {mean_ssim_after:.4f}")
    print("--" * 20)

def align_folders(acqA_folder, acqB_folder):
    print(f"\n[ALIGN] Aligning folder: {acqA_folder}")
    acqA_input, acqA_output = get_alignment_paths(acqA_folder)
    align_folder(acqA_input, acqA_output)

    print(f"\n[ALIGN] Aligning folder: {acqB_folder}")
    acqB_input, acqB_output = get_alignment_paths(acqB_folder)
    align_folder(acqB_input, acqB_output)

