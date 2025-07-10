# """
# This script generates a bar plot comparing the performance of three models (PTM, HSH, DMD) based on three metrics: MSE, PSNR, and SSIM.
# The plot is saved as a PNG file.
# """

import matplotlib.pyplot as plt
import numpy as np

# Models
models = ['PTM', 'HSH', 'DMD']
x = np.arange(len(models))  # the label locations
width = 0.35  # width of the bars

# Metric values get them after running the models on the same dataset:
# Before light correction
mse_vals_before = [55.56, 42.96, 68.45]
psnr_vals_before = [30.82, 32.11, 29.97]
ssim_vals_before = [0.8155, 0.8702, 0.7398]

# After light correction 
mse_vals_after = [43.9, 34.82, 50.75]
psnr_vals_after = [31.85, 33.19, 31.21]
ssim_vals_after = [0.8497, 0.8934, 0.8060]

# Create subplots
fig, axs = plt.subplots(1, 3, figsize=(18, 5))

# MSE Plot
axs[0].bar(x - width/2, mse_vals_before, width, label='Before', color='lightcoral')
axs[0].bar(x + width/2, mse_vals_after, width, label='After', color='indianred')
axs[0].set_title('Mean MSE')
axs[0].set_ylabel('Error (lower is better)')
axs[0].set_xticks(x)
axs[0].set_xticklabels(models)
axs[0].set_ylim([0, max(mse_vals_before + mse_vals_after) + 5])
axs[0].legend()
axs[0].grid(True)

# PSNR Plot
axs[1].bar(x - width/2, psnr_vals_before, width, label='Before', color='lightblue')
axs[1].bar(x + width/2, psnr_vals_after, width, label='After', color='steelblue')
axs[1].set_title('Mean PSNR')
axs[1].set_ylabel('dB (higher is better)')
axs[1].set_xticks(x)
axs[1].set_xticklabels(models)
axs[1].set_ylim([min(psnr_vals_before + psnr_vals_after) - 1, max(psnr_vals_before + psnr_vals_after) + 1])
axs[1].legend()
axs[1].grid(True)

# SSIM Plot
all_ssim_vals = ssim_vals_before + ssim_vals_after
axs[2].bar(x - width/2, ssim_vals_before, width, label='Before', color='lightgreen')
axs[2].bar(x + width/2, ssim_vals_after, width, label='After', color='seagreen')
axs[2].set_title('Mean SSIM')
axs[2].set_ylabel('Index (0 to 1, higher is better)')
axs[2].set_xticks(x)
axs[2].set_xticklabels(models)
axs[2].set_ylim([min(all_ssim_vals) - 0.02, max(all_ssim_vals) + 0.02])  # <-- dynamic range
axs[2].legend()
axs[2].grid(True)


# Final layout
plt.suptitle('Model Performance Comparison (PTM, HSH, DMD) – Before vs After Light Correction', fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.savefig(r"C:\Users\ad\Downloads\Thesis\data\retrato de lola flores\data\image_stitching_&_relighting\relighting\model_comparison_light_correction.png", dpi=300, bbox_inches='tight')
plt.show()
