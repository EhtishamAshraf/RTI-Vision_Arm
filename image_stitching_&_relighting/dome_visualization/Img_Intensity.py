# Code to plot the intensity of a specific row in a grayscale image for an image generated with a specific light source direction:
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

image = Image.open(r"C:\Users\ad\Downloads\Thesis\data\retrato de lola flores\data\AcqB_cropped\LDR_10_Theta_300.00_Phi_43.33.png").convert("L") # change path accordingly
img_array = np.array(image)

# Choose a specific row
row_index = 100 
row_intensity = img_array[row_index, :]

# Plot
plt.figure(figsize=(12, 4))
plt.bar(range(len(row_intensity)), row_intensity, color='gray')
plt.title(f"Pixel Intensity Along Row {row_index}")
plt.xlabel("Pixel Position")
plt.ylabel("Intensity (0-255)")
plt.tight_layout()

plt.savefig("row_intensity_plot.png", dpi=300) 
plt.show()
