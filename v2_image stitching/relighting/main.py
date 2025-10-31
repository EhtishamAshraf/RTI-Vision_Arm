"""
The main script for relighting images using different models.
1. PTM
2. HSH
3. DMD
"""
from ptm import PTMModel
from hsh import HSHModel
from dmd import DMDModel
from utils import dataset
from utils import params
import numpy as np
import cv2

# Load dataset
data_ = dataset(params.ACQ_PATH)

target_images = np.array(data_.images)
lps_cartesian = np.array(data_.lps_cartesian)
save_paths = data_.save_paths

print("Original target images shape: ", target_images.shape)
print("Shape of LPs cartesians: ", lps_cartesian.shape)

# ------------------------
# Downsample images to save memory
# ------------------------
scale = 0.5   # Reduce some % of original resolution (adjust as needed)
resized_images = [
    cv2.resize(img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    for img in target_images
]
target_images = np.stack(resized_images, axis=0)

print("Downsampled target images shape: ", target_images.shape)

"""
Uncomment the model you want to use!
If you want to relight from a new light position, simply define the new light positions,
directly in the code and update it like this: ptm.relight(lps_cartesian=new_lps, save_paths=save_paths, target_images=None)
"""

# For PTM
ptm = PTMModel()
coeffs = ptm.model_fit(lps_cartesian=lps_cartesian, target_images=target_images)
relit_images = ptm.relight(
    lps_cartesian, 
    target_images=target_images, 
    save_paths=save_paths
)

# For HSH
# hsh = HSHModel()
# coeffs = hsh.model_fit(lps_cartesian=lps_cartesian, target_images=target_images)
# relit_images = hsh.relight(lps_cartesian=lps_cartesian, 
#                            target_images=target_images, 
#                            save_paths=save_paths)

# For DMD
# dmd = DMDModel()
# coeffs = dmd.model_fit(lps_cartesian=lps_cartesian, target_images=target_images)
# relit_images = dmd.relight(lps_cartesian=lps_cartesian,
#                            target_images=target_images, 
#                            save_paths=save_paths)
