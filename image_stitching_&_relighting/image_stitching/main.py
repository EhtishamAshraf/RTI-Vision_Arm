"""Image Stitching Pipeline
This script implements a complete pipeline for image stitching, including cropping, alignment, light correction, stitching of images and blending.

Note: The .lp file must have the total number of light positions written as an interger in the first line. Like: 23, 24 etc
Note: This script only works for two acquisitions (AcqA and AcqB) at a time. It will not work for more than two acquisitions.
"""


from crop_images import crop_images
from Subpixel_Alignment import align_folders
from homomorphic_lightcorrection import HomomorphicFilter
from stitching import stitch_images
from paths import get_stitching_paths, get_crop_paths, get_alignment_paths, get_light_correction_paths
from acq_data import Acq
import shutil
from pathlib import Path

use_light_correction = True  # Set to False if you want to stitch without light correction

# Function to copy .lp files from raw acquisition folders to cropped and aligned folders
def copy_lp_files(acqA_raw, acqB_raw, acqA_cropped, acqB_cropped, acqA_aligned, acqB_aligned):
    for src, cropped_dst, aligned_dst in [
        (acqA_raw, acqA_cropped, acqA_aligned),
        (acqB_raw, acqB_cropped, acqB_aligned)
    ]:
        lp_files = list(Path(src).glob("*.lp"))
        if not lp_files:
            print(f"No .lp files found in {src}")
            continue

        for lp_file in lp_files:
            shutil.copy(lp_file, Path(cropped_dst) / lp_file.name)
            shutil.copy(lp_file, Path(aligned_dst) / lp_file.name)

# Main Stitching Pipeline
def main():
    
    print("STEP 1: CROP RAW ACQUISITIONS")
    acqA_raw, acqA_cropped = get_crop_paths("AcqA")
    acqB_raw, acqB_cropped = get_crop_paths("AcqB")
    crop_images(acqA_raw, acqA_cropped, 70, 0, 255, 345)
    crop_images(acqB_raw, acqB_cropped, 0, 0, 170, 410)


    print("STEP 2: Images ALIGNMENT")
    align_folders("AcqA", "AcqB")

    _, acqA_aligned = get_alignment_paths("AcqA")
    _, acqB_aligned = get_alignment_paths("AcqB")

    print("Copying .lp files to cropped and aligned folders")
    copy_lp_files(
        acqA_raw, acqB_raw,
        acqA_cropped, acqB_cropped,
        acqA_aligned, acqB_aligned
    )

    print("STEP 3: LIGHT CORRECTION")
    acqA_aligned, acqB_aligned, acqA_lightcorrected, acqB_lightcorrected = get_light_correction_paths()
    pivot_acq = Acq(acqA_aligned)
    joint_acq = Acq(acqB_aligned)

    HomomorphicFilter(pivot_acq, acqA_lightcorrected)
    HomomorphicFilter(joint_acq, acqB_lightcorrected)


    print("STEP 4: STITCHING")
    stitchA, stitchB, stitch_out, matches_folder, overlap_folder = get_stitching_paths(use_light_correction)
    stitch_images(stitchA, stitchB, stitch_out / "no_blending", matches_folder, overlap_folder, use_multiband=False) # Stitching images without multiband blending
    # stitch_images(stitchA, stitchB, stitch_out / "multiband", matches_folder, overlap_folder, use_multiband=True) # Stitching images with multiband blending

    print("Success: Pipeline completed!")

if __name__ == "__main__":
    main()
