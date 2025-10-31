"""Image Stitching Pipeline
This script implements a complete pipeline for image stitching, including cropping, alignment, light correction, stitching of images and blending.

Note: The .lp file must have the total number of light positions written as an interger in the first line. Like: 23, 24 etc
      During testing, run the steps 1-3 only once to save processing time.
"""

from crop_images import crop_images
from Subpixel_Alignment import align_folders
from homomorphic_lightcorrection import HomomorphicFilter
from lam_inv_sq_method import LamInvSqMethod
from stitching import stitch_images
from paths import (
    get_crop_paths,
    get_alignment_paths,
    get_light_correction_paths,
    get_stitching_paths,
)

from acq_data import Acq
from pathlib import Path
import shutil
import cv2


# ******************** important parameters ********************
use_light_correction = True  # Set True to apply light correction
use_multiband = True          # Set True to use multiband blending for stitching
multiband_levels = 3          # Number of levels for multiband blending
# ******************** important parameters ********************

# Detecting acquisition folders - use small dataset for testing
def get_acquisition_labels():
    raw_data_dir = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\1-raw_data")
    # raw_data_dir = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\5-small_dataset\1-raw_data") # use small dataset for testing
    return sorted([f.name for f in raw_data_dir.iterdir() if f.is_dir()])

acq_labels = get_acquisition_labels()
print("Detected acquisition labels:", acq_labels)


# Copying .lp file to cropped and aligned folders
def copy_lp_files(input_folder, cropped_folder, aligned_folder):
    lp_files = list(Path(input_folder).glob("*.lp"))
    for lp_file in lp_files:
        shutil.copy(lp_file, cropped_folder / lp_file.name)
        shutil.copy(lp_file, aligned_folder / lp_file.name)

def main():
    # print("STEP 1: CROP RAW ACQUISITIONS")
    # for label in acq_labels:
    #     print(f"Cropping images for {label}...")
    #     input_folder, cropped_folder = get_crop_paths([label])[0]
    #     crop_images(input_folder, cropped_folder, 0, 0, 0, 0)


    # print("STEP 2: IMAGE ALIGNMENT")
    # for label in acq_labels:
    #     input_folder, aligned_folder = get_alignment_paths([label])[0]
    #     align_folders(label)
    #     copy_lp_files(
    #         input_folder.parent.parent / f"1-raw_data/{label}",
    #         input_folder,
    #         aligned_folder,
    #     )


    # print("STEP 3: COPY LIGHT PROFILE FILES")
    # for label in acq_labels:
    #     input_folder, aligned_folder = get_alignment_paths([label])[0]
    #     copy_lp_files(
    #         input_folder.parent.parent / f"1-raw_data/{label}",
    #         input_folder,
    #         aligned_folder,
    #     )


    # print("STEP 4: LIGHT CORRECTION using Homomorphic Filter")
    # cropped_inputs, light_outputs = zip(*get_light_correction_paths(acq_labels))
    # for in_path, out_path in zip(cropped_inputs, light_outputs):
    #     acq = Acq(in_path)
    #     HomomorphicFilter(acq, out_path)

    # print("STEP 4: LIGHT CORRECTION using LamInvSqMethod")
    # cropped_inputs, light_outputs = zip(*get_light_correction_paths(acq_labels))
    # for in_path, out_path in zip(cropped_inputs, light_outputs):
    #     acq = Acq(in_path)
    #     LamInvSqMethod(acq, out_path)  

    # print("STEP 5: STITCHING")
    # cv2.ocl.setUseOpenCL(False)
    # acq_folders, base_output_folder = get_stitching_paths(acq_labels, use_light_correction)
    # print(len(acq_folders))
    # print(acq_folders)

    # blend_folder_name = "multiband_blending" if use_multiband else "without_blending"

    # step_outputs = [
    #     base_output_folder / blend_folder_name / "A1_2",
    #     base_output_folder / blend_folder_name / "A3_4",
    #     base_output_folder / blend_folder_name / "A12_34",
    #     base_output_folder / blend_folder_name / "B1_2",
    #     base_output_folder / blend_folder_name / "B3_4",
    #     base_output_folder / blend_folder_name / "B12_34",
    #     base_output_folder / blend_folder_name / "A1234_B1234",
    #     base_output_folder / blend_folder_name / "matches_folder",
    #     base_output_folder / blend_folder_name / "overlap_folder",
    #     base_output_folder / blend_folder_name / "stitched_cropped",
    # ]
    # for folder in step_outputs:
    #     folder.mkdir(parents=True, exist_ok=True)

    # print("Step 1: Stitch A1 + A2")
    # stitch_images(acq_folders[0], acq_folders[1], step_outputs[0], step_outputs[7], step_outputs[8], use_multiband, multiband_levels)

    # print("Step 2: Stitch A3 + A4")
    # stitch_images(acq_folders[2], acq_folders[3], step_outputs[1], step_outputs[7], step_outputs[8], use_multiband, multiband_levels)

    # print("Step 3: Stitch A12 + A34")
    # stitch_images(step_outputs[0], step_outputs[1], step_outputs[2], step_outputs[7], step_outputs[8], use_multiband, multiband_levels)

    # print("Step 4: Stitch B1 + B2")
    # stitch_images(acq_folders[4], acq_folders[5], step_outputs[3], step_outputs[7], step_outputs[8], use_multiband, multiband_levels)

    # print("Step 5: Stitch B3 + B4")
    # stitch_images(acq_folders[6], acq_folders[7], step_outputs[4], step_outputs[7], step_outputs[8], use_multiband, multiband_levels)

    # print("Step 6: Stitch B12 + B34")
    # stitch_images(step_outputs[3], step_outputs[4], step_outputs[5], step_outputs[7], step_outputs[8], use_multiband, multiband_levels)

    # print("Step 7: Merge A1234 with B1234")
    # stitch_images(step_outputs[2], step_outputs[5], step_outputs[6], step_outputs[7], step_outputs[8], use_multiband, multiband_levels)


    # print("STEP 6: CROP FINAL STITCHED IMAGES")
    # crop_images(step_outputs[6], step_outputs[9], 90, 135, 190, 70) # top, bottom, left, right


    print("STEP 4: LIGHT CORRECTION using LamInvSqMethod")
    cropped_inputs, light_outputs = zip(*get_light_correction_paths(selected_labels=["multiband"])) # can choose "multiband" or "without_blending"
    for in_path, out_path in zip(cropped_inputs, light_outputs):
        acq = Acq(in_path, resize_factor=0.6) 
        LamInvSqMethod(acq, out_path)


    # aa = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\A12_34")
    # bb = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\B12_34")
    # cc = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\A1234_B1234")
    # dd = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\matches_folder")
    # ee = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\overlap_folder")
    # ff = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\3-stitched_output\nonlightcorrected\without_blending\stitched_cropped")

    # print("Step 7: Merge A1234 with B1234")
    # stitch_images(aa, bb, cc, dd, ee, use_multiband, multiband_levels)


    # print("STEP 6: CROP FINAL STITCHED IMAGES")
    # crop_images(cc, ff, 90, 135, 190, 70) # top, bottom, left, right

if __name__ == "__main__":
    main()
