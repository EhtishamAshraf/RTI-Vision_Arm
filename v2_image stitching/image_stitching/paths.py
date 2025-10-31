"""Module to define and manage project paths for image stitching pipeline."""

from pathlib import Path

BASE_PATH = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data")
# BASE_PATH = Path(r"C:\Users\ad\Desktop\v2_image_stitching_&_relighting\data\5-small_dataset") # use small dataset for testing 


def get_crop_paths(acq_labels):
    """Returns input and output folders for raw to cropped conversion."""
    paths = []
    for label in acq_labels:
        input_folder = BASE_PATH / f"1-raw_data/{label}"
        output_folder = BASE_PATH / f"2-processed_data/1-{label}_cropped"
        output_folder.mkdir(parents=True, exist_ok=True)
        paths.append((input_folder, output_folder))
    return paths


def get_alignment_paths(acq_labels):
    """Returns input and output folders for cropped to aligned conversion."""
    paths = []
    for label in acq_labels:
        input_folder = BASE_PATH / f"2-processed_data/1-{label}_cropped"
        output_folder = BASE_PATH / f"2-processed_data/2-{label}_aligned"
        output_folder.mkdir(parents=True, exist_ok=True)
        paths.append((input_folder, output_folder))
    return paths


# def get_light_correction_paths(acq_labels):
#     """Returns input and output folders for alignment to light-corrected conversion."""
#     paths = []
#     for label in acq_labels:
#         input_folder = BASE_PATH / f"2-processed_data/1-{label}_cropped"
#         output_folder = BASE_PATH / f"2-processed_data/3-{label}_lightcorrected"
#         output_folder.mkdir(parents=True, exist_ok=True)
#         paths.append((input_folder, output_folder))
#     return paths

def get_light_correction_paths(selected_labels=None):
    all_inputs = {
        "multiband": BASE_PATH / r"3-stitched_output\nonlightcorrected\multiband_blending\stitched_cropped",
        "without_blending": BASE_PATH / r"3-stitched_output\nonlightcorrected\without_blending\stitched_cropped",
    }

    if selected_labels is None:
        selected_labels = all_inputs.keys()

    # Main output folder
    main_output_folder = BASE_PATH / r"3-stitched_output\lightcorrected"
    main_output_folder.mkdir(parents=True, exist_ok=True)

    paths = []
    for label in selected_labels:
        if label not in all_inputs:
            print(f"Warning: Unknown label '{label}' skipped.")
            continue

        input_folder = all_inputs[label]
        if not input_folder.exists():
            print(f"Warning: Input folder does not exist: {input_folder}")
            continue

        # Output folder for this input
        output_folder = main_output_folder / label
        output_folder.mkdir(parents=True, exist_ok=True)

        paths.append((input_folder, output_folder))

    return paths

def get_stitching_paths(acq_labels, use_light_correction):
    """Returns list of input folders for stitching and output/match/overlap folders."""
    acq_folders = []
    type_folder = "lightcorrected" if use_light_correction else "nonlightcorrected"

    for label in acq_labels:
        folder = BASE_PATH / f"2-processed_data/{'3' if use_light_correction else '1'}-{label}_{'lightcorrected' if use_light_correction else 'cropped'}"
        acq_folders.append(folder)

    output_folder = BASE_PATH / f"3-stitched_output/{type_folder}"
    output_folder.mkdir(parents=True, exist_ok=True)

    return acq_folders, output_folder
