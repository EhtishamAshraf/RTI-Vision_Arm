"""Module to define and manage project paths for image stitching pipeline."""

from pathlib import Path

def get_crop_paths(acq_label: str):
    base = Path(r"C:/Users/ad/Downloads/Thesis/data/retrato de lola flores/data/image_stitching_&_relighting/data")

    if acq_label == "AcqA":
        input_folder = base / "1-raw_data/AcqA"
        output_folder = base / "2-processed_data/1-AcqA_cropped"
    elif acq_label == "AcqB":
        input_folder = base / "1-raw_data/AcqB"
        output_folder = base / "2-processed_data/1-AcqB_cropped"
    else:
        raise ValueError("acq_label must be 'AcqA' or 'AcqB'")

    output_folder.mkdir(parents=True, exist_ok=True)
    return input_folder, output_folder


def get_alignment_paths(acq_label: str):
    base = Path(r"C:/Users/ad/Downloads/Thesis/data/retrato de lola flores/data/image_stitching_&_relighting/data")

    if acq_label == "AcqA":
        input_folder = base / "2-processed_data/1-AcqA_cropped"
        output_folder = base / "2-processed_data/2-aligned_AcqA"
    elif acq_label == "AcqB":
        input_folder = base / "2-processed_data/1-AcqB_cropped"
        output_folder = base / "2-processed_data/2-aligned_AcqB"
    else:
        raise ValueError("acq_label must be 'AcqA' or 'AcqB'")

    output_folder.mkdir(parents=True, exist_ok=True)
    return input_folder, output_folder


def get_light_correction_paths():
    base = Path(r"C:/Users/ad/Downloads/Thesis/data/retrato de lola flores/data/image_stitching_&_relighting/data")

    acqA_input = base / "2-processed_data/2-aligned_AcqA"
    acqB_input = base / "2-processed_data/2-aligned_AcqB"
    acqA_output = base / "2-processed_data/3-AcqA_lightcorrected"
    acqB_output = base / "2-processed_data/3-AcqB_lightcorrected"

    # Create output directories
    acqA_output.mkdir(parents=True, exist_ok=True)
    acqB_output.mkdir(parents=True, exist_ok=True)

    return acqA_input, acqB_input, acqA_output, acqB_output


def get_stitching_paths(use_light_correction=True):
    base = Path(r"C:/Users/ad/Downloads/Thesis/data/retrato de lola flores/data/image_stitching_&_relighting/data")

    if use_light_correction:
        acqA_folder = base / "2-processed_data/3-AcqA_lightcorrected"
        acqB_folder = base / "2-processed_data/3-AcqB_lightcorrected"
        output_folder = base / "3-stitched_output/lightcorrected"
    else:
        acqA_folder = base / "2-processed_data/2-aligned_AcqA"
        acqB_folder = base / "2-processed_data/2-aligned_AcqB"
        output_folder = base / "3-stitched_output/nonlightcorrected"

    matches_folder = output_folder / "matches"
    overlap_folder = output_folder / "overlap"

    # Create output directories
    output_folder.mkdir(parents=True, exist_ok=True)
    matches_folder.mkdir(parents=True, exist_ok=True)
    overlap_folder.mkdir(parents=True, exist_ok=True)

    return acqA_folder, acqB_folder, output_folder, matches_folder, overlap_folder
