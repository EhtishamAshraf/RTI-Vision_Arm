"""" Image Stitching Module
This module implements image stitching using SIFT feature matching and optional multiband blending."""

import cv2
import numpy as np
import os
from multiband_blending import multiband_blending


def stitch_images(acqA_folder, acqB_folder, output_folder, matches_folder, overlap_folder, use_multiband, levels):
    output_folder.mkdir(parents=True, exist_ok=True)
    sift = cv2.SIFT_create()
    bf = cv2.BFMatcher()

    if any(f.lower().endswith(('.png', '.jpg', '.jpeg')) for f in os.listdir(acqA_folder)):
        resultsA_folder = acqA_folder
    elif (acqA_folder / "results").exists():
        resultsA_folder = acqA_folder / "results"
    else:
        raise FileNotFoundError(f"No images found in {acqA_folder} or {acqA_folder/'results'}")

    if any(f.lower().endswith(('.png', '.jpg', '.jpeg')) for f in os.listdir(acqB_folder)):
        resultsB_folder = acqB_folder
    elif (acqB_folder / "results").exists():
        resultsB_folder = acqB_folder / "results"
    else:
        raise FileNotFoundError(f"No images found in {acqB_folder} or {acqB_folder/'results'}")

    for filename in sorted(os.listdir(resultsA_folder)):

        print(f"Processing {filename}...")
        if not (filename.endswith(".png") or filename.endswith(".JPG") or filename.endswith(".JPEG")):
            print(f"Skipping {filename}, not a PNG, or JPG, or JPEG file.")
            continue

        imgA = cv2.imread(str(resultsA_folder / filename))
        imgB = cv2.imread(str(resultsB_folder / filename))
        print(f"Loaded images: {imgA.shape if imgA is not None else 'None'}, {imgB.shape if imgB is not None else 'None'}")

        if imgA is None or imgB is None:
            print(f"Skipping {filename}")
            continue

        kp1, des1 = sift.detectAndCompute(imgA, None)
        kp2, des2 = sift.detectAndCompute(imgB, None)

        matches = bf.knnMatch(des1, des2, k=2)
        good = [m for m, n in matches if m.distance < 0.75 * n.distance]

        if len(good) < 10:
            print(f"Not enough matches in {filename}")
            continue

        # Saving SIFT Matches Visualization
        match_vis = cv2.drawMatches(imgA, kp1, imgB, kp2, good, None, flags=2)
        match_filename = filename.replace("aligned", "matches")
        cv2.imwrite(str(matches_folder / match_filename), match_vis)

        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        H, _ = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)

        hA, wA = imgA.shape[:2]
        hB, wB = imgB.shape[:2]
        corners_B = np.float32([[0, 0], [0, hB], [wB, hB], [wB, 0]]).reshape(-1, 1, 2)
        warped_corners_B = cv2.perspectiveTransform(corners_B, H)
        all_corners = np.concatenate((np.float32([[0, 0], [0, hA], [wA, hA], [wA, 0]]).reshape(-1, 1, 2), warped_corners_B), axis=0)

        [x_min, y_min] = np.int32(all_corners.min(axis=0).ravel() - 0.5)
        [x_max, y_max] = np.int32(all_corners.max(axis=0).ravel() + 0.5)

        dx, dy = -x_min, -y_min
        translation = np.array([[1, 0, dx], [0, 1, dy], [0, 0, 1]])
        H_translated = translation @ H
        canvas_size = (x_max - x_min, y_max - y_min)

        warped_B = cv2.warpPerspective(imgB, H_translated, canvas_size)
        canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
        canvas[dy:dy + hA, dx:dx + wA] = imgA

        if use_multiband:
            canvas = multiband_blending(imgA, imgB, H, levels=levels, filename=filename, overlap_folder=overlap_folder)
            
        else:
            mask_B = (warped_B > 0)
            canvas = np.where(mask_B, warped_B, canvas)
            print(f"Using simple blending for {filename}")

            # Calling multiband blending only for overlap preview
            if filename and overlap_folder is not None:
                 _ = multiband_blending(imgA, imgB, H, levels=levels, filename=f"simple_{filename}", overlap_folder=overlap_folder)

        out_name = filename
        cv2.imwrite(str(output_folder / out_name), canvas)
        print(f"Done: {out_name}")
