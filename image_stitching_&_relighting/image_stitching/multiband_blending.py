"""
Multiband blending for image stitching using OpenCV.
This module provides functions to perform multiband blending of two images using a homography matrix.
"""

import cv2
import numpy as np

def create_feather_mask(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    mask = mask.astype(np.float32) / 255.0

    dist = cv2.distanceTransform((mask * 255).astype(np.uint8), cv2.DIST_L2, 5)
    dist = np.clip(dist / dist.max(), 0, 1)

    feather_mask = cv2.merge([dist, dist, dist])
    return feather_mask

def multiband_blending(img_a, img_b, h_mat, levels=6, filename=None, overlap_folder=None):
    img_a = img_a.astype(np.float32)
    img_b = img_b.astype(np.float32)

    h_a, w_a = img_a.shape[:2]
    h_b, w_b = img_b.shape[:2]

    corners_a = np.array([[0, 0], [0, h_a], [w_a, h_a], [w_a, 0]], dtype=np.float32).reshape(-1, 1, 2)
    corners_b = np.array([[0, 0], [0, h_b], [w_b, h_b], [w_b, 0]], dtype=np.float32).reshape(-1, 1, 2)
    warped_corners_b = cv2.perspectiveTransform(corners_b, h_mat)
    all_corners = np.vstack((corners_a, warped_corners_b))

    xmin, ymin = np.int32(all_corners.min(axis=0).ravel() - 0.5)
    xmax, ymax = np.int32(all_corners.max(axis=0).ravel() + 0.5)
    width, height = xmax - xmin, ymax - ymin
    offset = (-xmin, -ymin)

    h_translation = np.array([[1, 0, offset[0]],
                              [0, 1, offset[1]],
                              [0, 0, 1]], dtype=np.float32)

    warped_img_a = cv2.warpPerspective(img_a, h_translation, (width, height))
    warped_img_b = cv2.warpPerspective(img_b, h_translation @ h_mat, (width, height))

    warped_img_a_8u = cv2.convertScaleAbs(warped_img_a)
    warped_img_b_8u = cv2.convertScaleAbs(warped_img_b)

    mask_a = create_feather_mask(warped_img_a_8u)
    mask_b = create_feather_mask(warped_img_b_8u)

    mask_a_uint8 = (mask_a[:, :, 0] * 255).astype(np.uint8)
    mask_b_uint8 = (mask_b[:, :, 0] * 255).astype(np.uint8)

    # Adding a Bounding Box on the overlap region
    if filename:
        overlap_mask = cv2.bitwise_and(mask_a_uint8, mask_b_uint8)
        x, y, w, h = cv2.boundingRect(overlap_mask)
        preview = cv2.addWeighted(warped_img_a_8u, 0.5, warped_img_b_8u, 0.5, 0)

        if w > 0 and h > 0:
            cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 0, 255), 2)  # red box
        cv2.imwrite(str(overlap_folder / f"overlap_{filename}"), preview)

    # Multi-band Blender
    blender = cv2.detail_MultiBandBlender(try_gpu=0)
    blender.setNumBands(levels)
    blender.prepare((xmin, ymin, width, height))
    blender.feed(cv2.UMat(warped_img_a_8u), cv2.UMat(mask_a_uint8), (xmin, ymin))
    blender.feed(cv2.UMat(warped_img_b_8u), cv2.UMat(mask_b_uint8), (xmin, ymin))

    result, _ = blender.blend(None, None)
    return result.get() if isinstance(result, cv2.UMat) else result
