#!/usr/bin/env python3
"""Generate an ArUco marker image to use as a texture in Gazebo."""

import cv2
import numpy as np
import os
import sys

# Use the original ArUco dictionary (5x5, up to 1024 markers)
ARUCO_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL
MARKER_ID = 0
MARKER_SIZE = 500  # pixels
BORDER_SIZE = 50   # white border in pixels

def generate_marker(output_path: str):
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    marker_img = np.zeros((MARKER_SIZE, MARKER_SIZE), dtype=np.uint8)
    marker_img = cv2.aruco.generateImageMarker(
        dictionary, MARKER_ID, MARKER_SIZE, marker_img, 1
    )

    # Add white border around the marker
    bordered = np.ones(
        (MARKER_SIZE + 2 * BORDER_SIZE, MARKER_SIZE + 2 * BORDER_SIZE),
        dtype=np.uint8
    ) * 255
    bordered[
        BORDER_SIZE:BORDER_SIZE + MARKER_SIZE,
        BORDER_SIZE:BORDER_SIZE + MARKER_SIZE
    ] = marker_img

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, bordered)
    print(f"ArUco marker (DICT_ARUCO_ORIGINAL, ID={MARKER_ID}) saved to: {output_path}")

if __name__ == '__main__':
    output = sys.argv[1] if len(sys.argv) > 1 else 'aruco_marker.png'
    generate_marker(output)
