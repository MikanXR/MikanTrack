"""Calibration patterns sized for a phone screen in landscape, and the geometry the probe detects them with.

    phone_patterns.py <out_dir>

Writes one PNG per pattern. A `_dim` image draws its white at a lower level, since a bright screen blooms in
the camera image.
"""

import argparse
import os

import cv2
import numpy as np

CHARUCO_SQUARES = (7, 4)
CHARUCO_MARKER_RATIO = 0.75
CHARUCO_DICTIONARIES = {"6x6": cv2.aruco.DICT_6X6_250, "4x4": cv2.aruco.DICT_4X4_250}

CHESSBOARD_SQUARES = (10, 5)
CHESSBOARD_INNER_CORNERS = (CHESSBOARD_SQUARES[0] - 1, CHESSBOARD_SQUARES[1] - 1)

# Asymmetric circle grid in OpenCV's layout: points per row, then rows, each odd row offset by half a step
CIRCLE_GRID = (4, 13)
CIRCLE_RADIUS_RATIO = 0.35

DIM_WHITE = 150


def charuco_board(dictionary_name):
    dictionary = cv2.aruco.getPredefinedDictionary(CHARUCO_DICTIONARIES[dictionary_name])
    return cv2.aruco.CharucoBoard(CHARUCO_SQUARES, 1.0, CHARUCO_MARKER_RATIO, dictionary)


def charuco_image(dictionary_name, square_px=320):
    size = (CHARUCO_SQUARES[0] * square_px, CHARUCO_SQUARES[1] * square_px)
    return charuco_board(dictionary_name).generateImage(size, marginSize=10, borderBits=1)


def chessboard_image(square_px=220):
    cols, rows = CHESSBOARD_SQUARES
    margin = square_px // 2
    image = np.full((rows * square_px + 2 * margin, cols * square_px + 2 * margin), 255, np.uint8)
    for row in range(rows):
        for col in range(cols):
            if (row + col) % 2 == 0:
                top, left = margin + row * square_px, margin + col * square_px
                image[top:top + square_px, left:left + square_px] = 0
    return image


def circle_grid_image(step_px=180):
    per_row, rows = CIRCLE_GRID
    margin = step_px
    width = (2 * (per_row - 1) + 1) * step_px + 2 * margin
    height = (rows - 1) * step_px + 2 * margin
    image = np.zeros((height, width), np.uint8)
    radius = int(round(CIRCLE_RADIUS_RATIO * step_px))
    for row in range(rows):
        for col in range(per_row):
            center = (margin + (2 * col + row % 2) * step_px, margin + row * step_px)
            cv2.circle(image, center, radius, 255, -1, cv2.LINE_AA)
    # Drawn with its rows down the page, turned so the long side lies along a landscape screen
    return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)


def dimmed(image):
    return (image.astype(np.float32) * (DIM_WHITE / 255.0)).astype(np.uint8)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out_dir")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    images = {
        "charuco_6x6": charuco_image("6x6"),
        "charuco_6x6_dim": dimmed(charuco_image("6x6")),
        "charuco_4x4": charuco_image("4x4"),
        "chessboard": chessboard_image(),
        "chessboard_dim": dimmed(chessboard_image()),
        "circles": circle_grid_image(),
    }
    for name, image in images.items():
        path = os.path.join(args.out_dir, name + ".png")
        cv2.imwrite(path, image)
        print(f"{path}  {image.shape[1]}x{image.shape[0]}")


if __name__ == "__main__":
    main()
