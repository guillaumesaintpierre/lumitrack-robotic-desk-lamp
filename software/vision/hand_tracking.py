import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np


# ============================================================
# Configuration
# ============================================================

MODEL_PATH = Path("models/hand_landmarker.task")

CAMERA_INDEX = 0

WINDOW_NAME = "LumiTrack — Hand Tracking"

CAMERA_WARMUP_S = 1.5
MAX_CAMERA_INIT_ATTEMPTS = 30


# MediaPipe hand skeleton
HAND_CONNECTIONS = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),

    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),

    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),

    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),

    (13, 17),
    (17, 18),
    (18, 19),
    (19, 20),

    (0, 17),
]


# ============================================================
# Utility functions
# ============================================================

def normalized_to_pixel(
    x_norm: float,
    y_norm: float,
    width: int,
    height: int,
) -> tuple[int, int]:
    """
    Convert MediaPipe normalized coordinates in [0, 1]
    to image pixel coordinates.
    """

    x_px = int(
        np.clip(
            x_norm * width,
            0,
            width - 1,
        )
    )

    y_px = int(
        np.clip(
            y_norm * height,
            0,
            height - 1,
        )
    )

    return x_px, y_px


def palm_center(
    landmarks,
) -> tuple[float, float]:
    """
    Estimate a stable palm centre using:
    - wrist
    - index MCP
    - middle MCP
    - ring MCP
    - pinky MCP
    """

    indices = [
        0,
        5,
        9,
        13,
        17,
    ]

    x = float(
        np.mean(
            [
                landmarks[index].x
                for index in indices
            ]
        )
    )

    y = float(
        np.mean(
            [
                landmarks[index].y
                for index in indices
            ]
        )
    )

    return x, y


def draw_hand(
    frame,
    landmarks,
) -> tuple[float, float]:
    """
    Draw landmarks, skeleton, and palm centre.

    Returns:
        palm_x_norm
        palm_y_norm
    """

    height, width = frame.shape[:2]

    points = []

    for landmark in landmarks:
        point = normalized_to_pixel(
            landmark.x,
            landmark.y,
            width,
            height,
        )

        points.append(point)

    # Draw hand skeleton
    for start, end in HAND_CONNECTIONS:
        cv2.line(
            frame,
            points[start],
            points[end],
            (255, 255, 255),
            2,
        )

    # Draw landmarks
    for point in points:
        cv2.circle(
            frame,
            point,
            4,
            (255, 255, 255),
            -1,
        )

    # Palm centre
    palm_x_norm, palm_y_norm = palm_center(
        landmarks
    )

    palm_x, palm_y = normalized_to_pixel(
        palm_x_norm,
        palm_y_norm,
        width,
        height,
    )

    cv2.circle(
        frame,
        (palm_x, palm_y),
        12,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Palm: ({palm_x}, {palm_y})",
        (
            palm_x + 15,
            palm_y - 15,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    return palm_x_norm, palm_y_norm


# ============================================================
# Camera
# ============================================================

def open_camera():
    """
    Open the FaceTime camera through AVFoundation
    and verify that frames can actually be read.
    """

    print("Opening camera...")

    camera = cv2.VideoCapture(
        CAMERA_INDEX,
        cv2.CAP_AVFOUNDATION,
    )

    if not camera.isOpened():
        raise RuntimeError(
            "Could not open webcam."
        )

    time.sleep(
        CAMERA_WARMUP_S
    )

    frame_ok = False
    frame_shape = None

    for _ in range(
        MAX_CAMERA_INIT_ATTEMPTS
    ):
        success, frame = camera.read()

        if success:
            frame_ok = True
            frame_shape = frame.shape
            break

        time.sleep(0.1)

    if not frame_ok:
        camera.release()

        raise RuntimeError(
            "Webcam opened, but no frame could be read."
        )

    print(
        f"Camera ready — frame shape: "
        f"{frame_shape}"
    )

    return camera


# ============================================================
# MediaPipe setup
# ============================================================

def create_hand_landmarker():
    """
    Create the MediaPipe Hand Landmarker.

    CPU is explicitly selected to avoid GPU / Metal issues
    observed on some macOS Apple Silicon configurations.
    """

    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"MediaPipe model not found: "
            f"{MODEL_PATH}"
        )

    BaseOptions = mp.tasks.BaseOptions

    HandLandmarker = (
        mp.tasks.vision.HandLandmarker
    )

    HandLandmarkerOptions = (
        mp.tasks.vision.HandLandmarkerOptions
    )

    RunningMode = (
        mp.tasks.vision.RunningMode
    )

    options = HandLandmarkerOptions(
        base_options=BaseOptions(
            model_asset_path=str(
                MODEL_PATH
            ),
            delegate=BaseOptions.Delegate.CPU,
        ),
        running_mode=RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    return HandLandmarker.create_from_options(
        options
    )


# ============================================================
# Main loop
# ============================================================

def main():

    camera = open_camera()

    start_time = time.perf_counter()

    try:
        with create_hand_landmarker() as landmarker:

            while True:

                success, frame = camera.read()

                if not success:
                    print(
                        "Warning: frame could not be read."
                    )

                    time.sleep(0.05)
                    continue

                # Mirror image for intuitive interaction
                frame = cv2.flip(
                    frame,
                    1,
                )

                # OpenCV BGR -> RGB
                rgb_frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGR2RGB,
                )

                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=rgb_frame,
                )

                timestamp_ms = int(
                    (
                        time.perf_counter()
                        - start_time
                    )
                    * 1000
                )

                result = (
                    landmarker.detect_for_video(
                        mp_image,
                        timestamp_ms,
                    )
                )

                # ------------------------------------------------
                # Hand detected
                # ------------------------------------------------

                if result.hand_landmarks:

                    landmarks = (
                        result.hand_landmarks[0]
                    )

                    palm_x, palm_y = draw_hand(
                        frame,
                        landmarks,
                    )

                    handedness = (
                        result.handedness[0][0]
                        .category_name
                    )

                    cv2.putText(
                        frame,
                        f"Hand: {handedness}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 255, 255),
                        2,
                    )

                    cv2.putText(
                        frame,
                        (
                            "Normalized palm: "
                            f"({palm_x:.3f}, "
                            f"{palm_y:.3f})"
                        ),
                        (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )

                # ------------------------------------------------
                # No hand detected
                # ------------------------------------------------

                else:

                    cv2.putText(
                        frame,
                        "No hand detected",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 255, 255),
                        2,
                    )

                # ------------------------------------------------
                # UI
                # ------------------------------------------------

                cv2.putText(
                    frame,
                    "Press Q to quit",
                    (
                        20,
                        frame.shape[0] - 20,
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                )

                cv2.imshow(
                    WINDOW_NAME,
                    frame,
                )

                key = (
                    cv2.waitKey(1)
                    & 0xFF
                )

                if key == ord("q"):
                    break

    finally:

        camera.release()
        cv2.destroyAllWindows()


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":
    main()

    