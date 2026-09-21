import time

import cv2
import numpy as np

from software.calibration.desk_calibration import (
    compute_homography,
    image_to_world,
    save_calibration,
)


CAMERA_INDEX = 0

OUTPUT_PATH = (
    "config/desk_calibration.json"
)

WINDOW_NAME = (
    "LumiTrack — Desk Calibration"
)


POINT_LABELS = [
    "P1 FAR-LEFT",
    "P2 FAR-RIGHT",
    "P3 NEAR-RIGHT",
    "P4 NEAR-LEFT",
    "LAMP BASE CENTER",
]


clicked_points = []


def mouse_callback(
    event,
    x,
    y,
    flags,
    param,
):
    """
    Record mouse clicks.
    """

    del flags
    del param

    if (
        event == cv2.EVENT_LBUTTONDOWN
        and len(clicked_points) < 5
    ):
        clicked_points.append(
            (x, y)
        )

        index = len(
            clicked_points
        ) - 1

        print(
            f"{POINT_LABELS[index]}: "
            f"({x}, {y})"
        )


def open_camera():

    print(
        "Opening camera..."
    )

    camera = cv2.VideoCapture(
        CAMERA_INDEX,
        cv2.CAP_AVFOUNDATION,
    )

    if not camera.isOpened():
        raise RuntimeError(
            "Could not open camera."
        )

    time.sleep(1.5)

    for _ in range(30):

        success, frame = camera.read()

        if success:
            print(
                "Camera ready."
            )

            return camera

        time.sleep(0.1)

    camera.release()

    raise RuntimeError(
        "Camera opened but frames could not be read."
    )


def get_positive_float(
    prompt: str,
) -> float:

    while True:

        try:
            value = float(
                input(prompt)
            )

            if value <= 0:
                raise ValueError

            return value

        except ValueError:

            print(
                "Please enter a positive number."
            )


def draw_points(
    frame,
):

    for index, point in enumerate(
        clicked_points
    ):

        cv2.circle(
            frame,
            point,
            8,
            (255, 255, 255),
            -1,
        )

        cv2.putText(
            frame,
            str(index + 1),
            (
                point[0] + 10,
                point[1] - 10,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )


def main():

    print()
    print(
        "=== LumiTrack Desk Calibration ==="
    )
    print()

    workspace_width_m = (
        get_positive_float(
            "Calibration rectangle WIDTH [m]: "
        )
    )

    workspace_depth_m = (
        get_positive_float(
            "Calibration rectangle DEPTH [m]: "
        )
    )

    print()
    print(
        "Place four markers on the desk."
    )

    print(
        "Physical click order:"
    )

    print(
        "1 — FAR-LEFT"
    )

    print(
        "2 — FAR-RIGHT"
    )

    print(
        "3 — NEAR-RIGHT"
    )

    print(
        "4 — NEAR-LEFT"
    )

    print(
        "5 — CENTER OF LAMP BASE AXIS"
    )

    print()
    print(
        "Press SPACE to freeze the image."
    )

    camera = open_camera()

    frozen_frame = None

    try:

        # --------------------------------------------------
        # Live preview
        # --------------------------------------------------

        while frozen_frame is None:

            success, frame = camera.read()

            if not success:
                continue

            # Must match the hand-tracking pipeline
            frame = cv2.flip(
                frame,
                1,
            )

            preview = frame.copy()

            cv2.putText(
                preview,
                "Press SPACE to freeze calibration image",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
            )

            cv2.imshow(
                WINDOW_NAME,
                preview,
            )

            key = (
                cv2.waitKey(1)
                & 0xFF
            )

            if key == 32:
                frozen_frame = (
                    frame.copy()
                )

            elif key == ord("q"):
                return

        # --------------------------------------------------
        # Point selection
        # --------------------------------------------------

        cv2.setMouseCallback(
            WINDOW_NAME,
            mouse_callback,
        )

        while True:

            display = (
                frozen_frame.copy()
            )

            draw_points(
                display
            )

            next_index = len(
                clicked_points
            )

            if next_index < 5:

                instruction = (
                    "Click: "
                    + POINT_LABELS[
                        next_index
                    ]
                )

            else:

                instruction = (
                    "Press S to save, "
                    "R to restart"
                )

            cv2.putText(
                display,
                instruction,
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )

            cv2.imshow(
                WINDOW_NAME,
                display,
            )

            key = (
                cv2.waitKey(20)
                & 0xFF
            )

            if key == ord("r"):

                clicked_points.clear()

            elif (
                key == ord("s")
                and len(
                    clicked_points
                ) == 5
            ):
                break

            elif key == ord("q"):
                return

        # --------------------------------------------------
        # Homography
        # --------------------------------------------------

        image_corners = np.asarray(
            clicked_points[:4],
            dtype=np.float32,
        )

        # Robot convention:
        #
        # x = forward, from FAR edge toward user
        # y = lateral
        #
        # y = 0 is the centre line of the
        # calibration rectangle.

        half_width = (
            workspace_width_m
            / 2.0
        )

        world_corners = np.array(
            [
                [
                    0.0,
                    -half_width,
                ],
                [
                    0.0,
                    half_width,
                ],
                [
                    workspace_depth_m,
                    half_width,
                ],
                [
                    workspace_depth_m,
                    -half_width,
                ],
            ],
            dtype=np.float32,
        )

        homography = compute_homography(
            image_corners,
            world_corners,
        )

        base_pixel = (
            clicked_points[4]
        )

        base_world = image_to_world(
            base_pixel,
            homography,
        )

        save_calibration(
            path=OUTPUT_PATH,
            homography=homography,
            base_world=base_world,
            image_points=image_corners,
            workspace_width_m=workspace_width_m,
            workspace_depth_m=workspace_depth_m,
        )

        print()
        print(
            "=== Calibration saved ==="
        )

        print(
            f"File: {OUTPUT_PATH}"
        )

        print()

        print(
            "Lamp base position "
            "in desk coordinates:"
        )

        print(
            f"x = {base_world[0]:.4f} m"
        )

        print(
            f"y = {base_world[1]:.4f} m"
        )

        print()

        print(
            "Homography:"
        )

        print(
            homography
        )

    finally:

        camera.release()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

    