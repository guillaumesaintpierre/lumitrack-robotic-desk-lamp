import time

import cv2

from software.calibration.desk_calibration import (
    image_to_robot,
    load_calibration,
)


CAMERA_INDEX = 0
CALIBRATION_PATH = "config/desk_calibration.json"

WINDOW_NAME = "LumiTrack — Calibration Verification"


def main():

    calibration = load_calibration(
        CALIBRATION_PATH
    )

    H = calibration["homography"]
    base_world = calibration["base_world"]

    camera = cv2.VideoCapture(
        CAMERA_INDEX,
        cv2.CAP_AVFOUNDATION,
    )

    if not camera.isOpened():
        raise RuntimeError(
            "Could not open camera."
        )

    time.sleep(1.5)

    print()
    print(
        "Move the mouse over the desk."
    )
    print(
        "The displayed coordinates are relative to the lamp base."
    )
    print(
        "Press Q to quit."
    )
    print()

    mouse_position = [0, 0]

    def mouse_callback(
        event,
        x,
        y,
        flags,
        param,
    ):

        del event
        del flags
        del param

        mouse_position[0] = x
        mouse_position[1] = y

    cv2.namedWindow(
        WINDOW_NAME
    )

    cv2.setMouseCallback(
        WINDOW_NAME,
        mouse_callback,
    )

    try:

        while True:

            success, frame = camera.read()

            if not success:
                continue

            frame = cv2.flip(
                frame,
                1,
            )

            robot_point = image_to_robot(
                mouse_position,
                H,
                base_world,
            )

            x_m = robot_point[0]
            y_m = robot_point[1]

            cv2.circle(
                frame,
                tuple(mouse_position),
                8,
                (255, 255, 255),
                2,
            )

            text = (
                f"Robot x={x_m:.3f} m, "
                f"y={y_m:.3f} m"
            )

            cv2.putText(
                frame,
                text,
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
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


if __name__ == "__main__":
    main()
    