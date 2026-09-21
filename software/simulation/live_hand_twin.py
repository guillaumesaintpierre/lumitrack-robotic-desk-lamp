import time

import cv2
import matplotlib.pyplot as plt
import mediapipe as mp
import numpy as np

from software.calibration.desk_calibration import (
    image_to_robot,
    load_calibration,
)
from software.control.kinematics import (
    LampGeometry,
    UnreachableTargetError,
)
from software.control.motion_controller import (
    LumiTrackMotionController,
)
from software.simulation.digital_twin import (
    LumiTrackDigitalTwin,
)
from software.simulation.light_geometry import (
    desk_intersection,
    head_position,
    tracking_error_m,
)
from software.vision.hand_tracking import (
    create_hand_landmarker,
    draw_hand,
    normalized_to_pixel,
    open_camera,
)


# ============================================================
# Configuration
# ============================================================

CALIBRATION_PATH = "config/desk_calibration.json"

# Temporary digital-twin geometry.
# These will later be replaced by geometry extracted from
# the FORSÅ reference model.
GEOMETRY = LampGeometry(
    head_radius_m=0.080,
    head_height_m=0.257,
)


# ============================================================
# Utilities
# ============================================================

def build_robot_workspace(calibration):
    """
    Construct desk corners relative to the lamp base.
    """

    width = calibration["workspace_width_m"]
    depth = calibration["workspace_depth_m"]

    half_width = width / 2.0

    world_corners = np.array(
        [
            [0.0, -half_width],
            [0.0, half_width],
            [depth, half_width],
            [depth, -half_width],
        ],
        dtype=float,
    )

    base_world = calibration["base_world"]

    robot_corners = (
        world_corners
        - base_world
    )

    return robot_corners


def configure_3d_axes(
    ax,
    robot_corners,
):
    """
    Configure 3D visualization limits based on the
    calibrated workspace.
    """

    x_values = robot_corners[:, 0]
    y_values = robot_corners[:, 1]

    x_padding = 0.10
    y_padding = 0.10

    ax.set_xlim(
        np.min(x_values) - x_padding,
        np.max(x_values) + x_padding,
    )

    ax.set_ylim(
        np.min(y_values) - y_padding,
        np.max(y_values) + y_padding,
    )

    ax.set_zlim(
        0.0,
        0.55,
    )

    ax.set_xlabel(
        "x [m]"
    )

    ax.set_ylabel(
        "y [m]"
    )

    ax.set_zlabel(
        "z [m]"
    )

    ax.set_title(
        "LumiTrack Digital Twin"
    )

    ax.view_init(
        elev=28,
        azim=-65,
    )


def draw_workspace(
    ax,
    robot_corners,
):
    """
    Draw calibrated desk workspace.
    """

    closed = np.vstack(
        [
            robot_corners,
            robot_corners[0],
        ]
    )

    ax.plot(
        closed[:, 0],
        closed[:, 1],
        np.zeros(
            len(closed)
        ),
        linewidth=1.5,
    )


# ============================================================
# Main application
# ============================================================

def main():

    print()
    print(
        "=== LumiTrack Live Hand-Controlled Digital Twin ==="
    )
    print()

    # --------------------------------------------------------
    # Calibration
    # --------------------------------------------------------

    calibration = load_calibration(
        CALIBRATION_PATH
    )

    H = calibration[
        "homography"
    ]

    base_world = calibration[
        "base_world"
    ]

    robot_corners = (
        build_robot_workspace(
            calibration
        )
    )

    # --------------------------------------------------------
    # Robot models
    # --------------------------------------------------------

    kinematic_model = (
        LumiTrackDigitalTwin(
            geometry=GEOMETRY
        )
    )

    motion_controller = (
        LumiTrackMotionController()
    )

    # --------------------------------------------------------
    # Camera
    # --------------------------------------------------------

    camera = open_camera()

    # --------------------------------------------------------
    # MediaPipe
    # --------------------------------------------------------

    landmarker = (
        create_hand_landmarker()
    )

    # --------------------------------------------------------
    # Matplotlib interface
    # --------------------------------------------------------

    plt.ion()

    fig = plt.figure(
        figsize=(15, 7)
    )

    ax_camera = fig.add_subplot(
        1,
        2,
        1,
    )

    ax_3d = fig.add_subplot(
        1,
        2,
        2,
        projection="3d",
    )

    fig.suptitle(
        "LumiTrack — Real-Time Vision-Guided Digital Twin"
    )

    running = True

    def on_key(event):

        nonlocal running

        if event.key in (
            "q",
            "escape",
        ):
            running = False

    fig.canvas.mpl_connect(
        "key_press_event",
        on_key,
    )

    start_time = (
        time.perf_counter()
    )

    previous_time = (
        start_time
    )

    last_timestamp_ms = -1

    try:

        with landmarker:

            while (
                running
                and plt.fignum_exists(
                    fig.number
                )
            ):

                # --------------------------------------------
                # Time step
                # --------------------------------------------

                now = (
                    time.perf_counter()
                )

                dt = (
                    now
                    - previous_time
                )

                previous_time = now

                dt = np.clip(
                    dt,
                    0.005,
                    0.10,
                )

                # --------------------------------------------
                # Camera frame
                # --------------------------------------------

                success, frame = (
                    camera.read()
                )

                if not success:
                    continue

                frame = cv2.flip(
                    frame,
                    1,
                )

                height, width = (
                    frame.shape[:2]
                )

                rgb_frame = (
                    cv2.cvtColor(
                        frame,
                        cv2.COLOR_BGR2RGB,
                    )
                )

                mp_image = mp.Image(
                    image_format=(
                        mp.ImageFormat.SRGB
                    ),
                    data=rgb_frame,
                )

                timestamp_ms = int(
                    (
                        now
                        - start_time
                    )
                    * 1000
                )

                timestamp_ms = max(
                    timestamp_ms,
                    last_timestamp_ms + 1,
                )

                last_timestamp_ms = (
                    timestamp_ms
                )

                result = (
                    landmarker.detect_for_video(
                        mp_image,
                        timestamp_ms,
                    )
                )

                # --------------------------------------------
                # Current robot state
                # --------------------------------------------

                target_robot = None
                illuminated = None
                tracking_error = None
                desired = None

                # --------------------------------------------
                # Hand detected
                # --------------------------------------------

                if result.hand_landmarks:

                    landmarks = (
                        result.hand_landmarks[0]
                    )

                    palm_u_norm, palm_v_norm = (
                        draw_hand(
                            frame,
                            landmarks,
                        )
                    )

                    palm_px = (
                        normalized_to_pixel(
                            palm_u_norm,
                            palm_v_norm,
                            width,
                            height,
                        )
                    )

                    # Camera -> robot coordinates
                    robot_xy = (
                        image_to_robot(
                            palm_px,
                            H,
                            base_world,
                        )
                    )

                    target_robot = (
                        np.array(
                            [
                                robot_xy[0],
                                robot_xy[1],
                                0.0,
                            ],
                            dtype=float,
                        )
                    )

                    try:

                        desired = (
                            kinematic_model.command_target(
                                x_m=target_robot[0],
                                y_m=target_robot[1],
                                z_m=0.0,
                            )
                        )

                        state = (
                            motion_controller.update(
                                target_base_deg=(
                                    desired.base_yaw_deg
                                ),
                                target_head_deg=(
                                    desired.head_pitch_down_deg
                                ),
                                dt=float(dt),
                            )
                        )

                        illuminated = (
                            desk_intersection(
                                base_yaw_deg=(
                                    state.base_angle_deg
                                ),
                                head_pitch_down_deg=(
                                    state.head_angle_deg
                                ),
                                geometry=GEOMETRY,
                            )
                        )

                        tracking_error = (
                            tracking_error_m(
                                target_robot,
                                illuminated,
                            )
                        )

                        status = (
                            "TRACKING"
                        )

                    except UnreachableTargetError:

                        status = (
                            "TARGET TOO CLOSE"
                        )

                else:

                    status = (
                        "NO HAND"
                    )

                # --------------------------------------------
                # Current actual joint state
                # --------------------------------------------

                actual_base = (
                    motion_controller
                    .base
                    .state
                    .angle_deg
                )

                actual_head = (
                    motion_controller
                    .head
                    .state
                    .angle_deg
                )

                head = head_position(
                    actual_base,
                    GEOMETRY,
                )

                # --------------------------------------------
                # Camera overlay
                # --------------------------------------------

                cv2.putText(
                    frame,
                    f"Status: {status}",
                    (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                )

                if target_robot is not None:

                    cv2.putText(
                        frame,
                        (
                            "Robot target: "
                            f"x={target_robot[0]:.3f} m, "
                            f"y={target_robot[1]:.3f} m"
                        ),
                        (20, 150),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )

                if desired is not None:

                    cv2.putText(
                        frame,
                        (
                            f"Desired: "
                            f"base={desired.base_yaw_deg:.1f} deg, "
                            f"head={desired.head_pitch_down_deg:.1f} deg"
                        ),
                        (20, 190),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (255, 255, 255),
                        2,
                    )

                cv2.putText(
                    frame,
                    (
                        f"Actual: "
                        f"base={actual_base:.1f} deg, "
                        f"head={actual_head:.1f} deg"
                    ),
                    (20, 230),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2,
                )

                if tracking_error is not None:

                    cv2.putText(
                        frame,
                        (
                            "Tracking error: "
                            f"{tracking_error * 100:.2f} cm"
                        ),
                        (20, 270),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (255, 255, 255),
                        2,
                    )

                # --------------------------------------------
                # Camera panel
                # --------------------------------------------

                ax_camera.clear()

                ax_camera.imshow(
                    cv2.cvtColor(
                        frame,
                        cv2.COLOR_BGR2RGB,
                    )
                )

                ax_camera.set_title(
                    "Real Camera + MediaPipe"
                )

                ax_camera.axis(
                    "off"
                )

                # --------------------------------------------
                # Digital twin panel
                # --------------------------------------------

                ax_3d.clear()

                configure_3d_axes(
                    ax_3d,
                    robot_corners,
                )

                draw_workspace(
                    ax_3d,
                    robot_corners,
                )

                # Base axis
                ax_3d.plot(
                    [0.0, 0.0],
                    [0.0, 0.0],
                    [0.0, 0.12],
                    linewidth=5,
                )

                # Simplified lamp body
                ax_3d.plot(
                    [
                        0.0,
                        head[0],
                    ],
                    [
                        0.0,
                        head[1],
                    ],
                    [
                        0.0,
                        head[2],
                    ],
                    linewidth=4,
                )

                # Lamp head
                ax_3d.scatter(
                    head[0],
                    head[1],
                    head[2],
                    s=80,
                    label="Lamp head",
                )

                # Target
                if target_robot is not None:

                    ax_3d.scatter(
                        target_robot[0],
                        target_robot[1],
                        target_robot[2],
                        s=100,
                        marker="x",
                        label="Hand target",
                    )

                # Illuminated point
                if illuminated is not None:

                    ax_3d.scatter(
                        illuminated[0],
                        illuminated[1],
                        illuminated[2],
                        s=80,
                        marker="o",
                        label="Illuminated point",
                    )

                    # Light beam
                    ax_3d.plot(
                        [
                            head[0],
                            illuminated[0],
                        ],
                        [
                            head[1],
                            illuminated[1],
                        ],
                        [
                            head[2],
                            illuminated[2],
                        ],
                        linestyle="--",
                        linewidth=2,
                    )

                ax_3d.legend(
                    loc="upper right"
                )

                # --------------------------------------------
                # Refresh
                # --------------------------------------------

                fig.canvas.draw_idle()

                plt.pause(
                    0.001
                )

    finally:

        camera.release()

        plt.ioff()

        plt.close(
            fig
        )


if __name__ == "__main__":
    main()

    