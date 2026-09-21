import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from software.control.kinematics import LampGeometry
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


DT = 0.04

GEOMETRY = LampGeometry(
    head_radius_m=0.20,
    head_height_m=0.40,
)

kinematic_model = LumiTrackDigitalTwin(
    geometry=GEOMETRY
)

motion_controller = LumiTrackMotionController()


def target_trajectory(num_points=300):

    t = np.linspace(
        0.0,
        2.0 * np.pi,
        num_points,
    )

    x = 0.50 + 0.05 * np.cos(t)
    y = 0.22 * np.sin(t)
    z = np.zeros_like(t)

    return np.column_stack(
        (x, y, z)
    )


trajectory = target_trajectory()


fig = plt.figure(
    figsize=(10, 7)
)

ax = fig.add_subplot(
    111,
    projection="3d",
)


def configure_axes():

    ax.set_xlim(
        -0.15,
        0.70,
    )

    ax.set_ylim(
        -0.40,
        0.40,
    )

    ax.set_zlim(
        0.0,
        0.55,
    )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    ax.set_title(
        "LumiTrack — Dynamic Tracking Simulation"
    )

    ax.view_init(
        elev=25,
        azim=-60,
    )


def update(frame):

    ax.cla()
    configure_axes()

    target = trajectory[frame]

    # Desired angles from inverse kinematics
    desired = kinematic_model.command_target(
        x_m=target[0],
        y_m=target[1],
        z_m=target[2],
    )

    # Actual simulated actuator state
    state = motion_controller.update(
        target_base_deg=desired.base_yaw_deg,
        target_head_deg=desired.head_pitch_down_deg,
        dt=DT,
    )

    head = head_position(
        state.base_angle_deg,
        GEOMETRY,
    )

    illuminated = desk_intersection(
        state.base_angle_deg,
        state.head_angle_deg,
        GEOMETRY,
    )

    error = tracking_error_m(
        target,
        illuminated,
    )

    # Desk outline
    desk_x = [
        -0.10,
        0.70,
        0.70,
        -0.10,
        -0.10,
    ]

    desk_y = [
        -0.40,
        -0.40,
        0.40,
        0.40,
        -0.40,
    ]

    desk_z = [0.0] * 5

    ax.plot(
        desk_x,
        desk_y,
        desk_z,
        linewidth=1,
    )

    # Simplified arm
    ax.plot(
        [0.0, head[0]],
        [0.0, head[1]],
        [0.0, head[2]],
        linewidth=4,
    )

    # Head
    ax.scatter(
        head[0],
        head[1],
        head[2],
        s=80,
    )

    # Target
    ax.scatter(
        target[0],
        target[1],
        target[2],
        s=100,
        marker="x",
        label="Target",
    )

    # Actual illuminated point
    ax.scatter(
        illuminated[0],
        illuminated[1],
        illuminated[2],
        s=70,
        marker="o",
        label="Illuminated point",
    )

    # Actual beam
    ax.plot(
        [head[0], illuminated[0]],
        [head[1], illuminated[1]],
        [head[2], illuminated[2]],
        linestyle="--",
        linewidth=2,
    )

    # Error line on desk
    ax.plot(
        [
            target[0],
            illuminated[0],
        ],
        [
            target[1],
            illuminated[1],
        ],
        [
            0.0,
            0.0,
        ],
        linewidth=2,
    )

    info = (
        f"Desired base: {desired.base_yaw_deg:.1f}°\n"
        f"Actual base: {state.base_angle_deg:.1f}°\n"
        f"Desired head: {desired.head_pitch_down_deg:.1f}°\n"
        f"Actual head: {state.head_angle_deg:.1f}°\n"
        f"Tracking error: {error * 100:.1f} cm"
    )

    ax.text2D(
        0.02,
        0.95,
        info,
        transform=ax.transAxes,
        verticalalignment="top",
    )

    ax.legend(
        loc="upper right"
    )


animation = FuncAnimation(
    fig,
    update,
    frames=len(trajectory),
    interval=int(DT * 1000),
    repeat=True,
)

plt.show()
