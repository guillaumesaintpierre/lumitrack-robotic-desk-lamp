import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from software.control.kinematics import LampGeometry
from software.simulation.digital_twin import LumiTrackDigitalTwin


GEOMETRY = LampGeometry(
    head_radius_m=0.080,
    head_height_m=0.257,
)

robot = LumiTrackDigitalTwin(
    geometry=GEOMETRY
)


def head_position(base_yaw_deg: float) -> np.ndarray:
    """
    Compute the lamp-head pivot position in world coordinates.
    """

    yaw = math.radians(base_yaw_deg)

    x = GEOMETRY.head_radius_m * math.cos(yaw)
    y = GEOMETRY.head_radius_m * math.sin(yaw)
    z = GEOMETRY.head_height_m

    return np.array([x, y, z])


def create_target_trajectory(num_points: int = 200):

    t = np.linspace(0.0, 2.0 * np.pi, num_points)

    x = 0.50 + 0.05 * np.cos(t)
    y = 0.22 * np.sin(t)
    z = np.zeros_like(t)

    return np.column_stack((x, y, z))


trajectory = create_target_trajectory()


fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection="3d")


def configure_axes():

    ax.set_xlim(-0.15, 0.70)
    ax.set_ylim(-0.40, 0.40)
    ax.set_zlim(0.0, 0.55)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    ax.set_title("LumiTrack — 2-DOF Digital Twin")

    ax.view_init(
        elev=25,
        azim=-60,
    )


def update(frame):

    ax.cla()
    configure_axes()

    target = trajectory[frame]

    x_target = target[0]
    y_target = target[1]
    z_target = target[2]

    angles = robot.command_target(
        x_m=x_target,
        y_m=y_target,
        z_m=z_target,
    )

    head = head_position(
        angles.base_yaw_deg
    )

    base = np.array(
        [0.0, 0.0, 0.0]
    )

    # Desk surface
    desk_x = np.array(
        [-0.10, 0.70, 0.70, -0.10, -0.10]
    )

    desk_y = np.array(
        [-0.40, -0.40, 0.40, 0.40, -0.40]
    )

    desk_z = np.zeros_like(desk_x)

    ax.plot(
        desk_x,
        desk_y,
        desk_z,
        linewidth=1,
    )

    # Lamp arm
    ax.plot(
        [base[0], head[0]],
        [base[1], head[1]],
        [base[2], head[2]],
        linewidth=4,
    )

    # Base vertical axis
    ax.plot(
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.12],
        linewidth=5,
    )

    # Lamp head
    ax.scatter(
        head[0],
        head[1],
        head[2],
        s=80,
    )

    # Target
    ax.scatter(
        x_target,
        y_target,
        z_target,
        s=100,
        marker="x",
    )

    # Light beam
    ax.plot(
        [head[0], x_target],
        [head[1], y_target],
        [head[2], z_target],
        linestyle="--",
        linewidth=2,
    )

    # Target trajectory
    ax.plot(
        trajectory[:, 0],
        trajectory[:, 1],
        trajectory[:, 2],
        linewidth=1,
        alpha=0.4,
    )

    info = (
        f"Target: ({x_target:.2f}, {y_target:.2f}) m\n"
        f"Base yaw: {angles.base_yaw_deg:.1f}°\n"
        f"Head pitch: {angles.head_pitch_down_deg:.1f}°"
    )

    ax.text2D(
        0.02,
        0.95,
        info,
        transform=ax.transAxes,
        verticalalignment="top",
    )


animation = FuncAnimation(
    fig,
    update,
    frames=len(trajectory),
    interval=40,
    repeat=True,
)

plt.show()
