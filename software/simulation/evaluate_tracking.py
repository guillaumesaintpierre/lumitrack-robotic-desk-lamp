from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from software.control.kinematics import LampGeometry
from software.control.motion_controller import LumiTrackMotionController
from software.simulation.digital_twin import LumiTrackDigitalTwin
from software.simulation.light_geometry import (
    desk_intersection,
    tracking_error_m,
)


DT = 0.04
NUM_POINTS = 300

GEOMETRY = LampGeometry(
    head_radius_m=0.080,
    head_height_m=0.257,
)


def create_target_trajectory():
    """
    Create a smooth closed trajectory on the desk.

    Returns:
        trajectory: array of shape (N, 3)
    """

    t = np.linspace(
        0.0,
        2.0 * np.pi,
        NUM_POINTS,
    )

    x = 0.50 + 0.05 * np.cos(t)
    y = 0.22 * np.sin(t)
    z = np.zeros_like(t)

    trajectory = np.column_stack(
        (x, y, z)
    )

    return trajectory


def rms(values):
    """
    Root-mean-square value.
    """

    values = np.asarray(
        values,
        dtype=float,
    )

    return float(
        np.sqrt(
            np.mean(values ** 2)
        )
    )


def run_simulation():
    """
    Run the dynamic tracking simulation.

    Returns:
        Dictionary containing time histories for:
        - desired joint angles
        - actual joint angles
        - workspace tracking error
    """

    kinematic_model = LumiTrackDigitalTwin(
        geometry=GEOMETRY
    )

    controller = LumiTrackMotionController()

    trajectory = create_target_trajectory()

    time = (
        np.arange(NUM_POINTS)
        * DT
    )

    desired_base = []
    actual_base = []

    desired_head = []
    actual_head = []

    tracking_errors = []

    for target in trajectory:

        desired = (
            kinematic_model.command_target(
                x_m=target[0],
                y_m=target[1],
                z_m=target[2],
            )
        )

        state = controller.update(
            target_base_deg=desired.base_yaw_deg,
            target_head_deg=desired.head_pitch_down_deg,
            dt=DT,
        )

        illuminated = desk_intersection(
            base_yaw_deg=state.base_angle_deg,
            head_pitch_down_deg=state.head_angle_deg,
            geometry=GEOMETRY,
        )

        error = tracking_error_m(
            target,
            illuminated,
        )

        desired_base.append(
            desired.base_yaw_deg
        )

        actual_base.append(
            state.base_angle_deg
        )

        desired_head.append(
            desired.head_pitch_down_deg
        )

        actual_head.append(
            state.head_angle_deg
        )

        tracking_errors.append(
            error
        )

    return {
        "time": time,
        "trajectory": trajectory,
        "desired_base": np.asarray(
            desired_base,
            dtype=float,
        ),
        "actual_base": np.asarray(
            actual_base,
            dtype=float,
        ),
        "desired_head": np.asarray(
            desired_head,
            dtype=float,
        ),
        "actual_head": np.asarray(
            actual_head,
            dtype=float,
        ),
        "tracking_error": np.asarray(
            tracking_errors,
            dtype=float,
        ),
    }


def print_metrics(data):
    """
    Print full-run and steady-state performance metrics.
    """

    error = data["tracking_error"]

    base_error = (
        data["desired_base"]
        - data["actual_base"]
    )

    head_error = (
        data["desired_head"]
        - data["actual_head"]
    )

    # --------------------------------------------------
    # Full-run metrics
    # --------------------------------------------------

    print(
        "=== LumiTrack Dynamic Tracking Evaluation ==="
    )

    print()

    print(
        f"Mean tracking error: "
        f"{np.mean(error) * 100:.2f} cm"
    )

    print(
        f"RMS tracking error: "
        f"{rms(error) * 100:.2f} cm"
    )

    print(
        f"95th percentile error: "
        f"{np.percentile(error, 95) * 100:.2f} cm"
    )

    print(
        f"Maximum tracking error: "
        f"{np.max(error) * 100:.2f} cm"
    )

    print()

    print(
        f"Base RMS angle error: "
        f"{rms(base_error):.2f} deg"
    )

    print(
        f"Head RMS angle error: "
        f"{rms(head_error):.2f} deg"
    )

    # --------------------------------------------------
    # Steady-state metrics
    # --------------------------------------------------

    warmup_time_s = 1.0

    warmup_samples = int(
        warmup_time_s / DT
    )

    steady_error = (
        error[warmup_samples:]
    )

    steady_base_error = (
        base_error[warmup_samples:]
    )

    steady_head_error = (
        head_error[warmup_samples:]
    )

    print()

    print(
        f"=== Steady-state performance "
        f"(after {warmup_time_s:.1f} s) ==="
    )

    print()

    print(
        f"Mean tracking error: "
        f"{np.mean(steady_error) * 100:.2f} cm"
    )

    print(
        f"RMS tracking error: "
        f"{rms(steady_error) * 100:.2f} cm"
    )

    print(
        f"95th percentile error: "
        f"{np.percentile(steady_error, 95) * 100:.2f} cm"
    )

    print(
        f"Maximum tracking error: "
        f"{np.max(steady_error) * 100:.2f} cm"
    )

    print()

    print(
        f"Base RMS angle error: "
        f"{rms(steady_base_error):.2f} deg"
    )

    print(
        f"Head RMS angle error: "
        f"{rms(steady_head_error):.2f} deg"
    )


def save_figures(data):
    """
    Save tracking figures to media/figures/.
    """

    output_dir = Path(
        "media/figures"
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    time = data["time"]

    # --------------------------------------------------
    # Base yaw tracking
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 5)
    )

    plt.plot(
        time,
        data["desired_base"],
        label="Desired",
    )

    plt.plot(
        time,
        data["actual_base"],
        label="Actual",
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Base yaw [deg]"
    )

    plt.title(
        "LumiTrack — Base Yaw Tracking"
    )

    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(
        output_dir
        / "base_yaw_tracking.png",
        dpi=200,
    )

    plt.close()

    # --------------------------------------------------
    # Head pitch tracking
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 5)
    )

    plt.plot(
        time,
        data["desired_head"],
        label="Desired",
    )

    plt.plot(
        time,
        data["actual_head"],
        label="Actual",
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Head pitch [deg]"
    )

    plt.title(
        "LumiTrack — Head Pitch Tracking"
    )

    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(
        output_dir
        / "head_pitch_tracking.png",
        dpi=200,
    )

    plt.close()

    # --------------------------------------------------
    # Workspace tracking error
    # --------------------------------------------------

    plt.figure(
        figsize=(8, 5)
    )

    plt.plot(
        time,
        data["tracking_error"]
        * 100,
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Tracking error [cm]"
    )

    plt.title(
        "LumiTrack — Workspace Tracking Error"
    )

    plt.grid(True)
    plt.tight_layout()

    plt.savefig(
        output_dir
        / "tracking_error.png",
        dpi=200,
    )

    plt.close()


def main():
    """
    Run evaluation, print metrics, and save figures.
    """

    results = run_simulation()

    print_metrics(
        results
    )

    save_figures(
        results
    )

    print()

    print(
        "Figures saved to media/figures/"
    )


if __name__ == "__main__":
    main()

    