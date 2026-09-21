from dataclasses import dataclass

import numpy as np

from software.control.kinematics import (
    LampGeometry,
    TargetAngles,
    solve_target_angles,
)


@dataclass
class JointLimits:
    base_min_deg: float = -100.0
    base_max_deg: float = 100.0

    head_min_deg: float = 0.0
    head_max_deg: float = 90.0


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class LumiTrackDigitalTwin:

    def __init__(
        self,
        geometry: LampGeometry,
        limits: JointLimits = JointLimits(),
    ):
        self.geometry = geometry
        self.limits = limits

        self.base_yaw_deg = 0.0
        self.head_pitch_deg = 45.0

    def command_target(
        self,
        x_m: float,
        y_m: float,
        z_m: float = 0.0,
    ) -> TargetAngles:

        target = solve_target_angles(
            x_m=x_m,
            y_m=y_m,
            z_m=z_m,
            geometry=self.geometry,
        )

        base = clamp(
            target.base_yaw_deg,
            self.limits.base_min_deg,
            self.limits.base_max_deg,
        )

        head = clamp(
            target.head_pitch_down_deg,
            self.limits.head_min_deg,
            self.limits.head_max_deg,
        )

        self.base_yaw_deg = base
        self.head_pitch_deg = head

        return TargetAngles(
            base_yaw_deg=base,
            head_pitch_down_deg=head,
        )

    def state_vector(self) -> np.ndarray:

        return np.array(
            [
                self.base_yaw_deg,
                self.head_pitch_deg,
            ],
            dtype=float,
        )


if __name__ == "__main__":

    geometry = LampGeometry(
        head_radius_m=0.080,
        head_height_m=0.257,
    )

    robot = LumiTrackDigitalTwin(
        geometry=geometry
    )

    positions = [
        (0.50, -0.20),
        (0.50, 0.00),
        (0.50, 0.20),
    ]

    for x, y in positions:

        angles = robot.command_target(
            x_m=x,
            y_m=y,
        )

        print()
        print(f"Target: x={x:.2f}, y={y:.2f}")
        print(
            f"Base yaw: "
            f"{angles.base_yaw_deg:.2f} deg"
        )
        print(
            f"Head pitch: "
            f"{angles.head_pitch_down_deg:.2f} deg"
        )
