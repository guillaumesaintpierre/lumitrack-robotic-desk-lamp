from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LampGeometry:
    """
    Simplified geometry of the lamp.

    head_radius_m:
        Horizontal distance between the base yaw axis
        and the lamp-head pivot.

    head_height_m:
        Height of the lamp-head pivot above the desk.
    """

    head_radius_m: float
    head_height_m: float


@dataclass(frozen=True)
class TargetAngles:
    """
    Desired optical orientation of the lamp.
    """

    base_yaw_deg: float
    head_pitch_down_deg: float


class UnreachableTargetError(ValueError):
    pass


def solve_target_angles(
    x_m: float,
    y_m: float,
    z_m: float,
    geometry: LampGeometry,
) -> TargetAngles:
    """
    Compute the desired lamp orientation for a target point.

    Coordinate system:
        x: forward
        y: lateral
        z: upward

    Returns:
        base yaw angle in degrees
        downward lamp-head pitch angle in degrees
    """

    rho = math.hypot(x_m, y_m)

    if rho <= geometry.head_radius_m:
        raise UnreachableTargetError(
            "Target is inside the radial position of the lamp head."
        )

    base_yaw_rad = math.atan2(y_m, x_m)

    horizontal_distance = rho - geometry.head_radius_m
    vertical_distance = geometry.head_height_m - z_m

    head_pitch_rad = math.atan2(
        vertical_distance,
        horizontal_distance,
    )

    return TargetAngles(
        base_yaw_deg=math.degrees(base_yaw_rad),
        head_pitch_down_deg=math.degrees(head_pitch_rad),
    )


if __name__ == "__main__":

    # Temporary geometry used only to demonstrate the algorithm.
    # These values are NOT final FORSÅ measurements.

    geometry = LampGeometry(
        head_radius_m=0.080,
        head_height_m=0.257,
    )

    target = (0.50, 0.10, 0.0)

    angles = solve_target_angles(
        *target,
        geometry=geometry,
    )

    print("Target:", target)
    print(f"Base yaw: {angles.base_yaw_deg:.2f} deg")
    print(f"Head pitch: {angles.head_pitch_down_deg:.2f} deg")