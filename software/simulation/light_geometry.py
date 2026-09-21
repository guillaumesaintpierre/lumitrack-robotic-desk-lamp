import math
import numpy as np

from software.control.kinematics import LampGeometry


def head_position(
    base_yaw_deg: float,
    geometry: LampGeometry,
) -> np.ndarray:

    yaw = math.radians(base_yaw_deg)

    return np.array(
        [
            geometry.head_radius_m * math.cos(yaw),
            geometry.head_radius_m * math.sin(yaw),
            geometry.head_height_m,
        ],
        dtype=float,
    )


def light_direction(
    base_yaw_deg: float,
    head_pitch_down_deg: float,
) -> np.ndarray:

    yaw = math.radians(base_yaw_deg)
    pitch = math.radians(head_pitch_down_deg)

    return np.array(
        [
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            -math.sin(pitch),
        ],
        dtype=float,
    )


def desk_intersection(
    base_yaw_deg: float,
    head_pitch_down_deg: float,
    geometry: LampGeometry,
) -> np.ndarray:

    head = head_position(
        base_yaw_deg,
        geometry,
    )

    direction = light_direction(
        base_yaw_deg,
        head_pitch_down_deg,
    )

    if direction[2] >= -1e-9:
        raise ValueError(
            "Light beam does not point toward the desk."
        )

    t = -head[2] / direction[2]

    return head + t * direction


def tracking_error_m(
    target: np.ndarray,
    illuminated_point: np.ndarray,
) -> float:

    return float(
        np.linalg.norm(
            target[:2] - illuminated_point[:2]
        )
    )
