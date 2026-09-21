import math

import pytest

from software.control.kinematics import (
    LampGeometry,
    UnreachableTargetError,
    solve_target_angles,
)


GEOMETRY = LampGeometry(
    head_radius_m=0.20,
    head_height_m=0.40,
)


def test_target_straight_ahead():

    angles = solve_target_angles(
        x_m=0.50,
        y_m=0.0,
        z_m=0.0,
        geometry=GEOMETRY,
    )

    assert math.isclose(
        angles.base_yaw_deg,
        0.0,
        abs_tol=1e-6,
    )

    assert math.isclose(
        angles.head_pitch_down_deg,
        53.130102,
        abs_tol=1e-5,
    )


def test_target_to_the_side():

    angles = solve_target_angles(
        x_m=0.50,
        y_m=0.20,
        z_m=0.0,
        geometry=GEOMETRY,
    )

    assert angles.base_yaw_deg > 0


def test_target_too_close():

    with pytest.raises(UnreachableTargetError):

        solve_target_angles(
            x_m=0.10,
            y_m=0.0,
            z_m=0.0,
            geometry=GEOMETRY,
        )
        