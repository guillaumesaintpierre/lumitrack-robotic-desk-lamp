import numpy as np

from software.control.kinematics import LampGeometry
from software.simulation.digital_twin import (
    LumiTrackDigitalTwin,
)


def test_initial_state():

    robot = LumiTrackDigitalTwin(
        LampGeometry(
            head_radius_m=0.20,
            head_height_m=0.40,
        )
    )

    assert np.allclose(
        robot.state_vector(),
        [0.0, 45.0],
    )


def test_target_changes_base_angle():

    robot = LumiTrackDigitalTwin(
        LampGeometry(
            head_radius_m=0.20,
            head_height_m=0.40,
        )
    )

    robot.command_target(
        x_m=0.50,
        y_m=0.20,
    )

    assert robot.base_yaw_deg > 0

def test_base_limit():

    robot = LumiTrackDigitalTwin(
        LampGeometry(
            head_radius_m=0.20,
            head_height_m=0.40,
        )
    )

    # This target requires a yaw angle
    # greater than the allowed +100 degrees.
    robot.command_target(
        x_m=-1.00,
        y_m=0.20,
    )

    assert robot.base_yaw_deg <= 100.0
    assert robot.base_yaw_deg == 100.0
