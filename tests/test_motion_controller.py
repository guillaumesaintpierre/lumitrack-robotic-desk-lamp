from software.control.motion_controller import (
    JointConfig,
    JointMotionController,
)


def test_joint_moves_toward_positive_target():

    controller = JointMotionController(
        JointConfig(
            max_velocity_deg_s=60.0,
            max_acceleration_deg_s2=180.0,
        )
    )

    state = controller.update(
        target_angle_deg=20.0,
        dt=0.02,
    )

    assert state.angle_deg > 0.0
    assert state.velocity_deg_s > 0.0


def test_velocity_limit():

    controller = JointMotionController(
        JointConfig(
            max_velocity_deg_s=60.0,
            max_acceleration_deg_s2=10000.0,
        )
    )

    state = controller.update(
        target_angle_deg=100.0,
        dt=0.1,
    )

    assert state.velocity_deg_s <= 60.0


def test_controller_reaches_target():

    controller = JointMotionController(
        JointConfig(
            max_velocity_deg_s=60.0,
            max_acceleration_deg_s2=180.0,
        )
    )

    for _ in range(300):
        controller.update(
            target_angle_deg=20.0,
            dt=0.02,
        )

    assert abs(
        controller.state.angle_deg - 20.0
    ) < 0.5

    