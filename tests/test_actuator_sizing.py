import math

from software.control.actuator_sizing import (
    Servo,
    Transmission,
    output_range_deg,
    output_torque_nm,
    required_ratio,
)


def test_base_ratio():

    ratio = required_ratio(
        servo_range_deg=180,
        desired_output_range_deg=100,
    )

    assert math.isclose(
        ratio,
        1.8,
        abs_tol=1e-9,
    )


def test_base_output_range():

    result = output_range_deg(
        servo_range_deg=180,
        transmission_ratio=1.8,
    )

    assert math.isclose(
        result,
        100.0,
        abs_tol=1e-9,
    )


def test_output_torque_positive():

    servo = Servo(
        stall_torque_kgf_cm=11.0
    )

    transmission = Transmission(
        ratio=1.8
    )

    torque = output_torque_nm(
        servo,
        transmission,
    )

    assert torque > 0

    