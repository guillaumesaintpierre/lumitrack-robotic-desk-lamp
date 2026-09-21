from dataclasses import dataclass


KGF_CM_TO_NM = 0.0980665


@dataclass(frozen=True)
class Servo:
    stall_torque_kgf_cm: float
    usable_fraction: float = 0.30


@dataclass(frozen=True)
class Transmission:
    ratio: float
    efficiency: float = 0.85


def kgf_cm_to_nm(torque_kgf_cm: float) -> float:
    return torque_kgf_cm * KGF_CM_TO_NM


def output_range_deg(
    servo_range_deg: float,
    transmission_ratio: float,
) -> float:
    return servo_range_deg / transmission_ratio


def output_torque_nm(
    servo: Servo,
    transmission: Transmission,
) -> float:

    stall_torque_nm = kgf_cm_to_nm(
        servo.stall_torque_kgf_cm
    )

    design_torque_nm = (
        stall_torque_nm
        * servo.usable_fraction
    )

    return (
        design_torque_nm
        * transmission.ratio
        * transmission.efficiency
    )


def required_ratio(
    servo_range_deg: float,
    desired_output_range_deg: float,
) -> float:

    return (
        servo_range_deg
        / desired_output_range_deg
    )


if __name__ == "__main__":

    servo = Servo(
        stall_torque_kgf_cm=11.0
    )

    servo_range = 180.0

    # Base yaw
    desired_base_range = 100.0

    base_ratio = required_ratio(
        servo_range,
        desired_base_range,
    )

    base_transmission = Transmission(
        ratio=base_ratio
    )

    # Head pitch
    desired_head_range = 120.0

    head_ratio = required_ratio(
        servo_range,
        desired_head_range,
    )

    head_transmission = Transmission(
        ratio=head_ratio
    )

    print("=== LumiTrack actuator sizing ===")

    print()
    print("BASE")
    print(f"Required ratio: {base_ratio:.2f}:1")
    print(
        f"Output range: "
        f"{output_range_deg(servo_range, base_ratio):.1f} deg"
    )
    print(
        f"Estimated design torque: "
        f"{output_torque_nm(servo, base_transmission):.3f} N.m"
    )

    print()
    print("HEAD")
    print(f"Required ratio: {head_ratio:.2f}:1")
    print(
        f"Output range: "
        f"{output_range_deg(servo_range, head_ratio):.1f} deg"
    )
    print(
        f"Estimated design torque: "
        f"{output_torque_nm(servo, head_transmission):.3f} N.m"
    )
    