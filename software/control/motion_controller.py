from dataclasses import dataclass
import math


@dataclass
class JointState:
    angle_deg: float = 0.0
    velocity_deg_s: float = 0.0


@dataclass(frozen=True)
class JointConfig:
    max_velocity_deg_s: float
    max_acceleration_deg_s2: float
    deadband_deg: float = 0.15


class JointMotionController:
    """
    Simple velocity- and acceleration-limited joint controller.
    """

    def __init__(
        self,
        config: JointConfig,
        initial_angle_deg: float = 0.0,
    ):
        self.config = config
        self.state = JointState(
            angle_deg=initial_angle_deg,
            velocity_deg_s=0.0,
        )

    def update(
        self,
        target_angle_deg: float,
        dt: float,
    ) -> JointState:

        error = target_angle_deg - self.state.angle_deg

        # Deadband near target
        if abs(error) < self.config.deadband_deg:
            desired_velocity = 0.0
        else:
            direction = 1.0 if error > 0.0 else -1.0

            desired_velocity = (
                direction
                * self.config.max_velocity_deg_s
            )

            # Avoid overshooting the target
            max_velocity_without_overshoot = abs(error) / dt

            desired_velocity = math.copysign(
                min(
                    abs(desired_velocity),
                    max_velocity_without_overshoot,
                ),
                desired_velocity,
            )

        # Acceleration limiting
        velocity_error = (
            desired_velocity
            - self.state.velocity_deg_s
        )

        max_delta_velocity = (
            self.config.max_acceleration_deg_s2
            * dt
        )

        delta_velocity = max(
            -max_delta_velocity,
            min(
                max_delta_velocity,
                velocity_error,
            ),
        )

        new_velocity = (
            self.state.velocity_deg_s
            + delta_velocity
        )

        new_angle = (
            self.state.angle_deg
            + new_velocity * dt
        )

        self.state = JointState(
            angle_deg=new_angle,
            velocity_deg_s=new_velocity,
        )

        return self.state


@dataclass
class LumiTrackMotionState:
    base_angle_deg: float
    head_angle_deg: float
    base_velocity_deg_s: float
    head_velocity_deg_s: float


class LumiTrackMotionController:

    def __init__(self):

        self.base = JointMotionController(
            JointConfig(
                max_velocity_deg_s=60.0,
                max_acceleration_deg_s2=180.0,
            ),
            initial_angle_deg=0.0,
        )

        self.head = JointMotionController(
            JointConfig(
                max_velocity_deg_s=90.0,
                max_acceleration_deg_s2=240.0,
            ),
            initial_angle_deg=45.0,
        )

    def update(
        self,
        target_base_deg: float,
        target_head_deg: float,
        dt: float,
    ) -> LumiTrackMotionState:

        base_state = self.base.update(
            target_base_deg,
            dt,
        )

        head_state = self.head.update(
            target_head_deg,
            dt,
        )

        return LumiTrackMotionState(
            base_angle_deg=base_state.angle_deg,
            head_angle_deg=head_state.angle_deg,
            base_velocity_deg_s=base_state.velocity_deg_s,
            head_velocity_deg_s=head_state.velocity_deg_s,
        )

    

    