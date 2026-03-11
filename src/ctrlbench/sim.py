from typing import Callable
from dataclasses import dataclass, field
import pandas as pd
from enum import Enum
from math import exp


# @dataclass
# class PlantConfig:
#     response: float = 1.0  # How well the plant tracks commands (0.0–1.0)
#     disturbance: float = 0.0  # Constant force applied each tick


@dataclass
class ProfileConfig:
    max_velocity: float = 0.0
    acceleration: float = 0.0
    deceleration: float = 0.0

    @classmethod
    def step(cls) -> "ProfileConfig":
        """Instantaneous jump to target position. Used for tuning step response."""
        return cls(
            max_velocity=float("inf"),
            acceleration=float("inf"),
            deceleration=float("inf"),
        )


@dataclass
class SimResult:
    time: list[float] = field(default_factory=list)
    setpoint: list[float] = field(default_factory=list)
    actual: list[float] = field(default_factory=list)
    error: list[float] = field(default_factory=list)
    output: list[float] = field(default_factory=list)
    integral: list[float] = field(default_factory=list)


class Simulator:
    """
    Runs a PID controller against a simulated plant and returns time-series data.
    """

    def __init__(self, gains: PidGains, plant: PlantConfig, profile: ProfileConfig):
        self.gains = gains
        self.plant = plant
        self.profile = profile

    def run(self, start: float, end: float, dt: float = 0.001) -> pd.DataFrame:
        """
        Simulate a move from start to end position.

        Args:
            start: Starting position.
            end: Target position.

        Returns:
            SimResult containing all recorded signals.
        """
        print(f"Hello from Simulator! Moving from {start} to {end}.")
        pg = ProfileGenerator(profile=self.profile)
        pg.move(start, end)

        pm = PlantModel(config=self.plant)

        pid = PidController(self.gains)

        time_data, setpoint_data, actual_data, error_data, output_data = (
            [],
            [],
            [],
            [],
            [],
        )

        current_time = 0.0
        settling_time_remaining = 1.0  # Extra time to run after reaching target
        max_time = dt * 10000  # Safety limit to prevent infinite loops

        while not pg.is_finished() or settling_time_remaining > 0:
            if current_time > max_time:
                break

            pg.calculate_next_step(dt)
            setpoint = pg.position
            actual = pm.position

            error = setpoint - actual

            torque_command = pid.update(error, dt)
            pm.step(command=torque_command, dt=dt)

            time_data.append(current_time)
            setpoint_data.append(setpoint)
            actual_data.append(actual)
            error_data.append(error)
            output_data.append(torque_command)

            current_time += dt
            if pg.is_finished():
                settling_time_remaining -= dt

        df = pd.DataFrame(
            {
                "time": time_data,
                "setpoint": setpoint_data,
                "actual": actual_data,
                "error": error_data,
                "output": output_data,
            }
        )

        return df

    def run_signal(
        self, signal_func: Callable[[float], float], duration: float, dt: float = 0.001
    ) -> pd.DataFrame:
        """
        Runs the PID controller against a continuous mathematical signal (like a square wave).

        Args:
            signal_func: A function that takes current time (float) and returns the desired setpoint (float).
            duration: How long to run the simulation in seconds.
            dt: Time step.
        """
        pm = PlantModel(config=self.plant)
        pid = PidController(self.gains)

        time_data, setpoint_data, actual_data, error_data, output_data = (
            [],
            [],
            [],
            [],
            [],
        )
        current_time = 0.0

        while current_time <= duration:
            # 1. Ask the custom function for the ideal setpoint at this exact millisecond
            setpoint = signal_func(current_time)

            actual = pm.position
            error = setpoint - actual

            # 2. Update PID and Plant
            torque_command = pid.update(error, dt)
            pm.step(command=torque_command, dt=dt)

            # 3. Log Data
            time_data.append(current_time)
            setpoint_data.append(setpoint)
            actual_data.append(actual)
            error_data.append(error)
            output_data.append(torque_command)

            current_time += dt

        return pd.DataFrame(
            {
                "time": time_data,
                "setpoint": setpoint_data,
                "actual": actual_data,
                "error": error_data,
                "output": output_data,
            }
        )


@dataclass
class PidGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit_max: float = 1000.0
    integral_limit_min: float = -1000.0


class PidController:
    """
    Owns error/integral/derivative math
    """

    def __init__(self, gains: PidGains):
        self.gains = gains
        self.first_run = True
        self.error_prev = 0.0
        self.error = 0.0
        self.integral = 0.0

    def reset(self):
        self.first_run = True
        self.error_prev = 0.0
        self.error = 0.0
        self.integral = 0.0

    def update(self, error: float, dt: float) -> float:
        if self.first_run:
            self.error_prev = error
            self.first_run = False

        self.integral += error * dt

        # Clamp integral to prevent windup
        self.integral = max(
            self.gains.integral_limit_min,
            min(self.gains.integral_limit_max, self.integral),
        )

        if dt > 0.0:
            derivative = (error - self.error_prev) / dt
        else:
            derivative = 0.0

        self.error_prev = error

        return (
            (self.gains.kp * error)
            + (self.gains.ki * self.integral)
            + (self.gains.kd * derivative)
        )


@dataclass
class PlantConfig:
    rotor_inertia: float
    peak_torque: float
    electrical_tau: float
    viscous_friction: float
    static_friction: float
    disturbance: float

    @classmethod
    def simple(cls, inertia: float = 1.0) -> "PlantConfig":
        """Double integrator, no friction, instant electrical response."""
        return cls(
            rotor_inertia=inertia,
            peak_torque=float("inf"),  # No limit
            electrical_tau=0.0,  # Instant torque
            viscous_friction=0.0,
            static_friction=0.0,
            disturbance=0.0,
        )

    @classmethod
    def from_datasheet(
        cls, rotor_inertia: float, peak_torque: float, electrical_tau: float
    ) -> "PlantConfig":
        """Physical model without non-linear stiction."""
        return cls(
            rotor_inertia=rotor_inertia,
            peak_torque=peak_torque,
            electrical_tau=electrical_tau,
            viscous_friction=0.01,  # Small default drag
            static_friction=0.0,
            disturbance=0.0,
        )

    @classmethod
    def xy42sth34(cls) -> "PlantConfig":
        """
        Physical model for the XY42STH34-0354A NEMA 17 Stepper Motor.
        Units are standard SI (kg*m^2, Nm, Seconds).
        """
        return cls(
            rotor_inertia=3.5e-6,  # 35 g-cm^2
            peak_torque=0.157,  # 1.6 kg-cm Holding Torque
            electrical_tau=0.00097,  # 33mH / 34 Ohms
            static_friction=0.0118,  # 120 g-cm Detent Torque
            viscous_friction=0.001,  # Estimated base drag
            disturbance=0.0,
        )


class PlantModel:
    """
    owns plant dynamics
    """

    def __init__(self, config: PlantConfig):
        self.config = config
        self.torque_prev = 0.0
        self.velocity = 0.0
        self.position = 0.0

    def reset(self):
        self.torque_prev = 0.0
        self.velocity = 0.0
        self.position = 0.0

    def step(self, command: float, dt: float):
        # Apply first-order lag to torque command
        if self.config.electrical_tau > 0.0:
            smooth_factor = 1.0 - exp(-dt / self.config.electrical_tau)
            actual = self.torque_prev + smooth_factor * (command - self.torque_prev)
        else:
            actual = command

        # Clamp to peak torque
        torque = max(-self.config.peak_torque, min(self.config.peak_torque, actual))

        self.torque_prev = torque

        viscous_friction = self.velocity * self.config.viscous_friction
        net_torque = torque + self.config.disturbance - viscous_friction

        acceleration = net_torque / self.config.rotor_inertia

        self.velocity += acceleration * dt
        self.position += self.velocity * dt


class ProfileGeneratorState(Enum):
    IDLE = 0
    ACCELERATING = 1
    CRUISING = 2
    DECELERATING = 3


class ProfileGenerator:
    def __init__(self, profile: ProfileConfig):
        self.profile = profile
        self.state = ProfileGeneratorState.IDLE
        self.direction = 1  # 1 for forward, -1 for reverse
        self.position = 0.0
        self.velocity = 0.0
        self.target_position = 0.0

    def is_finished(self) -> bool:
        return self.state == ProfileGeneratorState.IDLE

    def move(self, start: float, end: float):
        if end > start:
            new_direction = 1
        elif end < start:
            new_direction = -1
        else:
            self.state = ProfileGeneratorState.IDLE
            return

        if self.state != ProfileGeneratorState.IDLE:
            self.target_position = end

            if new_direction != self.direction:
                self.state = ProfileGeneratorState.DECELERATING
            elif self.state == ProfileGeneratorState.DECELERATING:
                dist_left = abs(self.target_position - start)
                if dist_left > self.calculate_braking_distance():
                    self.state = ProfileGeneratorState.ACCELERATING
        else:
            # Accelerating from standstill
            self.position = start
            self.target_position = end
            self.direction = new_direction
            self.velocity = 0.0
            self.state = ProfileGeneratorState.ACCELERATING

        self.calculate_next_step(0.0)

    def calculate_next_step(self, dt: float):
        if self.state == ProfileGeneratorState.IDLE:
            return

        match self.state:
            case ProfileGeneratorState.ACCELERATING:
                active_accel = self.profile.acceleration
            case ProfileGeneratorState.CRUISING:
                active_accel = 0.0
            case ProfileGeneratorState.DECELERATING:
                active_accel = -self.profile.deceleration

        distance_step = (self.velocity * dt) + (0.5 * active_accel * dt**2)
        self.position += self.direction * distance_step
        self.velocity += active_accel * dt

        if (
            self.state == ProfileGeneratorState.ACCELERATING
            and self.velocity >= self.profile.max_velocity
        ):
            self.velocity = self.profile.max_velocity
            self.state = ProfileGeneratorState.CRUISING
        elif self.state == ProfileGeneratorState.DECELERATING and self.velocity <= 0.0:
            self.velocity = 0.0
            dist_error = self.target_position - self.position
            if abs(dist_error) < 0.5:
                self.position = self.target_position
                self.state = ProfileGeneratorState.IDLE
            else:
                self.direction = 1 if dist_error > 0 else -1
                self.state = ProfileGeneratorState.ACCELERATING

        # Brake check
        if self.state in [
            ProfileGeneratorState.ACCELERATING,
            ProfileGeneratorState.CRUISING,
        ]:
            dist_left = abs(self.target_position - self.position)
            if dist_left <= self.calculate_braking_distance():
                self.state = ProfileGeneratorState.DECELERATING

    def calculate_braking_distance(self) -> float:
        assert self.profile.deceleration > 0, (
            "Deceleration must be greater than zero to calculate braking distance."
        )
        v_sq = self.velocity**2
        d_brake = 0.5 * (v_sq / self.profile.deceleration)
        return d_brake
