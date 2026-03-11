import math
import pandas as pd
from ctrlbench.sim import PidGains, PlantConfig, ProfileConfig, Simulator
from ctrlbench.plot import plot_basic_dashboard

STEP_ANGLE_DEG = 1.8
RADIANS_PER_STEP = math.radians(STEP_ANGLE_DEG)


def main():
    print("Configuring simulation")
    plant = PlantConfig.xy42sth34()

    target_steps = 10000.0

    max_vel_rad = 2000 * RADIANS_PER_STEP
    accel_rad = 5000 * RADIANS_PER_STEP

    profile = ProfileConfig(
        max_velocity=max_vel_rad, acceleration=accel_rad, deceleration=accel_rad
    )

    gains = PidGains(
        kp=5.0,
        ki=0.05,
        kd=0.01,
        integral_limit_max=0.1,
        integral_limit_min=-0.1,
    )

    print(f"Running simulation for {target_steps} steps")
    sim = Simulator(gains, plant, profile)

    df = sim.run(start=0.0, end=target_steps * RADIANS_PER_STEP, dt=0.0001)

    print("Converting radians to steps for plotting")
    df["setpoint"] = df["setpoint"] / RADIANS_PER_STEP
    df["actual"] = df["actual"] / RADIANS_PER_STEP
    df["error"] = df["error"] / RADIANS_PER_STEP

    fig = plot_basic_dashboard(df)
    fig.show()


if __name__ == "__main__":
    main()
