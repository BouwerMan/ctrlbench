import pandas as pd
from ctrlbench.sim import PidGains, PlantConfig, ProfileConfig, Simulator
from ctrlbench.plot import plot_basic_dashboard


def main():
    plant = PlantConfig.simple(inertia=1.0)

    profile = ProfileConfig(
        max_velocity=10.0,
        acceleration=5.0,
        deceleration=5.0,
    )

    gains = PidGains(
        kp=5.0,
        ki=0.1,
        kd=2.5,
        integral_limit_max=100.0,
        integral_limit_min=-100.0,
    )

    sim = Simulator(gains, plant, profile)

    df = sim.run(start=0.0, end=50.0, dt=0.01)
    plot_basic_dashboard(df)


if __name__ == "__main__":
    main()
