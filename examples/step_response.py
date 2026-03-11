import pandas as pd
import matplotlib.pyplot as plt
from ctrlbench.sim import PidGains, PlantConfig, ProfileConfig, Simulator
from ctrlbench.plot import plot_basic_dashboard


def main():
    print("Configuring Step Response Simulation...")

    plant = PlantConfig.simple(inertia=1.0)

    profile = ProfileConfig.step()

    gains = PidGains(
        kp=20.0,  # High proportional gain acts like a very stiff spring
        ki=0.0,  # No integral needed since there is no static friction
        kd=4.0,  # Low derivative gain acts like weak viscous damping
    )

    # TODO: Allow profile to be None
    sim = Simulator(gains=gains, plant=plant, profile=profile)

    def square_wave(t: float) -> float:
        return 1.0 if (t % 2.0) < 1.0 else 0.0

    print("Running Square Wave test for 5 seconds...")
    df = sim.run_signal(square_wave, 5.0, 0.001)

    print("Simulation complete. Opening Matplotlib dashboard...")
    fig = plot_basic_dashboard(df)
    plt.show()


if __name__ == "__main__":
    main()
