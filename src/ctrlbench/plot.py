import matplotlib.pyplot as plt
import pandas as pd


def plot_basic_dashboard(df: pd.DataFrame):
    # Create 2 vertically stacked plots sharing the X (time) axis
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # --- Top Plot: Position Tracking ---
    ax1.plot(df["time"], df["setpoint"], label="Setpoint", linestyle="--", color="gray")
    ax1.plot(df["time"], df["actual"], label="Actual Position", color="blue")
    ax1.set_title("System Response")
    ax1.set_ylabel("Position")
    ax1.legend()
    ax1.grid(True)

    # --- Bottom Plot: Control Effort ---
    ax2.plot(df["time"], df["output"], label="Control Output", color="red")
    ax2.set_title("Controller Output (Torque)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Command")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()
