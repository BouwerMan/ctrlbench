import matplotlib.pyplot as plt
import pandas as pd


def plot_basic_dashboard(df: pd.DataFrame):
    # Create 3 vertically stacked plots sharing the X (time) axis
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # --- Top Plot: Position Tracking ---
    ax1.plot(df["time"], df["setpoint"], label="Setpoint", linestyle="--", color="gray")
    ax1.plot(df["time"], df["actual"], label="Actual Position", color="blue")
    ax1.set_title("System Response")
    ax1.set_ylabel("Position")
    ax1.legend()
    ax1.grid(True)

    # --- Middle Plot: Tracking Error ---
    # Plotting error on its own axis lets you zoom in on tiny oscillations
    ax2.plot(df["time"], df["error"], label="Error (Setpoint - Actual)", color="orange")
    # Adding a zero-line makes it easy to see when the motor crosses the target
    ax2.axhline(0, color="black", linestyle="-", linewidth=0.8)
    ax2.set_title("Tracking Error")
    ax2.set_ylabel("Error")
    ax2.legend()
    ax2.grid(True)

    # --- Bottom Plot: Control Effort ---
    ax3.plot(df["time"], df["output"], label="Control Output", color="red")
    ax3.set_title("Controller Output (Torque)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Command")
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    plt.show()
