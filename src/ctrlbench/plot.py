import plotly.graph_objects as go
from plotly.subplots import make_subplots
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
    return fig


def plot_interactive_dashboard(df: pd.DataFrame):
    # Create 3 stacked subplots sharing the X-axis
    fig = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.05,
        subplot_titles=("System Response", "Tracking Error", "Controller Output"),
    )

    # High-contrast dark palette
    color_bg = "#282828"
    color_grid = "#3c3836"
    color_text = "#ebdbb2"
    color_setpoint = "#a89984"  # Gray
    color_actual = "#83a598"  # Blue
    color_error = "#fabd2f"  # Yellow
    color_output = "#fb4934"  # Red

    # --- Top Plot: Position ---
    fig.add_trace(
        go.Scatter(
            x=df["time"],
            y=df["setpoint"],
            name="Setpoint",
            line=dict(dash="dash", color=color_setpoint),
        ),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=df["time"], y=df["actual"], name="Actual", line=dict(color=color_actual)
        ),
        row=1,
        col=1,
    )

    # --- Middle Plot: Error ---
    fig.add_trace(
        go.Scatter(
            x=df["time"], y=df["error"], name="Error", line=dict(color=color_error)
        ),
        row=2,
        col=1,
    )
    # Add a zero-line for reference
    fig.add_hline(
        y=0, line_width=1, line_dash="solid", line_color=color_text, row=2, col=1
    )

    # --- Bottom Plot: Output ---
    fig.add_trace(
        go.Scatter(
            x=df["time"], y=df["output"], name="Output", line=dict(color=color_output)
        ),
        row=3,
        col=1,
    )

    # --- Layout & Styling ---
    fig.update_layout(
        height=800,
        plot_bgcolor=color_bg,
        paper_bgcolor=color_bg,
        font=dict(color=color_text),
        margin=dict(l=20, r=20, t=40, b=20),
        showlegend=True,
        # Move legend to the top so it doesn't squish the plots
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    )

    # Update axes to have subtle gridlines
    fig.update_xaxes(showgrid=True, gridwidth=1, gridcolor=color_grid)
    fig.update_xaxes(title_text="Time (s)", row=3, col=1)  # Only label bottom X axis

    fig.update_yaxes(showgrid=True, gridwidth=1, gridcolor=color_grid)
    fig.update_yaxes(title_text="Position", row=1, col=1)
    fig.update_yaxes(title_text="Error", row=2, col=1)
    fig.update_yaxes(title_text="Command", row=3, col=1)

    return fig
