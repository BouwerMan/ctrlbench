import streamlit as st
from ctrlbench.sim import PidGains, PlantConfig, ProfileConfig, Simulator
from ctrlbench.plot import plot_interactive_dashboard
import math

STEP_ANGLE_DEG = 1.8
RADIANS_PER_STEP = math.radians(STEP_ANGLE_DEG)

st.set_page_config(page_title="PID Tuner", layout="wide")

st.sidebar.header("PID Gains")

kp = st.sidebar.slider(
    "Kp (Proportional)", min_value=0.0, max_value=2.0, value=0.5, format="%.5f"
)
ki = st.sidebar.slider(
    "Ki (Integral)", min_value=0.0, max_value=0.5, value=0.05, format="%.5f"
)
kd = st.sidebar.slider(
    "Kd (Derivative)", min_value=0.0, max_value=0.1, value=0.01, format="%.5f"
)

st.sidebar.markdown("---")
st.sidebar.header("Simulation Settings")

profile_type = st.sidebar.selectbox(
    "Profile Type", options=["Trapezoidal", "Step Response"]
)

target_steps = st.sidebar.slider(
    "Target Steps", min_value=100, max_value=50000, value=10000, step=100
)

dt = st.sidebar.number_input(
    "Time Step (dt)",
    min_value=0.00001,
    max_value=0.01,
    value=0.0001,
    step=0.0001,
    format="%.5f",
)

gains = PidGains(kp=kp, ki=ki, kd=kd)
plant = PlantConfig.xy42sth34()


if profile_type == "Trapezoidal":
    max_vel_rad = 2000 * RADIANS_PER_STEP
    accel_rad = 5000 * RADIANS_PER_STEP
    profile = ProfileConfig(
        max_velocity=max_vel_rad, acceleration=accel_rad, deceleration=accel_rad
    )
else:
    profile = ProfileConfig.step()

sim = Simulator(gains=gains, plant=plant, profile=profile)

if profile_type == "Trapezoidal":
    df = sim.run(start=0.0, end=target_steps * RADIANS_PER_STEP, dt=dt)
else:

    def square_wave(t: float) -> float:
        return 1.0 if (t % 2.0) < 1.0 else 0.0

    df = sim.run_signal(square_wave, 5.0, dt)

print("Converting radians to steps for plotting")
df["setpoint"] = df["setpoint"] / RADIANS_PER_STEP
df["actual"] = df["actual"] / RADIANS_PER_STEP
df["error"] = df["error"] / RADIANS_PER_STEP

st.title("Stepper Motor PID Tuner")
fig = plot_interactive_dashboard(df)
st.plotly_chart(fig, width="stretch")
