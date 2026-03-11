import streamlit as st
from ctrlbench.sim import PidGains, PlantConfig, ProfileConfig, Simulator
from ctrlbench.plot import plot_interactive_dashboard
import math

STEP_ANGLE_DEG = 1.8
RADIANS_PER_STEP = math.radians(STEP_ANGLE_DEG)

st.set_page_config(page_title="PID Tuner", layout="wide")


def synced_slider_input(
    label: str,
    key_prefix: str,
    min_val: float,
    max_val: float,
    default_val: float,
    step: float = 1.0,
    format_str: str = "%g",
    container=None,  # Pass st.sidebar, or leave None for main area
    col_ratio: tuple = (3, 1),
):
    """Creates a linked slider and number input that stay perfectly in sync."""
    master_key = f"{key_prefix}_master"
    slider_key = f"{key_prefix}_slider"
    number_key = f"{key_prefix}_number"

    # Initialize all three keys once
    if master_key not in st.session_state:
        st.session_state[master_key] = float(default_val)
        st.session_state[slider_key] = float(default_val)
        st.session_state[number_key] = float(default_val)

    def update_from_slider():
        st.session_state[master_key] = st.session_state[slider_key]
        st.session_state[number_key] = st.session_state[slider_key]

    def update_from_number():
        st.session_state[master_key] = st.session_state[number_key]
        st.session_state[slider_key] = st.session_state[number_key]

    ctx = container if container is not None else st
    ctx.markdown(f"**{label}**")
    col1, col2 = ctx.columns(list(col_ratio))

    with col1:
        st.slider(
            label,
            min_value=float(min_val),
            max_value=float(max_val),
            step=float(step),
            format=format_str,
            key=slider_key,  # ← no value= here
            on_change=update_from_slider,
            label_visibility="collapsed",
        )
    with col2:
        st.number_input(
            label,
            min_value=float(min_val),
            max_value=float(max_val),
            step=float(step),
            format=format_str,
            key=number_key,  # ← no value= here
            on_change=update_from_number,
            label_visibility="collapsed",
        )

    return st.session_state[master_key]


st.sidebar.header("PID Gains")

with st.sidebar.expander("Configure Slider Limits"):
    kp_max = st.number_input("Max Kp Limit", min_value=0.01, value=2.0, step=1.0)
    ki_max = st.number_input("Max Ki Limit", min_value=0.01, value=0.5, step=0.1)
    kd_max = st.number_input("Max Kd Limit", min_value=0.001, value=0.1, step=0.01)

kp = synced_slider_input(
    "Kp (Proportional)",
    "kp",
    min_val=0.0,
    max_val=kp_max,
    default_val=0.5,
    step=kp_max / 200.0,  # Auto-scales resolution so the slider is always smooth
    format_str="%.4f",
    container=st.sidebar,
)

ki = synced_slider_input(
    "Ki (Integral)",
    "ki",
    min_val=0.0,
    max_val=ki_max,
    default_val=0.05,
    step=ki_max / 200.0,
    format_str="%.4f",
    container=st.sidebar,
)

kd = synced_slider_input(
    "Kd (Derivative)",
    "kd",
    min_val=0.0,
    max_val=kd_max,
    default_val=0.01,
    step=kd_max / 200.0,
    format_str="%.5f",
    container=st.sidebar,
)

st.sidebar.markdown("---")
st.sidebar.header("Simulation Settings")

profile_type = st.sidebar.selectbox(
    "Profile Type", options=["Trapezoidal", "Step Response"]
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
    target_steps = st.sidebar.slider(
        "Target Steps", min_value=100, max_value=50000, value=10000, step=100
    )
    vel_steps = st.sidebar.number_input("Cruise Velocity (Steps/s)", value=2000)
    accel_steps = st.sidebar.number_input("Acceleration (Steps/s^2)", value=5000)
    max_time = st.sidebar.number_input("Max Sim Time (s)", value=10.0, step=1.0)

    max_vel_rad = vel_steps * RADIANS_PER_STEP
    accel_rad = accel_steps * RADIANS_PER_STEP
    profile = ProfileConfig(
        max_velocity=max_vel_rad, acceleration=accel_rad, deceleration=accel_rad
    )

    sim = Simulator(gains=gains, plant=plant, profile=profile)
    df = sim.run(
        start=0.0, end=target_steps * RADIANS_PER_STEP, dt=dt, max_time=max_time
    )

else:
    step_amplitude = st.sidebar.number_input(
        "Step Amplitude (Steps)", min_value=1, value=10
    )
    wave_period = st.sidebar.number_input(
        "Wave Period (s)", min_value=0.1, value=2.0, step=0.1
    )
    sim_duration = st.sidebar.number_input(
        "Total Sim Duration (s)", min_value=1.0, value=5.0, step=1.0
    )

    sim = Simulator(gains=gains, plant=plant, profile=None)
    amplitude_rad = step_amplitude * RADIANS_PER_STEP

    def square_wave(t: float) -> float:
        return amplitude_rad if (t % wave_period) < (wave_period / 2.0) else 0.0

    df = sim.run_signal(square_wave, duration=sim_duration, dt=dt)

st.sidebar.markdown("---")
st.sidebar.header("View Settings")

lock_axes = st.sidebar.checkbox("Lock X-Axis Window")

if lock_axes:
    # Use columns to put Min and Max next to each other to save vertical space
    col1, col2 = st.sidebar.columns(2)
    with col1:
        x_min = st.number_input("X Min (s)", value=0.0, step=0.1)
    with col2:
        x_max = st.number_input("X Max (s)", value=2.0, step=0.1)
else:
    x_min, x_max = None, None

print("Converting radians to steps for plotting")
df["setpoint"] = df["setpoint"] / RADIANS_PER_STEP
df["actual"] = df["actual"] / RADIANS_PER_STEP
df["error"] = df["error"] / RADIANS_PER_STEP

st.title("Stepper Motor PID Tuner")
initial_setpoint = df["setpoint"].iloc[0]
final_setpoint = df["setpoint"].iloc[-1]
step_size = final_setpoint - initial_setpoint

if abs(step_size) > 1e-6:  # Prevent division by zero on static holds
    if step_size > 0:
        overshoot_val = max(0.0, df["actual"].max() - final_setpoint)
    else:
        overshoot_val = max(0.0, final_setpoint - df["actual"].min())

    overshoot_pct = (overshoot_val / abs(step_size)) * 100.0
else:
    overshoot_val = df["error"].abs().max()
    overshoot_pct = 0.0

tail_length = max(1, int(len(df) * 0.05))
sse_val = df["error"].iloc[-tail_length:].mean()

tolerance = abs(step_size) * 0.02 if abs(step_size) > 1e-6 else 0.02
unsettled_data = df[df["error"].abs() > tolerance]

# If the data never left the tolerance band, or settled immediately
if unsettled_data.empty:
    settling_time = 0.0
else:
    settling_time = unsettled_data["time"].max()

col1, col2, col3 = st.columns(3)

with col1:
    st.metric(
        label="Max Overshoot",
        value=f"{overshoot_val:.4g}",
        delta=f"{overshoot_pct:.1f}%",
        delta_color="inverse",
    )

with col2:
    st.metric(label="Steady-State Error", value=f"{sse_val:.4g}")

with col3:
    st.metric(label="Settling Time (2% Band)", value=f"{settling_time:.3f} s")

fig = plot_interactive_dashboard(df, x_min=x_min, x_max=x_max)
st.plotly_chart(fig, width="stretch")
