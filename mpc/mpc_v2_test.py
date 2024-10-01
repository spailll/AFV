import matplotlib.pyplot as plt
import numpy as np
import time
from mpc_v2 import MPC
from vehicle import Vehicle
from utils import densify_path, detect_turns, apply_curvature, interpolate_spline, plot_path


waypoints = np.array([
    [0.0, 0.0],
    [10.0, 0.0],
    [10.0, 10.0],
    [20.0, 10.0],
    [20.0, 0.0],
    [30.0, 0.0],
    [30.0, 10.0],
    [40.0, 10.0],
    [40.0, 20.0],
    [50.0, 20.0],
    [50.0, 0.0]
])

densified_waypoints = densify_path(waypoints, 1.0)

turn_indices = detect_turns(densified_waypoints, angle_threshold_deg=45)

rounded_waypoints = apply_curvature(densified_waypoints, turn_indices, radius=100, num_curve_points=100)

s_fine, x_fine, y_fine, cs_x, cs_y = interpolate_spline(rounded_waypoints, num_fine_points=1000)

x_ref = rounded_waypoints[:, 0]
y_ref = rounded_waypoints[:, 1]


plot_path(waypoints, densified_waypoints, rounded_waypoints, x_ref, y_ref)

vehicle = Vehicle()

# issue_ssh_command(0, 90)
# issue_ssh_command(1, 90)
# issue_ssh_command(2, 97)

# Parameters
N = 20
dt = 0.1
v = 10.0

steps = 100000

# Initialize the MPC controller
mpc = MPC(N, dt)

# Reference trajectory
t_ref = np.arange(0, 1000+ N * dt, dt)
# x_ref = v * t_ref
# y_ref = np.sin(0.05 * x_ref)
theta_ref = np.arctan2(np.gradient(y_ref), np.gradient(x_ref))
ref_states = np.vstack((x_ref, y_ref, theta_ref)).T

# Initial state
x0 = np.array([0, 0, 0]) # [x, y, theta]

# Initial control input guess
u0 = np.zeros(N)

# Storage for simulation results
x_hist = []
u_hist = []
t_hist = []

for i in range(steps):
    current_time = i * dt

    # Reference trajectory for current time window
    start_idx = i
    end_idx = start_idx + N
    if end_idx >= len(ref_states):
        end_idx = len(ref_states) - 1
    xs = ref_states[start_idx:end_idx]

    # If the reference trajectory is shorter than N, pad with the last point
    if xs.shape[0] < N:
        last_row = xs[-1]
        xs = np.vstack((xs, np.tile(last_row, (N - xs.shape[0], 1))))

    if xs.size <= i - 1:
        break
    # Solve the MPC Problem
    u_opt, x_pred = mpc.solve_mpc(x0, xs, u0)

    # Apply the first control input
    delta_opt = u_opt[0]
    u0 = u_opt

    print(delta_opt)

    vehicle.move(0.2, delta_opt)

    # Simulate the vehicle with applied control input
    f_value = mpc.f(x0, delta_opt)
    x0 = x0 + dt * np.array(f_value.full()).flatten()

    # Store the results
    x_hist.append(x0)
    u_hist.append(delta_opt)
    t_hist.append(current_time)

    time.sleep(dt)

# Convert the results to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
t_hist = np.array(t_hist)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(ref_states[:, 0], ref_states[:, 1], 'r--', label='Reference Path')
plt.plot(x_hist[:, 0], x_hist[:, 1], 'b-', label='Vehicle Path')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.title('Vehicle Trajectory v Reference Trajectory')
plt.grid(True)
plt.show()

plt.figure()
plt.plot(t_hist, u_hist, 'b-', label='Control Input')
plt.xlabel('Time [s]')
plt.ylabel('Steering Angle [rad]')
plt.title('Steering Angle vs Time')
plt.grid(True)
plt.show()
