import matplotlib.pyplot as plt
import numpy as np
from mpc_v3 import MPC, compute_desired_speed

# Parameters
N = 20
dt = 0.1
v = 10.0

# Simulation parameters
sim_time = 20 # seconds
steps = int(sim_time / dt)

# Initialize the MPC controller
mpc = MPC(N, dt)

# Reference trajectory
t_ref = np.arange(0, sim_time + N * dt, dt)
x_ref = v * t_ref
y_ref = np.sin(0.05 * x_ref)
theta_ref = np.arctan2(np.gradient(y_ref), np.gradient(x_ref))
v_desired = compute_desired_speed(x_ref, y_ref, mpc.v_max, mpc.v_min)
ref_states = np.vstack((x_ref, y_ref, theta_ref, v_desired)).T

# Initial state
# x0 = np.array([x_ref[0], y_ref[0], theta_ref[0], v_desired[0]]) # [x, y, theta]
x0 = np.array([0.0, 0.0, 0.0, 0.0]) # [x, y, theta]

# Initial control input guess
u0 = np.zeros((mpc.N, mpc.NU))

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

    # Solve the MPC Problem
    u_opt, x_pred = mpc.solve_mpc(x0, xs, u0)

    # Apply the first control input
    delta_opt = u_opt[0]
    a_opt = u_opt[1]

    # Update control input for next iteration
    u0 = np.concatenate((u_opt[mpc.NU:], u_opt[-mpc.NU:]))

    # Simulate the vehicle with applied control input
    f_value = mpc.f(x0, np.array([delta_opt, a_opt]))
    x0 = x0 + dt * np.array(f_value.full()).flatten()

    # Store the results
    x_hist.append(x0)
    u_hist.append([delta_opt, a_opt])
    t_hist.append(current_time)

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

plt.figure()
plt.plot(t_hist, x_hist[:, 3], 'b-', label='Vehicle Speed')
plt.plot(t_hist, v_desired[:len(t_hist)], 'r--', label='Desired Speed')
plt.xlabel('Time [s]')
plt.ylabel('Speed [m/s]')
plt.legend()
plt.title('Vehicle Speed over Time')
plt.grid(True)
plt.show()
