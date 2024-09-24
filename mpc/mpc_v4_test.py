import matplotlib.pyplot as plt
import numpy as np
import casadi as ca 
from scipy.interpolate import interp1d
from mpc_v4 import MPC

def compute_desired_speed(x_ref, y_ref, v_max, v_min):
    # Calculate derivatives
    dx = np.gradient(x_ref)
    dy = np.gradient(y_ref)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    # Calculate curvature
    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    curvature = np.nan_to_num(curvature)

    # Normalize curvature
    curvature_max = np.max(curvature) if np.max(curvature) != 0 else 1

    # Define speed-curvature relationship
    v_des = v_max * (1 - curvature / curvature_max)
    v_des = np.clip(v_des, v_min, v_max)

    return v_des

def compute_frenet_state(x, y, theta, x_ref, y_ref, s_ref):
    # Compute the closest point on the path
    dx = x_ref - x
    dy = y_ref - y
    dist_squared = dx**2 + dy**2
    min_idx = np.argmin(dist_squared)

    # Corresponding s value
    s = s_ref[min_idx]

    # Path heading at that point
    path_theta = np.arctan2(np.gradient(y_ref)[min_idx], np.gradient(x_ref)[min_idx])
    
    # Lateral deviation
    d = (x - x_ref[min_idx]) * (-np.sin(path_theta)) + (y - y_ref[min_idx]) * np.cos(path_theta)

    # Orientation error
    psi = theta - path_theta
    psi = (psi + np.pi) % (2 * np.pi) - np.pi # Normalize to [-pi, pi]

    return d, psi, s

# Parameters
N = 20
dt = 0.1
v = 10.0

# Simulation parameters
sim_time = 20 # seconds
steps = int(sim_time / dt)

# Reference trajectory
t_ref = np.arange(0, sim_time + N * dt, dt)
x_ref = v * t_ref
y_ref = np.sin(0.05 * x_ref)

# Compute c umulative arc length along the path
dx_ref = np.diff(x_ref)
dy_ref = np.diff(y_ref)
theta_ref = np.arctan2(dy_ref, dx_ref)                      
theta_ref = np.concatenate((theta_ref, [theta_ref[-1]]))    
ds_ref = np.hypot(dx_ref, dy_ref)
s_ref = np.concatenate(([0], np.cumsum(ds_ref)))

# Interpolate x(s) and y(s)
x_s = interp1d(s_ref, x_ref, kind='cubic', fill_value='extrapolate')
y_s = interp1d(s_ref, y_ref, kind='cubic', fill_value='extrapolate')

# Compute curvature kappa(s)
s_vals = s_ref
x_s_vals = x_s(s_vals)
y_s_vals = y_s(s_vals)

x_s_prime = np.gradient(x_s_vals, s_vals)
y_s_prime = np.gradient(y_s_vals, s_vals)

x_s_prime_prime = np.gradient(x_s_prime, s_vals)
y_s_prime_prime = np.gradient(y_s_prime, s_vals)

kappa_ref = (x_s_prime * y_s_prime_prime - y_s_prime * x_s_prime_prime) / (x_s_prime**2 + y_s_prime**2)**1.5
kappa_ref = np.nan_to_num(kappa_ref)

poly_degree = 5
kappa_coeffs = np.polyfit(s_vals, kappa_ref, deg=poly_degree)
# Creaet CasADi-compatible variables
# s_vals_casadi = ca.DM(s_vals)
# kappa_ref_casadi = ca.DM(kappa_ref)

# Initialize the MPC controller
mpc = MPC(N, dt, kappa_coeffs)

# Desired speed profile
v_desired = compute_desired_speed(x_ref, y_ref, mpc.v_max, mpc.v_min)

# Initial state in global coordinates
x0_global = np.array([x_ref[0], y_ref[0], 0.0, 0.0])

# Compute initial Frenet state
d0, psi0, s0 = compute_frenet_state(x0_global[0], x0_global[1], x0_global[2], x_ref, y_ref, s_ref)
x0_frenet = np.array([d0, psi0, x0_global[3], s0])

# Initial control input guess
u0 = np.zeros((mpc.N, mpc.NU))

# Storage for simulation results
x_hist = []
u_hist = []
t_hist = []

for i in range(steps):
    current_time = i * dt

    # Compute Frenet state
    d, psi, s = compute_frenet_state(x0_global[0], x0_global[1], x0_global[2], x_ref, y_ref, s_ref)
    x0_frenet = np.array([d, psi, x0_global[3], s])

    # Reference speed for current prediction horizon
    s_future = s + np.cumsum([dt * x0_frenet[2]] * mpc.N)
    v_ref_horizon = np.interp(s_future, s_ref, v_desired)
    v_ref_horizon = np.clip(v_ref_horizon, mpc.v_min, mpc.v_max)

    x_ref_traj = np.zeros((mpc.NX, mpc.N))
    x_ref_traj[0, :] = 0
    x_ref_traj[1, :] = 0
    x_ref_traj[2, :] = v_ref_horizon
    x_ref_traj[3, :] = s_future

    # Solve the MPC Problem
    u_opt, x_pred = mpc.solve_mpc(x0_frenet, x_ref_traj, u0)

    # Apply the first control input
    u_opt = u_opt.reshape(mpc.N, mpc.NU)
    delta_opt = u_opt[0, 0]
    a_opt = u_opt[0, 1]

    # Update control input for next iteration
    u0 = np.concatenate((u_opt[1:], u_opt[-1:]))

    # Simulate the vehicle with applied control input
    f_value = mpc.f(x0_frenet, np.array([delta_opt, a_opt]))
    x0_frenet += dt * np.array(f_value.full()).flatten()

    # Convert Frenet state to global coordinates
    s = x0_frenet[3]
    x_path = np.interp(s, s_ref, x_ref)
    y_path = np.interp(s, s_ref, y_ref)
    # path_theta = np.arctan2(np.gradient(y_ref)[min_idx], np.gradient(x_ref)[min_idx])
    theta_path = np.interp(s, s_ref, theta_ref)

    # Calculate global position
    x = x_path - x0_frenet[0] * np.sin(theta_path)
    y = y_path + x0_frenet[0] * np.cos(theta_path)
    theta = theta_path + x0_frenet[1]

    # Update global state
    x0_global = np.array([x, y, theta, x0_frenet[2]])

    # Store the results
    x_hist.append(x0_global.copy())
    u_hist.append([delta_opt, a_opt])
    t_hist.append(current_time)

# Convert the results to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
t_hist = np.array(t_hist)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(x_ref, y_ref, 'r--', label='Reference Path')
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

plt.figure()
plt.plot(s_ref, v_desired, 'r-', label='Desired Speed')
plt.xlabel('Arc Length [m]')
plt.ylabel('Speed [m/s]')
plt.title('Desired Speed Profile')
plt.grid(True)
plt.legend()
plt.show()

