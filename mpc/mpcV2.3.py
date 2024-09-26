#NMPC using bicycle model

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

# Define a sinusoidal reference path in global coordinates
def reference_path_sin(t, amplitude=5, frequency=1):
    return np.array([t, amplitude * np.sin(frequency * t)])

# Bicycle Model Dynamics for state update
def bicycle_model_dynamics(state, a, delta, dt=0.1):
    # Bicycle model parameters
    Lf = 2.5  # Distance from center to front axle
    beta = np.arctan2((Lf / (Lf + Lf)) * np.tan(delta), 1)  # Slip angle
    s = state[0] + a * np.cos(state[2] + beta) * dt  # Longitudinal position
    d = state[1] + a * np.sin(state[2] + beta) * dt  # Lateral position
    theta = state[2] + (a / Lf) * np.sin(beta) * dt  # Heading angle
    return np.array([s, d, theta])

# Convert to Frenet frame (s, d) given a reference path
def global_to_frenet(x, y, ref_x, ref_y):
    dx = x - ref_x
    dy = y - ref_y
    s = np.sqrt(ref_x**2 + ref_y**2)
    d = np.hypot(dx, dy)
    return s, d

# Convert Frenet frame (s, d) back to global (x, y)
def frenet_to_global(s, d, ref_x, ref_y):
    theta = np.arctan2(ref_y, ref_x)
    x = ref_x - d * np.sin(theta)
    y = ref_y + d * np.cos(theta)
    return x, y

# Corrected adaptive penalty function using CasADi
def adaptive_penalty(error):
    return ca.if_else(ca.fabs(error) > 1.0, 300, 100)

# Define CasADi MPC problem with Bicycle Model and adaptive cost function
def mpc_frenet_casadi(state, ref_path, ref_theta, horizon):
    opti = ca.Opti()  # Initialize CasADi optimization problem

    # Define variables for control inputs and states
    s = opti.variable(horizon)
    d = opti.variable(horizon)
    theta = opti.variable(horizon)

    a = opti.variable(horizon - 1)
    delta = opti.variable(horizon - 1)

    cost = 0  # Initialize cost function
    s0, d0, theta0 = state

    discount_factor = 0.85  # Reduced discount factor for long-term planning

    # Build the cost function over the prediction horizon
    for i in range(horizon - 1):
        ref_s, ref_d = ref_path[i]
        ref_theta_i = ref_theta[i]
        
        discount = discount_factor ** i

        # Penalize deviation from reference path and heading with adaptive penalties
        cost += discount * (adaptive_penalty(s[i] - ref_s) * (s[i] - ref_s)**2 + 
                            adaptive_penalty(d[i] - ref_d) * (d[i] - ref_d)**2 + 
                            300 * (theta[i] - ref_theta_i)**2)

        # Penalize large control effort (steering and acceleration)
        cost += discount * (10 * a[i]**2 + 10 * delta[i]**2)

        # Penalize large changes in control effort for smoother transitions
        if i > 0:
            cost += 100 * (a[i] - a[i-1])**2 + 100 * (delta[i] - delta[i-1])**2

        # Update the state for the next time step using bicycle model dynamics
        state_next = bicycle_model_dynamics([s[i], d[i], theta[i]], a[i], delta[i])
        s[i+1] = state_next[0]
        d[i+1] = state_next[1]
        theta[i+1] = state_next[2]

    # Add terminal constraint to ensure the final state reaches the desired value
    cost += 2000 * (s[-1] - ref_s)**2 + 2000 * (d[-1] - ref_d)**2  # Stronger terminal constraint

    # Minimize the total cost
    opti.minimize(cost)

    # Initial conditions
    opti.subject_to(s[0] == s0)
    opti.subject_to(d[0] == d0)
    opti.subject_to(theta[0] == theta0)

    # Bounds on acceleration and steering angle
    opti.subject_to(opti.bounded(-6, a, 6))
    opti.subject_to(opti.bounded(-np.pi, delta, np.pi))

    # Solve the optimization problem using IPOPT solver
    opti.solver('ipopt')
    sol = opti.solve()

    # Return the first control inputs and predicted states
    return sol.value(a[0]), sol.value(delta[0]), sol.value(s), sol.value(d), sol.value(theta)

# Main function to test the sinusoidal path following
def run_mpc_on_sin_path():
    # Initial state in Frenet coordinates [s, d, theta]
    state = np.array([0.0, 0.0, 0.0])

    # Generate a sinusoidal reference path
    t_values = np.linspace(0, 20, 100)
    ref_path_global = np.array([reference_path_sin(t) for t in t_values])

    # Calculate reference heading angles
    ref_theta = np.arctan2(np.gradient(ref_path_global[:, 1]), np.gradient(ref_path_global[:, 0]))

    # Convert reference path to Frenet coordinates
    ref_path_frenet = np.array([global_to_frenet(x, y, 0, 0) for x, y in ref_path_global])

    horizon = 70  # Increased horizon for better long-term planning

    # Initialize variables to store the vehicle path
    vehicle_path = []

    # Loop through the entire reference path
    for t in range(len(ref_path_frenet) - horizon):
        ref_horizon = ref_path_frenet[t:t + horizon]
        ref_horizon_theta = ref_theta[t:t + horizon]

        a_opt, delta_opt, s_opt, d_opt, theta_opt = mpc_frenet_casadi(state, ref_horizon, ref_horizon_theta, horizon)

        # Update the state using bicycle model dynamics
        state = bicycle_model_dynamics(state, a_opt, delta_opt)

        # Convert back to global coordinates for visualization
        vehicle_position_global = frenet_to_global(state[0], state[1], ref_path_global[t][0], ref_path_global[t][1])
        vehicle_path.append(vehicle_position_global)

    vehicle_path = np.array(vehicle_path)

    # Plot the reference path and vehicle path
    plt.figure()
    plt.plot(ref_path_global[:, 0], ref_path_global[:, 1], 'g--', label='Reference Path')
    plt.plot(vehicle_path[:, 0], vehicle_path[:, 1], 'b-', label='Vehicle Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title(f'MPC Path Following: Sinusoidal Path (Final Adjustments)')
    plt.grid()
    plt.show()

if __name__ == '__main__':
    run_mpc_on_sin_path()
