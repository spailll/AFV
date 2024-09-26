import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

# Define a simple circular reference path in global coordinates
def reference_path(t):
    return np.array([10 * np.cos(t), 10 * np.sin(t)])

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

# Define CasADi MPC problem
def mpc_frenet_casadi(state, ref_path, ref_theta, horizon):
    opti = ca.Opti()

    # Define variables for control inputs and states
    s = opti.variable(horizon)
    d = opti.variable(horizon)
    theta = opti.variable(horizon)

    a = opti.variable(horizon - 1)  # Acceleration
    delta = opti.variable(horizon - 1)  # Steering angle

    cost = 0
    s0, d0, theta0 = state

    # Loop through horizon to define the cost function
    for i in range(horizon - 1):
        ref_s, ref_d = ref_path[i]
        ref_theta_i = ref_theta[i]

        # Penalize deviation from reference path
        cost += 20 * (s[i] - ref_s)**2 + 25 * (d[i] - ref_d)**2 + 15 * (theta[i] - ref_theta_i)**2
        
        # Penalize for control effort (steering and acceleration)
        cost += 0.05 * a[i]**2 + 0.05 * delta[i]**2

        # Update the state for next time step using dynamics
        s[i+1] = s[i] + a[i] * ca.cos(theta[i])
        d[i+1] = d[i] + a[i] * ca.sin(theta[i])
        theta[i+1] = theta[i] + delta[i]

    opti.minimize(cost)

    # Initial conditions
    opti.subject_to(s[0] == s0)
    opti.subject_to(d[0] == d0)
    opti.subject_to(theta[0] == theta0)

    # Bounds on acceleration and steering angle
    opti.subject_to(opti.bounded(-2, a, 2))  # Acceleration bounds
    opti.subject_to(opti.bounded(-np.pi / 4, delta, np.pi / 4))  # Steering bounds

    # Solve the problem
    opti.solver('ipopt')  # Use IPOPT solver
    sol = opti.solve()

    return sol.value(a[0]), sol.value(delta[0]), sol.value(s), sol.value(d), sol.value(theta)

# Main function for Frenet-based MPC with CasADi
def main():
    # Initial state in Frenet coordinates [s, d, theta]
    state = np.array([0.0, 0.0, 0.0])

    # Generate a circular reference path in global coordinates
    t_values = np.linspace(0, 2 * np.pi, 50)
    ref_path_global = np.array([reference_path(t) for t in t_values])

    # Calculate reference heading angles
    ref_theta = np.arctan2(np.gradient(ref_path_global[:, 1]), np.gradient(ref_path_global[:, 0]))

    # Convert reference path to Frenet coordinates
    ref_path_frenet = np.array([global_to_frenet(x, y, 0, 0) for x, y in ref_path_global])

    # MPC horizon
    horizon = 10

    # Initialize variables to store the vehicle path
    vehicle_path = []

    # Loop through the entire reference path
    for t in range(len(ref_path_frenet) - horizon):
        # Select the current horizon of the reference path
        ref_horizon = ref_path_frenet[t:t + horizon]
        ref_horizon_theta = ref_theta[t:t + horizon]

        # Solve the MPC optimization using CasADi
        a_opt, delta_opt, s_opt, d_opt, theta_opt = mpc_frenet_casadi(state, ref_horizon, ref_horizon_theta, horizon)

        # Apply the first control input to update the vehicle's state
        state[0] += a_opt  # Update s (longitudinal progress)
        state[1] += delta_opt  # Update d (lateral offset)
        state[2] += delta_opt  # Update theta (heading)

        # Convert back to global coordinates for visualization
        vehicle_position_global = frenet_to_global(state[0], state[1], ref_path_global[t][0], ref_path_global[t][1])
        vehicle_path.append(vehicle_position_global)

    # Convert vehicle path to numpy array for plotting
    vehicle_path = np.array(vehicle_path)

    # Plot the reference path and vehicle path
    plt.figure()
    plt.plot(ref_path_global[:, 0], ref_path_global[:, 1], 'g--', label='Reference Path')
    plt.plot(vehicle_path[:, 0], vehicle_path[:, 1], 'b-', label='Vehicle Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('MPC Path Following in Frenet Coordinates with CasADi')
    plt.grid()
    plt.show()

if __name__ == '__main__':
    main()
