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
    s = np.sqrt(ref_x**2 + ref_y**2)  # Longitudinal distance along the path
    d = np.hypot(dx, dy)  # Lateral deviation from the path
    return s, d

# Convert Frenet frame (s, d) back to global (x, y)
def frenet_to_global(s, d, ref_x, ref_y):
    theta = np.arctan2(ref_y, ref_x)  # Heading angle of the reference path
    x = ref_x - d * np.sin(theta)
    y = ref_y + d * np.cos(theta)
    return x, y

# Define CasADi MPC problem with adjustments to penalties and horizon tuning
def mpc_frenet_casadi(state, ref_path, ref_theta, horizon):
    opti = ca.Opti()  # Initialize CasADi optimization problem

    # Define variables for control inputs and states
    s = opti.variable(horizon)  # Longitudinal position (s)
    d = opti.variable(horizon)  # Lateral position (d)
    theta = opti.variable(horizon)  # Heading angle (theta)

    a = opti.variable(horizon - 1)  # Acceleration (forward)
    delta = opti.variable(horizon - 1)  # Steering angle

    cost = 0  # Initialize cost function
    s0, d0, theta0 = state  # Initial state

    discount_factor = 0.95  # Discount factor, tuned to balance short-term and long-term planning

    # Build the cost function over the prediction horizon
    for i in range(horizon - 1):
        ref_s, ref_d = ref_path[i]
        ref_theta_i = ref_theta[i]
        
        # Add discount factor to future steps, making near-term deviations more important
        discount = discount_factor ** i

        # Penalize deviation from reference path (lateral and longitudinal) and heading
        cost += discount * (80 * (s[i] - ref_s)**2 + 90 * (d[i] - ref_d)**2 + 70 * (theta[i] - ref_theta_i)**2)

        # Penalize for large control effort (steering and acceleration)
        cost += discount * (0.1 * a[i]**2 + 0.1 * delta[i]**2)

        # Update the state for the next time step using dynamics
        s[i+1] = s[i] + a[i] * ca.cos(theta[i])  # Update longitudinal position
        d[i+1] = d[i] + a[i] * ca.sin(theta[i])  # Update lateral position
        theta[i+1] = theta[i] + delta[i]  # Update heading

    # Minimize the total cost
    opti.minimize(cost)

    # Initial conditions (start from the current state)
    opti.subject_to(s[0] == s0)
    opti.subject_to(d[0] == d0)
    opti.subject_to(theta[0] == theta0)

    # Bounds on acceleration and steering angle
    opti.subject_to(opti.bounded(-3, a, 3))  # Acceleration limits [-3, 3] for more responsive control
    opti.subject_to(opti.bounded(-np.pi / 2, delta, np.pi / 2))  # Steering limits [-pi/2, pi/2] for more flexible steering

    # Solve the optimization problem using IPOPT solver
    opti.solver('ipopt')
    sol = opti.solve()

    # Return the first control inputs and predicted states
    return sol.value(a[0]), sol.value(delta[0]), sol.value(s), sol.value(d), sol.value(theta)

# Main function for Frenet-based MPC with CasADi and further refined penalties
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
    horizon = 15  # Extended horizon for smoother control over a longer distance

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
    plt.title('MPC Path Following in Frenet Coordinates with CasADi (Further Refined)')
    plt.grid()
    plt.show()

if __name__ == '__main__':
    main()
