#NMPC using frenet coordinates system

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

"""
    Cartesian Coordinates (CC)
    Frenet Coordinates (FC)
    
    CC states = [x y theta] 
        where theta is heading angle
    CF states = [s d alpha]
        where s is longitudinal postion and d is lateral along the path
        and delta is the difference angle
    

"""

# Define a sinusoidal reference path in global coordinates
def reference_path_sin(t, amplitude=5, frequency=1):
    return np.array([t, amplitude * np.sin(frequency * t)])

# Convert to Frenet frame (s, d) given a reference path
def global_to_frenet(x, y, ref_x, ref_y):
    dx = x - ref_x # difference between current position x and reference postion ref_x
    dy = y - ref_y # difference between current position y and reference postion ref_y
    s = np.sqrt(ref_x**2 + ref_y**2)  # Longitudinal distance along the path (distance along the path)
    d = np.hypot(dx, dy)  # Lateral deviation from the path (distance perpindicular to the path)
    return s, d

# Convert Frenet frame (s, d) back to global (x, y)
def frenet_to_global(s, d, ref_x, ref_y):
    theta = np.arctan2(ref_y, ref_x)  # Heading angle of the reference path
    x = ref_x - d * np.sin(theta)
    y = ref_y + d * np.cos(theta)
    return x, y

# Define CasADi MPC problem with increased penalties and adjusted control effort smoothing
def mpc_frenet_casadi(state, ref_path, ref_theta, horizon):
    
    """
        This sets up the structure for the optimization problem. 
        The goal of MPC is to solve an optimization problem at each control step, 
        finding the optimal set of control inputs that minimize a cost function.    
    """
    opti = ca.Opti()  # Initialize CasADi optimization problem

    # Define variables for control inputs and states
    s = opti.variable(horizon)  # Longitudinal position (s)
    d = opti.variable(horizon)  # Lateral position (d)
    theta = opti.variable(horizon)  # Heading angle (theta)

    a = opti.variable(horizon - 1)  # Acceleration (forward)
    delta = opti.variable(horizon - 1)  # Steering angle

    cost = 0  # Initialize cost function, measures how far the current state 
    #is from the desired state and how much effort it takes to get there
    s0, d0, theta0 = state  # Initial state, used to set constraints for the first step.

    discount_factor = 0.99  #he discount factor gives more importance to near-term steps and less to far-term steps

    # Build the cost function over the prediction horizon
    for i in range(horizon - 1):
        ref_s, ref_d = ref_path[i]
        ref_theta_i = ref_theta[i]
        
        # Add discount factor to future steps, making near-term deviations more important
        discount = discount_factor ** i

        # Penalize deviation from reference path (lateral and longitudinal) and heading
        cost += discount * (10000000 * (s[i] - ref_s)**2 + 10000000 * (d[i] - ref_d)**2 + 10000000 * (theta[i] - ref_theta_i)**2)

        # Penalize for large control effort (steering and acceleration)
        cost += discount * (0.00001 * a[i]**2 + 0.00001 * delta[i]**2)

        # Penalize large changes in control effort for smoother transitions
        if i > 0:
            cost += 25 * (a[i] - a[i-1])**2 + 25 * (delta[i] - delta[i-1])**2  # Stronger penalty on sudden control changes

        # Update the state for the next time step using dynamics
        s[i+1] = s[i] + a[i] * ca.cos(theta[i])  # Update longitudinal position
        d[i+1] = d[i] + a[i] * ca.sin(theta[i])  # Update lateral position
        theta[i+1] = theta[i] + delta[i]  # Update heading

    # Minimize the total cost
    opti.minimize(cost)
    """
    This tells CasADi to minimize the total cost function 
    (which consists of path deviations, control effort, and smoothness penalties). 
    CasADi will try to find the control inputs (a, delta) 
    that minimize the cost over the prediction horizon.    
    """

    # Initial conditions (start from the current state)
    opti.subject_to(s[0] == s0)
    opti.subject_to(d[0] == d0)
    opti.subject_to(theta[0] == theta0)

    # Bounds on acceleration and steering angle
    opti.subject_to(opti.bounded(-4, a, 4))  # Tighter acceleration bounds for less extreme corrections
    opti.subject_to(opti.bounded(-np.pi, delta, np.pi))  # Tighter steering bounds

    # Solve the optimization problem using IPOPT solver
    opti.solver('ipopt')
    sol = opti.solve()

    # Return the first control inputs and predicted states
    return sol.value(a[0]), sol.value(delta[0]), sol.value(s), sol.value(d), sol.value(theta)

# Main function to test the sinusoidal path following
def run_mpc_on_sin_path():
    # Initial state in Frenet coordinates [s, d, theta]
    state = np.array([0.0, 0.0, 0.0])  # Modify this if needed for a better starting alignment

    # Generate a sinusoidal reference path
    t_values = np.linspace(0, 20, 100)
    ref_path_global = np.array([reference_path_sin(t) for t in t_values])

    # Calculate reference heading angles
    ref_theta = np.arctan2(np.gradient(ref_path_global[:, 1]), np.gradient(ref_path_global[:, 0]))

    # Convert reference path to Frenet coordinates
    ref_path_frenet = np.array([global_to_frenet(x, y, 0, 0) for x, y in ref_path_global])

    horizon = 30  # Set horizon for balanced short-term corrections

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
    plt.title(f'MPC Path Following: Sinusoidal Path (Final Adjustments)')
    plt.grid()
    plt.show()

if __name__ == '__main__':
    run_mpc_on_sin_path()
