import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Constants for vehicle dynamics
dt = 0.1  # Time step
L = 2.0   # Wheelbase length

# Kinematic Bicycle Model
def vehicle_model(state, u, dt):
    x, y, theta = state
    v, delta = u  # velocity and steering angle

    # Update the state based on the bicycle model equations
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += v / L * np.tan(delta) * dt
    return np.array([x, y, theta])

# MPC objective function
def mpc_objective(u, state, ref_path):
    cost = 0.0
    for i in range(len(ref_path)):
        # Apply control inputs to simulate vehicle motion
        state = vehicle_model(state, u[i * 2:(i + 1) * 2], dt)
        # Calculate the cost as the distance from the reference path
        cost += np.linalg.norm(state[:2] - ref_path[i])
    return cost

# A* Pathfinding (for simplicity, generate a straight path here)
def generate_astar_path(start, goal, steps):
    path_x = np.linspace(start[0], goal[0], steps)
    path_y = np.linspace(start[1], goal[1], steps)
    path = np.vstack((path_x, path_y)).T
    return path

# Visualization function
def plot_vehicle_path(ref_path, vehicle_path):
    plt.figure()
    plt.plot(ref_path[:, 0], ref_path[:, 1], 'g--', label='Reference Path')
    plt.plot(vehicle_path[:, 0], vehicle_path[:, 1], 'b-', label='Vehicle Path')
    plt.scatter(ref_path[:, 0], ref_path[:, 1], c='g')
    plt.scatter(vehicle_path[:, 0], vehicle_path[:, 1], c='b')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('MPC Path Following')
    plt.grid()
    plt.show()

# Main function
def main():
    # Initial state of the vehicle [x, y, theta]
    state = np.array([0.0, 0.0, np.pi / 4])

    # Generate a reference path using A* (for simplicity, straight line path)
    start = [0, 0]
    goal = [10, 10]
    steps = 50
    ref_path = generate_astar_path(start, goal, steps)
    
    # Optimization (MPC) horizon
    horizon = 10  # Number of steps to predict
    
    # Initialize variables to store vehicle path
    vehicle_path = []
    
    # Loop through the entire reference path
    for t in range(len(ref_path) - horizon):
        # Select the current horizon of the reference path
        ref_horizon = ref_path[t:t + horizon]
        
        # Set initial guess for control inputs (velocity and steering)
        u0 = np.zeros(horizon * 2)  # [v1, delta1, v2, delta2, ...]

        # Bounds for control inputs (velocity and steering angle)
        bounds = [(0.0, 2.0)] * horizon + [(-max_steering, max_steering)] * horizon

        # Minimize the cost using MPC
        res = minimize(mpc_objective, u0, args=(state, ref_horizon), bounds=bounds)

        # Apply the first control input to the vehicle model
        u_opt = res.x[:2]  # First velocity and steering angle
        state = vehicle_model(state, u_opt, dt)
        vehicle_path.append(state[:2])

    # Convert vehicle path to numpy array for plotting
    vehicle_path = np.array(vehicle_path)

    # Plot the results
    plot_vehicle_path(ref_path, vehicle_path)

if __name__ == '__main__':
    main()
