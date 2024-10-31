import numpy as np
import matplotlib.pyplot as plt

from scipy.interpolate import CubicSpline

from mpc_controller import MPCController
from path_planning import PathPlanner
from vehicle_model import VehicleModel
from utils import (
    global_to_frenet,
    frenet_to_global,
    find_closest_path_point,
    compute_desired_speed,
    densify_path,
    smooth_path_with_splines,
    detect_turns,
    apply_curvature,
    interpolate_spline,
    plot_path,
    is_target_reached
)
def plot_path(waypoints, new_waypoints, s_fine, x_fine, y_fine):
    plt.figure(figsize=(12, 8))
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Original Waypoints')      # Original waypoints
    plt.plot(new_waypoints[:, 0], new_waypoints[:, 1], 'gx--', label='Rounded Waypoints') # Rounded waypoints
    plt.plot(x_fine, y_fine, 'b-', label='Smoothed Spline Path')            # Smoothed path
    plt.title('Path with Rounded Corners and Cubic Spline Interpolation')
    plt.xlabel('X [meters]')
    plt.ylabel('Y [meters]')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Equal scaling for both axes
    plt.show()

def main():
    debug = True

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
    x_ref = waypoints[:, 0]
    y_ref = waypoints[:, 1]

    plot_path(waypoints, rounded_waypoints, s_fine, x_fine, y_fine)

    # Initialize Path Planner
    path_planner = PathPlanner()
    path_planner.set_path(x_ref, y_ref)
    v_desired = compute_desired_speed(x_ref, y_ref, v_max=15.0, v_min=0.0)
    path_planner.set_speed_profile(v_desired)

    if debug:
        # Plot orientation along the path
        plt.figure(figsize=(10, 4))
        plt.plot(path_planner.s_ref, np.rad2deg(path_planner.theta_ref), 'm-')
        plt.title('Orientation (theta_ref) Along the Path')
        plt.xlabel('Cumulative Distance s [meters]')
        plt.ylabel('Orientation [degrees]')
        plt.grid(True)
        plt.show()

        # Plot curvature along the path
        plt.figure(figsize=(10, 4))
        plt.plot(path_planner.s_ref, path_planner.kappa_ref, 'c-')
        plt.title('Curvature (kappa_ref) Along the Path')
        plt.xlabel('Cumulative Distance s [meters]')
        plt.ylabel('Curvature [1/meters]')
        plt.grid(True)
        plt.show()

    # Initialize Vehicle Model
    vehicle = VehicleModel()
    
    # Initialize MPC Controller
    mpc = MPCController(path_planner, vehicle, N=20, debug=debug)
    
    # Simulation Parameters
    dt = 0.1        # time step

    # Initial Vehicle State (Global Coordinates)
    x0_global = np.array([x_ref[0], y_ref[0], 0.0, 0.0])  # [x, y, theta, v]
    
    d0, psi0, s0 = find_closest_path_point(x0_global, path_planner)

    d0 = float(d0)
    psi0 = float(psi0)
    s0 = float(s0)
    v0 = float(x0_global[3])

    x0_frenet = np.array([d0, psi0, v0, s0])

    # Storage for Simulation Data
    state_history = []
    frenet_state_history = []
    control_history = []
    time_history = []

    tolerance_d = 0.1 
    tolerance_psi = np.deg2rad(1)
    
    current_position = np.array([x0_global[0], x0_global[1]])
    final_position = np.array([x_ref[-1], y_ref[-1]])

    # Initialize Target Waypoint Index
    target_index = 1  # Start with the second waypoint as the first target
    max_iterations = 10000
    iteration = 0
    is_done = False  # Initialize the done flag

    while not is_done and iteration < max_iterations:
        # Current Vehicle State
        x, y, theta, v = x0_global

        # Convert to Frenet Coordinates
        d, psi, s = global_to_frenet(x, y, theta, path_planner)
        frenet_state = np.array([d, psi, v, s])
            
        # Generate Reference Trajectory for MPC
        reference_traj = mpc.generate_reference_trajectory(frenet_state)
        
        # Solve MPC to Get Control Inputs
        delta, a = mpc.solve(frenet_state, reference_traj)    

        # Update Frenet State
        frenet_state_updated = frenet_state + np.array([0, 0, a * dt, v * dt])
        x_new, y_new, theta_new, v_new = frenet_to_global(frenet_state_updated, path_planner)

        # Update Global State
        x0_global = np.array([x_new, y_new, theta_new, v_new])
            
        current_position = np.array([x_new, y_new])

        # Store Data for Visualization
        state_history.append(x0_global.copy())
        frenet_state_history.append(frenet_state_updated.copy())
        control_history.append([delta, a])
        time_history.append(iteration * dt)

        # Check if the current target is reached
        target_position = densified_waypoints[target_index]
        if is_target_reached(current_position, target_position, distance_threshold=1.0):
            print(f"Reached waypoint index {target_index}: {target_position}")
            target_index += 1  # Advance to the next waypoint
            if target_index >= len(densified_waypoints):
                print("All waypoints reached.")
                is_done = True

        iteration += 1

    if iteration >= max_iterations:
        print("Maximum iterations reached. Terminating simulation.")
    
    # Convert Histories to Numpy Arrays for Plotting
    state_history = np.array(state_history)
    frenet_state_history = np.array(frenet_state_history)
    control_history = np.array(control_history)
    time_history = np.array(time_history)
    
    # Visualization
    plot_results(x_ref, y_ref, frenet_state_history, state_history, control_history, time_history)

def plot_results(x_ref, y_ref, frenet_state_history, state_history, control_history, time_history):
    plt.figure(figsize=(10, 6))
    plt.plot(x_ref, y_ref, 'r--', label='Reference Path')
    plt.plot(state_history[:, 0], state_history[:, 1], 'b-', label='Vehicle Path')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.title('Vehicle Trajectory vs Reference Path')
    plt.grid(True)
    plt.show()
    

    # plt.figure()
    # plt.plot(time_history, control_history[:, 0], 'g-', label='Steering Angle [rad]')
    # plt.plot(time_history, control_history[:, 1], 'm-', label='Acceleration [m/s²]')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Control Inputs')
    # plt.legend()
    # plt.title('Control Inputs Over Time')
    # plt.grid(True)
    # plt.show()

    # plt.figure()
    # plt.plot(time_history, frenet_state_history[:, 0], 'b-', label='Lateral Offset [m]')
    # plt.plot(time_history, frenet_state_history[:, 1], 'g-', label='Orientation Error [rad]')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Frenet State variables')
    # plt.legend()
    # plt.title('Frenet State Variables Over Time')
    # plt.grid(True)
    # plt.show()

if __name__ == '__main__':
    main()
