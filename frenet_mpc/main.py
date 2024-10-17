# main.py

import numpy as np
import matplotlib.pyplot as plt
import subprocess

from mpc_controller import MPCController
from path_planning import PathPlanner
from vehicle_model import VehicleModel
from utils import (
    global_to_frenet,
    frenet_to_global,
    find_closest_path_point,
    compute_desired_speed,
    is_target_reached
)


def main():

    # Initialize Path Planner
    path_planner = PathPlanner()

    # Generate Reference Path
    x_ref = np.linspace(0, 100, 1000)
    y_ref = np.sin(x_ref * 0.5)

    path_planner.set_path(x_ref, y_ref)
    v_desired = compute_desired_speed(x_ref, y_ref, v_max=15.0, v_min=0.0)
    path_planner.set_speed_profile(v_desired)

    # Initialize Vehicle Model
    vehicle = VehicleModel()
    
    # Initialize MPC Controller
    mpc = MPCController(path_planner, vehicle, N=8)
    
    # Simulation Parameters
    sim_time = 20  # seconds
    dt = 0.1        # time step
    steps = int(sim_time / dt)
    
    target_position = np.array([x_ref[-1], y_ref[-1]])


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
    max_iterations = 1000

    is_done = False

    # for step in range(steps):
    i = 0
    while not is_done:
        current_time = i * dt
        # current_time = step * dt

        x, y, theta, v = x0_global

        d, psi, s = global_to_frenet(x, y, theta, path_planner)
        frenet_state = np.array([d, psi, v, s])
            
        # Generate Reference Trajectory for MPC
        reference_traj = mpc.generate_reference_trajectory(frenet_state)
        
        # Solve MPC to Get Control Inputs
        delta, a = mpc.solve(frenet_state, reference_traj)    

        frenet_state_updated = np.array([d, psi, v, s]) + np.array([0, 0, a * dt, v * dt])
        x_new, y_new, theta_new, v_new = frenet_to_global(frenet_state_updated, path_planner)

        x0_global = np.array([x_new, y_new, theta_new, v_new])

        # Store Data for Visualization
        state_history.append(x0_global.copy())
        frenet_state_history.append(frenet_state_updated.copy())
        control_history.append([delta, a])
        time_history.append(current_time)

        current_position = [x_new, y_new]
        is_done = is_target_reached(current_position, target_position, distance_threshold=1.0)
        if i >= max_iterations: 
            break
        i += 1

        ############ USE a AND delta TO ASSIGN CONTROLS TO HARDWHERE HERE #####
        set_controls(a, v, dt, delta, )
        # ssh_command = ["ssh", "pi@10.42.0.75", ]

    # Convert Histories to Numpy Arrays for Plotting
    state_history = np.array(state_history)
    frenet_state_history = np.array(frenet_state_history)
    control_history = np.array(control_history)
    time_history = np.array(time_history)
    
    # Visualization
    plot_results(x_ref, y_ref, frenet_state_history, state_history, control_history, time_history)

def set_controls(acceleration, current_speed, dt, delta, max_speed=15, max_steering_angle=30):
    set_steering(delta)
    set_speed(acceleration, current_speed, dt, delta)

def set_steering(delta, max_angle=30):
    delta = max(-max_angle, min(max_angle, delta))
    angle = 90 + delta * 90 
    issue_ssh_command(2, angle)

def set_speed(acceleration, current_speed, dt, delta, max_speed=15, max_steering_angle=30):
    new_speed = current_speed + acceleration * dt
    new_speed = max(0, min(max_speed, new_speed))
    steering_ratio = delta / max_steering_angle
    
    left_speed = new_speed * (1 + steering_ratio)
    right_speed = new_speed * (1 - steering_ratio)

    left_speed = 90 + max(0, min(max_speed, left_speed)) * 35 / max_speed 
    right_speed = 90 + max(0, min(max_speed, right_speed)) * 35 / max_speed 



    issue_ssh_command(0, left_speed)
    issue_ssh_command(1, right_speed)

def issue_ssh_command(servo_num, degree):
    ssh_command = ["ssh", "pi@10.42.0.75", "./servo3.py", str(servo_num), str(degree)]
    print(ssh_command)
    # try:
    #     result = subprocess.run(ssh_command, check=True, capture_output=True, text=True)
    # except subprocess.CalledProcessError as e:
    #     print("Error executing servo command: ")
    #     print(e.stderr)
    # except FileNotFoundError:
    #     print("SSH command not found")



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
    

    plt.figure()
    plt.plot(time_history, control_history[:, 0], 'g-', label='Steering Angle [rad]')
    plt.plot(time_history, control_history[:, 1], 'm-', label='Acceleration [m/s²]')
    plt.xlabel('Time [s]')
    plt.ylabel('Control Inputs')
    plt.legend()
    plt.title('Control Inputs Over Time')
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.plot(time_history, frenet_state_history[:, 0], 'b-', label='Lateral Offset [m]')
    plt.plot(time_history, frenet_state_history[:, 1], 'g-', label='Orientation Error [rad]')
    plt.xlabel('Time [s]')
    plt.ylabel('Frenet State variables')
    plt.legend()
    plt.title('Frenet State Variables Over Time')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()