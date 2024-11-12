# mppic.py

import numpy as np
import matplotlib.pyplot as plt
import time
import serial
from threading import Thread, Event
from Mavlink import MAVLink
from vehicle import Vehicle

# CALLSIGN = 'YOURCALSGN'     # Not necessary for RF controlled Vehicles, but still advised

# emergency_stop = False

# def recieve_messages(mav, stop_event):
#     global emergency_stop
#     while not stop_event.is_set():
#         msg = mav.recv_match(blocking=True)
#         if not msg:
#             continue

#         msg_type = msg.get_type()

#         elif msg_type == 'EMERGENCY_STOP':
#             handle_emergency_stop(msg)

# def handle_emergency_stop(msg):
#     global emergency_stop
#     emergency_stop = True

# def send_current_location(mav, x, y):
#     msg = mav.MAVLink_current_location_message(x_coordinate=x, y_coordinate=y, callsign=CALLSIGN)
#     mav.send(msg)
#     print(f"Sent current location: X={x}, Y={y}")

class RMPPIController:
    def __init__(self, path_x, path_y, mav, wheel_base=0.72898, dt=0.25, lambda_=1, N=500):
        self.path_x = path_x
        self.path_y = path_y
        self.wheel_base = wheel_base
        self.dt = dt
        self.lambda_ = lambda_    # Temperature parameter for MPPI
        self.N = N                # Number of control sequences
        self.state_real = np.array([0.0, 0.0, 0.0, 0.0])  # Real system state: [x, y, theta, v]
        self.vehicle = Vehicle()
        self.control_mean = np.array([5.0, np.radians(0.0)])  # Initial control input mean: [v, delta]
        self.previous_delta = 0.0  # Previous steering angle (radians)

        # Cost function weights
        self.w_position = 200.0
        self.w_orientation = 100.0
        self.w_control = 1.0
        self.w_acceleration = 0.01
        self.w_speed = 20.0
        self.w_high_speed = 500.0
        self.w_low_speed = 500.0
        self.w_speed_deviation = 1.0
        self.w_deviation = 500.0
        self.w_heading = 200.0
        self.w_cumulative_heading = 200.0
        self.w_backward = 1000.0

        # Vehicle parameters
        self.max_acceleration = 1.0  # Maximum acceleration (m/s^2)
        self.max_steering_rate = np.radians(10)  # Maximum steering rate (rad/s)
        self.max_deviation = 1.0  # Maximum deviation from path (meters)
        self.max_speed = 2.0 # Maximum speed (m/s)
        self.desired_speed = 1.5  # Desired cruising speed (m/s)
        self.min_speed = 1.0  # Minimum speed (m/s)
        
        # Prediction horizon parameters
        self.T = 15  # Prediction horizon steps
        self.T_min = 10
        self.T_max = 20
        self.scaling_factor_T = 1.1  # For adjusting T based on speed


        # Lookahead distance parameters
        self.base_lookahead = 3.0  # Base lookahead distance (meters)
        self.scaling_factor_lookahead = 0.5  # Scaling factor for speed

        # Initialize the MAVLink object
        self.mav = mav

    def dynamics(self, state, control, prev_delta):
        x, y, theta, v = state
        v_cmd, delta_cmd = control
        L = self.wheel_base

        # Acceleration calculation with limits
        acceleration = (v_cmd - v) / self.dt
        acceleration = np.clip(acceleration, -self.max_acceleration, self.max_acceleration)
        v_new = v + acceleration * self.dt
        v_new = max(v_new, 0.0)  # Ensure non-negative velocity

        # Steering angle rate calculation with limits
        delta_change = delta_cmd - self.previous_delta
        steering_rate = delta_change / self.dt
        steering_rate = np.clip(steering_rate, -self.max_steering_rate, self.max_steering_rate)
        delta_new = prev_delta + steering_rate * self.dt

        delta_new = np.clip(delta_new, self.vehicle.delta_min, self.vehicle.delta_max)

        # Update position and heading
        x_new = x + v_new * np.cos(theta) * self.dt
        y_new = y + v_new * np.sin(theta) * self.dt
        theta_new = theta + (v_new / L) * np.tan(delta_new) * self.dt

        # Wrap theta_new to [-pi, pi]
        theta_new = (theta_new + np.pi) % (2 * np.pi) - np.pi

        return np.array([x_new, y_new, theta_new, v_new]), delta_new

    def angle_difference(self, angle1, angle2):
        diff = angle1 - angle2
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
        return diff

    def compute_lookahead_distance(self, speed):
        return self.base_lookahead + self.scaling_factor_lookahead * abs(speed)

    def adjust_prediction_horizon(self, speed):
        T = int(np.clip(self.scaling_factor_T * abs(speed) / self.dt, self.T_min, self.T_max))
        return T

    def find_closest_path_index(self, position):
        distances = np.hypot(self.path_x - position[0], self.path_y - position[1])
        closest_index = np.argmin(distances)
        return closest_index

    def cost_function(self, state, control, prev_control, cumulative_heading_change, prev_index):
        x, y, theta, v = state
        # target_x, target_y = target
        v_cmd, delta_cmd = control

        closest_index_sim = self.find_closest_path_index([x, y])

        target_x = self.path_x[closest_index_sim]
        target_y = self.path_y[closest_index_sim]

        progress = closest_index_sim - prev_index 

        if progress < 0:
            backward_progress_penalty = self.w_backward * (-progress)
        else:
            backward_progress_penalty = 0.0
        prev_index = closest_index_sim

        # Position error
        position_error = np.sqrt((x - target_x) ** 2 + (y - target_y) ** 2)

        # Orientation error
        desired_theta = np.arctan2(target_y - y, target_x - x)
        orientation_error = np.abs(self.angle_difference(theta, desired_theta))

        # Control effort (smoothness)
        control_effort = np.linalg.norm(control - prev_control)

        # Acceleration cost
        acceleration = (v_cmd - prev_control[0]) / self.dt
        acceleration_cost = self.w_acceleration * acceleration**2

        # Speed cost (penalize exceeding maximum speed)
        speed_cost = 0.0
        if v_cmd > self.desired_speed:
            speed_cost = self.w_speed * (v_cmd - self.desired_speed)**2
        elif v_cmd < self.min_speed:
            speed_cost = self.w_speed * (self.min_speed - v_cmd)**2

        low_speed_penalty = 0.0
        if v_cmd < self.min_speed:
            low_speed_penalty = self.w_low_speed * (self.min_speed - v_cmd)**2

        high_speed_penalty = 0.0
        if v_cmd > self.max_speed:
            high_speed_penalty = self.w_high_speed * (v_cmd - self.max_speed)**2

        # Speed deviation cost (penalize deviation from desired speed)
        speed_deviation = v_cmd - self.desired_speed
        speed_deviation_cost = self.w_speed_deviation * speed_deviation**2

        if closest_index_sim >= len(self.path_x) - 1:
            idx1 = len(self.path_x) - 2
            idx2 = len(self.path_x) - 1
        elif closest_index_sim == 0:
            idx1 = 0
            idx2 = 1
        else:
            idx1 = closest_index_sim - 1
            idx2 = closest_index_sim

        path_dx = self.path_x[idx2] - self.path_x[idx1]
        path_dy = self.path_y[idx2] - self.path_y[idx1]
        
        path_direction = np.array([path_dx, path_dy], dtype=float)
        path_direction_norm = np.linalg.norm(path_direction)
        
        if path_direction_norm == 0:
            path_direction_norm = 1.0
        path_direction /= path_direction_norm

        path_point = np.array([self.path_x[closest_index_sim], self.path_y[closest_index_sim]])
        to_vehicle = np.array([x - path_point[0], y - path_point[1]], dtype=float)

        deviation = np.abs(path_direction[0] * to_vehicle[1] - path_direction[1] * to_vehicle[0])

        deviation_penalty = 0.0
        if deviation > self.max_deviation:
            deviation_penalty = self.w_deviation * (deviation - self.max_deviation)**2

        path_direction_angle = np.arctan2(path_dy, path_dx)
        heading_diff = self.angle_difference(theta, path_direction_angle)
        heading_alignment_cost = self.w_heading * heading_diff**2

        heading_change = np.abs(control[1] - prev_control[1])
        cumulative_heading_change += heading_change

        cumulative_heading_cost = self.w_cumulative_heading * cumulative_heading_change



        # Total cost
        total_cost = (
                      self.w_position * position_error +
                      self.w_orientation * orientation_error +
                      self.w_control * control_effort +
                      acceleration_cost +
                      speed_cost +
                      speed_deviation_cost +
                      deviation_penalty +
                      heading_alignment_cost + 
                      low_speed_penalty + 
                      high_speed_penalty +
                      cumulative_heading_cost + 
                      backward_progress_penalty
                      )
        return total_cost, cumulative_heading_change, prev_index

    def simulate_control(self):
        self.vehicle.cleanup()

        # global emergency_stop

        # stop_event = Event()

        # recieve_thread = Thread(target=recieve_messages, args=(mav, stop_event))

        # time.sleep(0.25)
        plt.figure()
        plt.ion()
        plt.show()
        # plt.plot(self.path_x, self.path_y, 'r--', label='Target Path')  # Plot the target path

        num_steps = len(self.path_x)
        prev_control = self.control_mean.copy()

        # Initialize lists for plotting speed and theta over time
        speeds = []
        thetas = []
        state = self.state_real

        path_taken = []

        print("num steps:", num_steps)
        final_x = self.path_x[-1]
        final_y = self.path_y[-1]

        distance_from_end = np.sqrt((state[0] - final_x)**2 + (state[1] - final_y)**2)

        t = 0
        prev_index = 0
        # while abs(distance_from_end) > 5 and not emergency_stop:  
        while abs(distance_from_end) > 5:  
            plt.cla()

            plt.plot(self.path_x, self.path_y, 'r--', label='Target Path' if t == 0 else "")
            # Get the current state
            state = self.state_real
            current_speed = state[3]

            # Adjust prediction horizon based on speed
            self.T = self.adjust_prediction_horizon(current_speed)

            # Compute lookahead distance
            lookahead_distance = self.compute_lookahead_distance(current_speed)

            # Find the closest path index
            closest_index = self.find_closest_path_index(state[:2])

            # Compute target index with lookahead
            path_length = len(self.path_x)
            target_index = min(closest_index + int(lookahead_distance / (np.hypot(np.diff(self.path_x).mean(), np.diff(self.path_y).mean()))), path_length - 1)
            target_index = max(target_index, closest_index + 1)
            target = np.array([self.path_x[target_index], self.path_y[target_index]])
            
            # Generate control sequences
            # control_std = np.array([0.1, np.radians(1)])
            position_error = np.linalg.norm(state[:2] - target)
            
            max_control_std = np.array([0.5, np.radians(5)])
            min_control_std = np.array([0.1, np.radians(1)])

            error_ratio = min(position_error / self.max_deviation, 1.0)
            control_std = min_control_std + error_ratio * (max_control_std - min_control_std)

            max_control_std = np.array([0.3, np.radians(3)])
            control_std = np.minimum(control_std, max_control_std)
            # Sample control sequences
            U_samples = np.random.normal(self.control_mean, control_std, size=(self.N, self.T, 2))

            # Clip control samples to physical limits
            U_samples[:, :, 0] = np.clip(U_samples[:, :, 0], self.vehicle.v_min, self.vehicle.v_max)  # Speed
            U_samples[:, :, 1] = np.clip(U_samples[:, :, 1], self.vehicle.delta_min, self.vehicle.delta_max)  # Steering

            # self.state_real = self.dynamics(self.state_real, self.control_mean)

            # Simulate trajectories and compute costs
            costs = np.zeros(self.N)
            trajectories = []
            for n in range(self.N):
                state_sim = state.copy()
                cost = 0.0
                prev_u = prev_control.copy()
                prev_delta_sim = prev_control[1]
                prev_index = 0
                cumulative_heading_change = 0.0
                trajectory = [state_sim[:2].copy()]
                for k in range(self.T):
                    u = U_samples[n, k, :]
                    # Simulate the vehicle dynamics
                    state_sim, delta_new_sim = self.dynamics(state_sim, u, prev_delta_sim) 
                    prev_delta_sim = delta_new_sim
                    trajectory.append(state_sim[:2].copy())
                    # Compute cost
                    cost_step, cumulative_heading_change, prev_index = self.cost_function(state_sim, u, prev_u, cumulative_heading_change, prev_index)
                    cost += cost_step
                    # cost += self.cost_function(state_sim, target, u, prev_u)
                    prev_u = u  # Update previous control
                costs[n] = cost
                trajectories.append(trajectory)
            
            # Plot all trajectories
            for trajectory in trajectories:
                traj = np.array(trajectory)
                plt.plot(traj[:, 0], traj[:, 1], 'g-', alpha=0.1)

            path_taken.append(self.state_real[:2].copy())
            if len(path_taken) > 1:
                path_taken_array = np.array(path_taken)
                plt.plot(path_taken_array[:, 0], path_taken_array[:, 1], 'b-', alpha=0.5)

            best_index = np.argmin(costs)
            best_trajectory = trajectories[best_index]
            best_traj = np.array(best_trajectory)
            plt.plot(best_traj[:, 0], best_traj[:, 1], 'b-', label='Best Path' if t == 0 else "")

            # Compute weights
            beta = np.min(costs)
            weights = np.exp(-1.0 / self.lambda_ * (costs - beta))
            weights_sum = np.sum(weights)
            if weights_sum == 0:
                weights = np.ones_like(weights) / len(weights)
            else:
                weights /= weights_sum

            # Compute weighted average of control sequences
            U_weighted = np.sum(U_samples * weights[:, np.newaxis, np.newaxis], axis=0)

            # Apply the first control input
            # control = U_weighted[0, :]  # First control input in the sequence
            control = U_samples[best_index, 0, :]  # Best control input in the sequence

            # Smooth control inputs using exponential moving average
            alpha = 0.7  # Smoothing factor
            self.control_mean = alpha * control + (1 - alpha) * self.control_mean
            # self.control_mean = control
            self.control_mean[0] = np.clip(self.control_mean[0], self.vehicle.v_min, self.vehicle.v_max)
            self.control_mean[1] = np.clip(self.control_mean[1], self.vehicle.delta_min, self.vehicle.delta_max)

            # Move the vehicle based on the computed control inputs
            self.vehicle.move(self.control_mean[0], np.degrees(self.control_mean[1]))

            # Send the current location to the ground station
            # send_current_location(mav, self.state_real[0], self.state_real[1])
            send_current_location(mav, self.state_real[0], self.state_real[1])
            
            # Update the real state based on the control inputs
            self.state_real, delta_new_real = self.dynamics(self.state_real, self.control_mean, self.previous_delta)
            self.previous_delta = delta_new_real

            prev_control = self.control_mean.copy()

            # Record speed and theta for plotting
            speeds.append(self.state_real[3])
            thetas.append(self.state_real[2])

            plt.plot(self.state_real[0], self.state_real[1], 'ro', label='Real Path' if t == 0 else "")

            # Print debug information
            print(f"Time step { t }:")
            print(f"  Control: v_cmd={self.control_mean[0]:.2f} m/s, delta_cmd={np.degrees(self.control_mean[1]):.2f} degrees")
            print(f"  State after move: x={self.state_real[0]:.2f}, y={self.state_real[1]:.2f}, theta={np.degrees(self.state_real[2]):.2f} degrees, v={self.state_real[3]:.2f} m/s")

            # Plot the real system's current position at each time step
            plt.plot(self.state_real[0], self.state_real[1], 'bo', label='Real Path' if t == 0 else "")

            plt.pause(0.01)  # Update the plot in real-time

            distance_from_end = np.sqrt((state[0] - final_x)**2 + (state[1] - final_y)**2)

            t += 1
        self.vehicle.cleanup()

        stop_event.set()
        recieve_thread.join()

        plt.legend()
        plt.ioff()
        plt.show()  # Show the final plot at the end of the simulation

        # Plot speed and orientation over time
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(speeds)
        plt.title('Vehicle Speed Over Time')
        plt.ylabel('Speed (m/s)')

        plt.subplot(2, 1, 2)
        plt.plot(np.degrees(thetas))
        plt.title('Vehicle Orientation Over Time')
        plt.ylabel('Theta (degrees)')
        plt.xlabel('Time Step')
        plt.tight_layout()
        plt.show()
