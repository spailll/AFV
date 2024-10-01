import numpy as np
import matplotlib.pyplot as plt
from vehicle import Vehicle

class RMPPIController:
    def __init__(self, path_x, path_y, wheel_base=0.5, dt=0.1, T=10, lambda_=1.0):
        self.path_x = path_x
        self.path_y = path_y
        self.wheel_base = wheel_base
        self.dt = dt
        self.T = T
        self.lambda_ = lambda_
        self.state_real = np.array([0, 0, 0])  # Real system state: [x, y, theta]
        self.U_init = np.zeros((T, 2))  # Initial control sequence
        
        self.vehicle = Vehicle()

    def dynamics(self, state, control):
        x, y, theta = state
        v, omega = control
        x_new = x + v * np.cos(theta) * self.dt
        y_new = y + v * np.sin(theta) * self.dt
        theta_new = theta + omega * self.dt
        return np.array([x_new, y_new, theta_new])

    def cost_function(self, state, target):
        x, y, _ = state
        target_x, target_y = target
        return np.sqrt((x - target_x) ** 2 + (y - target_y) ** 2)

    def feedback_control(self, state, target):
        Kp_v = 0.6  # Proportional gain for linear velocity
        Kp_omega = 0.9  # Proportional gain for angular velocity

        target_x, target_y = target
        x, y, theta = state

        # Compute the angle and distance to the target
        angle_to_target = np.arctan2(target_y - y, target_x - x)
        distance_to_target = np.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)

        # Compute the control inputs: linear velocity and angular velocity
        v = Kp_v * distance_to_target  # Linear velocity
        omega = Kp_omega * (angle_to_target - theta)  # Angular velocity

        # Limit v and omega to prevent overflow or instability
        v = np.clip(v, -1.0, 1.0)  # Limit speed to a range [-1, 1] m/s
        omega = np.clip(omega, -1.0, 1.0)  # Limit angular velocity to a range [-1, 1] rad/s

        return np.array([v, omega])

    def simulate_control(self, lookahead_steps=5):
        plt.figure()
        plt.plot(self.path_x, self.path_y, 'r--', label='Target Path')  # Plot the target path

        for t in range(len(self.path_x)):
            # Define the lookahead target point (future point on the path)
            lookahead_index = min(t + lookahead_steps, len(self.path_x) - 1)
            target_point = (self.path_x[lookahead_index], self.path_y[lookahead_index])

            # Apply feedback control to compute the velocities to move towards the lookahead target point
            control = self.feedback_control(self.state_real, target_point)
            
            # Move the vehicle based on the computed control inputs
            self.vehicle.move(control[0], control[1])
            print(control)

            # Update the real state based on the control inputs
            self.state_real = self.dynamics(self.state_real, control)

            # Plot the real system's current position at each time step
            plt.plot(self.state_real[0], self.state_real[1], 'bo', label='Real Path' if t == 0 else "")

            plt.pause(0.1)  # Update the plot in real-time

        plt.legend()
        plt.show()  # Show the final plot at the end of the simulation
