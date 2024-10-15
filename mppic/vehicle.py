import subprocess
import numpy as np
import math

class Vehicle:
    def __init__(self):
        self.v_min = 0.0        # Minimum linear velocity (m/s)
        self.v_max = 7.0         # Maximum linear velocity (m/s)
        self.delta_min = -math.radians(20)  # Minimum steering angle (radians)
        self.delta_max = math.radians(20)   # Maximum steering angle (radians)
        self.L = 0.72898         # Wheelbase length (m)
        self.processes = []      # List of processes for each motor
        

    def move(self, v_cmd, delta_cmd):
        """
        Control the vehicle's speed and steering using differential drive.

        :param v_cmd: Linear velocity command in m/s
        :param delta_cmd: Steering angle command in radians
        """
        # Ensure v_cmd and delta_cmd are within physical limits
        v_cmd = np.clip(v_cmd, self.v_min, self.v_max)
        delta_cmd = np.clip(delta_cmd, self.delta_min, self.delta_max)

        # Compute angular velocity omega
        omega = v_cmd / self.L * np.tan(delta_cmd)

        # Compute left and right wheel speeds
        v_left = v_cmd - (self.L / 2.0) * omega
        v_right = v_cmd + (self.L / 2.0) * omega

        # Map wheel speeds to servo commands
        left_servo_value_int = self.map_speed_to_servo(v_left)
        right_servo_value_int = self.map_speed_to_servo(v_right)

        # Map steering angle to servo command
        steering_servo_value_int = self.map_steering_to_servo(delta_cmd)

        # Prepare SSH commands
        ssh_command_left = f"ssh pi@10.42.0.75 './servo3.py 0 {left_servo_value_int}'"
        ssh_command_right = f"ssh pi@10.42.0.75 './servo3.py 1 {right_servo_value_int}'"
        ssh_command_steer = f"ssh pi@10.42.0.75 './servo3.py 2 {steering_servo_value_int}'"
        print(ssh_command_left)
        print(ssh_command_right)
        print(ssh_command_steer)

        # Execute SSH commands (non-blocking)
        self.processes = []
        self.processes.append(subprocess.Popen(ssh_command_left, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self.processes.append(subprocess.Popen(ssh_command_right, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self.processes.append(subprocess.Popen(ssh_command_steer, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE))

    def map_speed_to_servo(self, speed):
        """
        Maps wheel speed to servo value.

        :param speed: Wheel speed in m/s
        :return: Servo value (int)
        """
        # Servo parameters
        servo_min = 45
        servo_max = 135
        servo_neutral = 90

        # Normalize speed to [-1, 1]
        speed_normalized = speed / self.v_max
        speed_normalized = np.clip(speed_normalized, -1.0, 1.0)

        # Map to servo value
        if speed_normalized >= 0:
            servo_value = servo_neutral + speed_normalized * (servo_max - servo_neutral)
        else:
            servo_value = servo_neutral + speed_normalized * (servo_neutral - servo_min)

        servo_value = int(round(servo_value))
        servo_value = np.clip(servo_value, servo_min, servo_max)

        return servo_value

    def map_steering_to_servo(self, delta):
        """
        Maps steering angle to servo value.

        :param delta: Steering angle in radians
        :return: Servo value (int)
        """
        # Steering servo parameters
        steering_servo_min = 67
        steering_servo_max = 127

        delta_deg = np.degrees(delta)
        delta_min_deg = np.degrees(self.delta_min)
        delta_max_deg = np.degrees(self.delta_max)

        # Map delta from [delta_min_deg, delta_max_deg] to [steering_servo_min, steering_servo_max]
        steering_servo_value = steering_servo_min + (delta_deg - delta_min_deg) * (steering_servo_max - steering_servo_min) / (delta_max_deg - delta_min_deg)
        steering_servo_value = int(round(steering_servo_value))
        steering_servo_value = np.clip(steering_servo_value, steering_servo_min, steering_servo_max)

        return steering_servo_value

    def cleanup(self):
        ssh_command_left = f"ssh pi@10.42.0.75 './servo3.py 0 90'"
        ssh_command_right = f"ssh pi@10.42.0.75 './servo3.py 1 90'"
        ssh_command_steer = f"ssh pi@10.42.0.75 './servo3.py 2 97'"

        # Execute SSH commands to reset servos
        self.processes = []
        self.processes.append(subprocess.Popen(ssh_command_left, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self.processes.append(subprocess.Popen(ssh_command_right, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self.processes.append(subprocess.Popen(ssh_command_steer, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
