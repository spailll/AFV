import subprocess
import numpy as np
import math

class Vehicle:
    def __init__(self):
        self.v_min = 0.0        # Minimum linear velocity (m/s)
        self.v_max = 7.0        # Maximum linear velocity (m/s)
        self.delta_min = -20    # Minimum steering angle (radians)
        self.delta_max = 20     # Maximum steering angle (radians)
        self.remote_user = 'pi'
        self.remote_host = '10.42.0.75'
        self.remote_cmd = 'servo_cmd.py'
        self.process = None

    def move(self, v_cmd, delta_cmd):
        """
        Control the vehicle's speed and steering using differential drive.

        :param v_cmd: Linear velocity command in m/s
        :param delta_cmd: Steering angle command in radians
        """
        # Ensure v_cmd and delta_cmd are within physical limits
        v_cmd = np.clip(v_cmd, self.v_min, self.v_max)
        delta_cmd = np.clip(delta_cmd, self.delta_min, self.delta_max)

        ssh_command = f"ssh {self.remote_user}@{self.remote_host} './{self.remote_cmd} {v_cmd} {delta_cmd}'"
        print(ssh_command)

        # Execute SSH commands (non-blocking)
        self.process = subprocess.Popen(ssh_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def cleanup(self):
        ssh_command = f"ssh {self.remote_user}@{self.remote_host} './{self.remote_cmd} 0.0 0.0'"
        
        # Execute SSH commands to reset servos
        self.process = subprocess.Popen(ssh_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
