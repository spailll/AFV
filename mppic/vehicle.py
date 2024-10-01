import subprocess

class Vehicle:
    def __init__(self):
        self.processes = []  # List of processes for each motor

    def move(self, speed, delta):
        """
        Control the vehicle's speed and steering using differential drive.

        :param speed: A float between -1.0 and 1.0 where:
                    #   -1.0 is full reverse,
                       0.0 is neutral,
                       1.0 is full forward.
        :param delta: A float between -1.0 and 1.0 where:
                      -1.0 is full left turn,
                       0.0 is straight,
                       1.0 is full right turn.
        """
        # Ensure speed and delta are within -1.0 to 1.0
        speed = max(-1.0, min(1.0, speed))
        delta = max(-1.0, min(1.0, delta))

        # Calculate left and right wheel speeds for differential drive
        left_speed = speed * (1.0 - delta)
        right_speed = speed * (1.0 + delta)
        # Normalize wheel speeds if they exceed the range [-1.0, 1.0]
        max_wheel_speed = max(abs(left_speed), abs(right_speed))
        if max_wheel_speed > 1.0:
            left_speed /= max_wheel_speed
            right_speed /= max_wheel_speed

        # Map wheel speeds to servo values (45 to 135)
        left_servo_value = left_speed * 30 + 90
        right_servo_value = right_speed * 30 + 90

        left_servo_value = max(60, min(120, left_servo_value))
        right_servo_value = max(60, min(120, right_servo_value))

        left_servo_value_int = int(round(left_servo_value))
        right_servo_value_int = int(round(right_servo_value))

        steering_servo_value = (delta + 1.0) * 30 + 67  # Scale delta to [0, 2], then to [0, 60], shift to [67, 127]

        steering_servo_value = max(67, min(127, steering_servo_value))
        steering_servo_value_int = int(round(steering_servo_value))

        ssh_command_left = f"ssh pi@10.42.0.75 './servo3.py 0 {str(left_servo_value_int)}'"
        ssh_command_right = f"ssh pi@10.42.0.75 './servo3.py 1 {str(right_servo_value_int)}'"
        ssh_command_steer = f"ssh pi@10.42.0.75 './servo3.py 2 {str(steering_servo_value_int)}'"
        # print(ssh_command_left)
        # print(ssh_command_right)
        # print(ssh_command_steer)

        # Call servo3.py for each motor, non-blocking
        self.processes = []
        self.processes.append(subprocess.Popen(ssh_command_left, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)) 
        self.processes.append(subprocess.Popen(ssh_command_right, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)) 
        self.processes.append(subprocess.Popen(ssh_command_steer, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE))

    def cleanup(self):
        ssh_command_left = f"ssh pi@10.42.0.75 './servo3.py 0 90'"
        ssh_command_right = f"ssh pi@10.42.0.75 './servo3.py 1 90'"
        ssh_command_steer = f"ssh pi@10.42.0.75 './servo3.py 2 97'"
        # print(ssh_command_left)
        # print(ssh_command_right)
        # print(ssh_command_steer)

        # Call servo3.py for each motor, non-blocking
        self.processes = []
        self.processes.append(subprocess.Popen(ssh_command_left, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)) 
        self.processes.append(subprocess.Popen(ssh_command_right, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)) 
        self.processes.append(subprocess.Popen(ssh_command_steer, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE))