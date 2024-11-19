import time
import sys
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

# Define GPIO pin for the relay
RELAY_PIN = 21

# Define servo index
LEFT_SERVO_INDEX = 0
RIGHT_SERVO_INDEX = 1
STEERING_SERVO_INDEX = 2

# Vehicle Parameters
WHEEL_TRACK = 0.72898      # Distance between center of left and right wheel (meters)
WHEELBASE = 0.93345        # Distance between front and rear axle (meters)
V_MAX = 10.0                # Maximum speed (m/s)
MAX_STEERING_ANGLE = 20.0  # Maximum steering angle before wheel rub (degrees) 

# Steering servo parameters
STEERING_CENTER_ANGLE = 97
STEERING_LEFT_LIMIT = STEERING_CENTER_ANGLE - MAX_STEERING_ANGLE
STEERING_RIGHT_LIMIT = STEERING_CENTER_ANGLE + MAX_STEERING_ANGLE

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT)
    
# Initialize servo kit
kit = ServoKit(channels=8)

# Function to turn on relay pin
def turn_on_relay():
    GPIO.output(RELAY_PIN, GPIO.HIGH)

# Function to turn off relay pin
def turn_off_relay():
    GPIO.output(RELAY_PIN, GPIO.LOW)

# Function to move servo
def move_servo(servo_index, angle):
    angle = max(0, min(180, angle))
    kit.servo[servo_index].angle = angle

def steering_input_to_servo_angle(steering_input_deg):
    servo_angle = STEERING_CENTER_ANGLE + steering_input_deg
    servo_angle = max(STEERING_LEFT_LIMIT, min(STEERING_RIGHT_LIMIT, servo_angle))
    return servo_angle

def compute_wheel_speeds(velocity, steering_angle_deg):
    # Limit steering to maximum allowed angle
    angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle_deg))

    if steering_angle_deg == 0:
        V_left = velocity
        V_right = velocity
    else:
        # Convert angle to radians
        delta = np.radians(angle)

        # Calculate turning radius R
        R = WHEELBASE / np.tan(delta)

        # Calculate Radii of left and right wheels
        R_left = R - (WHEEL_TRACK / 2)
        R_right = R + (WHEEL_TRACK / 2)

        # Calculate angular velocity omega
        omega = velocity / R

        V_left = omega * R_left
        V_right = omega * R_right

        max_wheel_speed = max(abs(V_left), abs(V_right))
        if max_wheel_speed > V_MAX:
            scale = V_MAX / max_wheel_speed
            V_left *= scale
            V_right *= scale
    return V_left, V_right

def wheel_speed_to_servo_angle(wheel_speed, V_max):
    # Map wheel speed (-V_max, V_max) to servo angle (0, 180)
    servo_angle = 90 + (wheel_speed / V_max) * 90
    servo_angle = max(0, min(180, servo_angle))
    return servo_angle

if len(sys.argv) != 3:
    print("Usage: python3 servo_cmd.py <velocity> <angle>")
    sys.exit(1)

# Turn on relay pin 21
turn_on_relay()

try:
    velocity = int(sys.argv[1])
    steering_input_deg = int(sys.argv[2])

    # Map steering input to servo angle
    steering_input_deg = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_input_deg))
    steering_servo_angle = steering_input_to_servo_angle(steering_input_deg)
    
    # Compute wheel speeds
    V_left, V_right = compute_wheel_speeds(velocity, steering_input_deg)

    # Map wheel speed to servo angles
    servo_angle_left = wheel_speed_to_servo_angle(V_left, V_MAX)
    servo_angle_right = wheel_speed_to_servo_angle(V_right, V_MAX)

    # Move servos
    move_servo(LEFT_SERVO_INDEX, servo_angle_left)
    move_servo(RIGHT_SERVO_INDEX, servo_angle_right)   
    move_servo(STEERING_SERVO_INDEX, steering_servo_angle)

except KeyboardInterrupt:
    print("\nProgram stopped by the user.")

finally:
    # Turn off relay pin 21
    turn_off_relay()
    # Clean up GPIO settings
    GPIO.cleanup()
