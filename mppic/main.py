'''
 Author: Ben Sailor
 Description: This script is the main function that recieves waypoints from the 
 ground station and sends them to the vehicle to follow in the mppic simulation
'''
import numpy as np
from mppic import RMPPIController
from utils import generate_path_from_waypoints
import time
import serial
from threading import Thread, Event
from Mavlink import MAVLink

SERIAL_PORT_RF = '/dev/ttyUSB0'
SERIAL_PORT_IMU = '/dev/ttyUSB1'
BAUD_RATE = 57600
CALLSIGN = 'YOURCALSGN'

waypoints = []
mission_started = False



def handle_waypoint_list(msg):
    if msg.callsign != CALLSIGN:
        print(f"Received waypoints from {msg.callsign}. Ignoring.")
        return
    
    wp_count = msg.wp_count
    
    for i in range(wp_count):
        x = msg.x_coordinate[i]
        y = msg.y_coordinate[i]
        
        waypoints.append([x, y])

    print(f"Received {wp_count} waypoints from {msg.callsign}.")
    for idx, wp in enumerate(waypoints):
        print(f"Waypoint {idx+1}: X={wp[0]}, Y={wp[1]}")

def handle_start_mission(msg):
    if msg.callsign != CALLSIGN:
        print(f"Received mission start from {msg.callsign}. Ignoring.")
        return

    mission_started = True
    print(f"Received mission start from {msg.callsign}. Starting mission.")


def main():
    try:
        ser = serial.Serial(SERIAL_PORT_RF, BAUD_RATE)
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT_RF}: {e}")
        return
    
    mav = MAVLink(ser)

    while True:
        if ser.in_waiting > 0:
            c = ser.read(1)
            msg = mav.parse_char(c)
            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == 'WAYPOINT_LIST':
                    handle_waypoint_list(msg)
                elif msg_type == 'START_MISSION':
                    handle_start_mission(msg)
            if mission_started == True:
                break


    path_x, path_y = generate_path_from_waypoints(waypoints, corner_radius=5.0, num_points_per_arc=20)
    print(path_x, path_y)

    if path_x is None or path_y is None:
        print("Failed to generate path from waypoints.")
        return

    # Initialize the RMPPI controller
    controller = RMPPIController(path_x, path_y, mav)
    controller.setup(mav, port=SERIAL_PORT_IMU, baudrate=None)

    # Simulate control and follow the path
    controller.control()

    ser.close()
if __name__ == '__main__':
    main()
