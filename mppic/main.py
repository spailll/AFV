import numpy as np
from mppic import RMPPIController
from utils import generate_path_from_waypoints
import time
import serial
from threading import Thread, Event
from Mavlink import mavlink

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 57600
CALLSIGN = 'YOURCALSGN'

waypoints = []
mission_started = False

def recieve_messages(mav, stop_event):
    global waypoints, mission_started, emergency_stop
    while not stop_event.is_set():
        msg = mav.recv_match(blocking=True)
        if not msg:
            continue

        msg_type = msg.get_type()

        if msg_type == 'WAYPOINT_LIST':
            handle_waypoint_list(msg)

        elif msg_type == 'START_MISSION':
            handle_start_mission(msg)
        
def handle_waypoint_list(msg):
    global waypoints

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
    global mission_started

    if msg.callsign != CALLSIGN:
        print(f"Received mission start from {msg.callsign}. Ignoring.")
        return

    mission_started = True
    print(f"Received mission start from {msg.callsign}. Starting mission.")


def main():
    global waypoints, mission_started, emergency_stop

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return
    
    mav = mavlink.MAVLink(ser)

    stop_event = Event()

    recieve_thread = Thread(target=recieve_messages, args=(mav, stop_event))
    recieve_thread.daemon = True
    recieve_thread.start()

    while True:
        if mission_started:
            break
        time.sleep(0.5)

    stop_event.set()
    recieve_thread.join()

    path_x, path_y = generate_path_from_waypoints(waypoints, corner_radius=5.0, num_points_per_arc=20)

    print(path_x, path_y)

    # Initialize the RMPPI controller
    controller = RMPPIController(path_x, path_y)

    # Simulate control and follow the path
    controller.simulate_control(mav)

    ser.close()
if __name__ == '__main__':
    main()
