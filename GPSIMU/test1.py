import serial
import struct
import os
import time 

gps_imu = serial.Serial('COM3', baudrate=19200, timeout=1)

def clear_screen():
    """Clears the terminal screen."""
    os.system('cls' if os.name == 'nt' else 'clear')

def parse_wtgahrs2_data(raw_data):
    """Parses the raw hex data from the WTGAHRS2 module and extracts labeled values."""
    try:
        
        hex_data = raw_data.hex()

        
        ax = struct.unpack('>h', bytes.fromhex(hex_data[68:72]))[0] / 32768 * 16  # X-axis acceleration in g
        ay = struct.unpack('>h', bytes.fromhex(hex_data[72:76]))[0] / 32768 * 16  # Y-axis acceleration in g
        az = struct.unpack('>h', bytes.fromhex(hex_data[76:80]))[0] / 32768 * 16  # Z-axis acceleration in g

        gx = struct.unpack('>h', bytes.fromhex(hex_data[80:84]))[0] / 32768 * 2000  # X-axis angular velocity in degrees/sec
        gy = struct.unpack('>h', bytes.fromhex(hex_data[84:88]))[0] / 32768 * 2000  # Y-axis angular velocity in degrees/sec
        gz = struct.unpack('>h', bytes.fromhex(hex_data[88:92]))[0] / 32768 * 2000  # Z-axis angular velocity in degrees/sec

        lon = struct.unpack('>l', bytes.fromhex(hex_data[146:154]))[0] / 1e7  # Longitude in degrees
        lat = struct.unpack('>l', bytes.fromhex(hex_data[154:162]))[0] / 1e7  # Latitude in degrees

        
        return (f"Acceleration X: {ax} g\n"
                f"Acceleration Y: {ay} g\n"
                f"Acceleration Z: {az} g\n\n"
                f"Angular Velocity X: {gx} °/s\n"
                f"Angular Velocity Y: {gy} °/s\n"
                f"Angular Velocity Z: {gz} °/s\n\n"
                f"GPS Latitude: {lat}°\n"
                f"GPS Longitude: {lon}°")

    except Exception as e:
        print(f"Error parsing data: {e}")
        return None

def read_wtgahrs2():
    """Reads raw data from the WTGAHRS2 module and parses it."""
    while True:
        try:
            clear_screen()

            raw_data = gps_imu.read(100)
            print(f"Raw Data (Hex): {raw_data.hex()}")
            labeled_data = parse_wtgahrs2_data(raw_data)
            if labeled_data:
                print(f"Parsed Data:\n{labeled_data}")
            
            
            time.sleep(2) 

        except Exception as exp:
            print(f"Error reading data: {exp}")

try:
    read_wtgahrs2()
finally:
    print("Closing serial connection...")
    if gps_imu.is_open:
        gps_imu.close()
