'''
 Author: Mohammed Sheshtar
 Description: This script is designed to extract RMC data from the 
 GPSIMU sensor from WIT motions by importing the module's library to extract
 the data from the appropriate registers within the module and display it
 in CSV format
'''
import platform
import os
from Dll.lib.Modular.JY901 import JY901

greet = """
GPSIMU module, Extracting RMC data.
Press Ctrl+C to stop.
"""

class GPSIMUDevice:
    def __init__(self, port=None, baudrate=None):
        self.port = "/dev/ttyUSB0" if port is None else port
        self.baudrate = 9600 if baudrate is None else baudrate
        self.device = JY901()
        self.device.setPortName(self.port)
        self.device.setBaudrate(self.baudrate)
        self.is_open = False

    def open(self):
        self.device.open()
        if self.device.IsOpen():
            self.is_open = True
            time.sleep(2)
        else:
            raise ConnectionError("Failed to open the GPSIMU device.")

    def close(self):
        if self.device.IsOpen():
            self.device.Close()
            self.is_open = False

    def get_data(self):
        if not self.is_open:
            raise ConnectionError("Device is not open. Call 'open()' before retrieving data.")
        try:
            data = {
                "Temperature": self.device.GetDeviceData("Temperature"),
                "AccX": self.device.GetDeviceData("AccX"),
                "AccY": self.device.GetDeviceData("AccY"),
                "AccZ": self.device.GetDeviceData("AccZ"),
                "GyroX": self.device.GetDeviceData("GyroX"),
                "GyroY": self.device.GetDeviceData("GyroY"),
                "GyroZ": self.device.GetDeviceData("GyroZ"),
                "AngleX": self.device.GetDeviceData("AngleX"),
                "AngleY": self.device.GetDeviceData("AngleY"),
                "AngleZ": self.device.GetDeviceData("AngleZ"),
                "Lon": self.device.GetDeviceData("Lon"),
                "Lat": self.device.GetDeviceData("Lat")
            }
            return data
        except Exception as e:
            print(f"An error occurred while retrieving data: {e}")
            return None
    
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

