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

def extract_rmc_data(deviceModel):
    
    # This function extracts RMC data and prints it as a comma-separated list.
    
    # Extract the data from the device
    temperature = deviceModel.GetDeviceData("Temperature")
    accX = deviceModel.GetDeviceData("AccX")
    accY = deviceModel.GetDeviceData("AccY")
    accZ = deviceModel.GetDeviceData("AccZ")
    gyroX = deviceModel.GetDeviceData("GyroX")
    gyroY = deviceModel.GetDeviceData("GyroY")
    gyroZ = deviceModel.GetDeviceData("GyroZ")
    angleX = deviceModel.GetDeviceData("AngleX")
    angleY = deviceModel.GetDeviceData("AngleY")
    angleZ = deviceModel.GetDeviceData("AngleZ")
    lon = deviceModel.GetDeviceData("Lon")
    lat = deviceModel.GetDeviceData("Lat")

    # Format the data as a comma-separated list
    csv_data = (
        f"{temperature},"
        f"{accX}, {accY}, {accZ},"
        f"{gyroX}, {gyroY}, {gyroZ},"
        f"{angleX}, {angleY}, {angleZ},"
        f"{lon}, {lat}"
    )

    # Print the CSV data
    print(csv_data)

def JY901_updateData(deviceModel):

    # This function is called when sensor data is refreshed and extracts necessary data.
    
    extract_rmc_data(deviceModel)

if __name__ == '__main__':
    print(greet)

    try:
        # Initialize the device
        JY901 = JY901()
        if platform.system().lower() == 'linux':
            PortName = "/dev/ttyUSB0"
        else:
            PortName = "COM3"
        Baudrate = 9600

        # Set serial port and baud rate
        JY901.SetPortName(PortName)
        JY901.SetBaudrate(Baudrate)

        # Open serial port
        JY901.Open()

        if JY901.IsOpen():
            print("Device opened successfully.")

            # Bind receiving events to extract RMC data
            JY901.AddOnRecord(JY901_updateData)

            print("Reading RMC data ... Press Ctrl+C to stop.")

            # Keep the program running until interrupted by Ctrl+C
            while True:
                pass

        else:
            print("Failed to open the device.")

    except KeyboardInterrupt:
        print("\nStopping data extraction...")

    finally:
        if JY901.IsOpen():
            # Close the device
            JY901.Close()
            # Remove event
            JY901.RemoveOnRecord(JY901_updateData)
            print("Device closed.")
