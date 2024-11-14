import subprocess
import threading
import time

def run_gpsimu():
    # Run the gpsimu_reader executable with stdbuf to disable buffering
    process = subprocess.Popen(
        ["stdbuf", "-oL", "/home/m7q8/Desktop/Linux_C/normal/gpsimu_reader", "/dev/ttyUSB0"],
        stdout=subprocess.PIPE,
        text=True
    )

    # Read and print each line of output from the executable in real-time
    for line in process.stdout:
        print(line.strip())  # Replace this with your data processing or telemetry sending

    # Close the process's output stream and wait for the process to finish
    process.stdout.close()
    process.wait()

def start_gpsimu_thread():
    # Create and start a thread for GPS/IMU data extraction
    gps_thread = threading.Thread(target=run_gpsimu, daemon=True)
    gps_thread.start()
    return gps_thread

# Main program
if __name__ == "__main__":
    # Start the GPS/IMU data extraction thread
    gps_thread = start_gpsimu_thread()

    # Run other parts of your main program in parallel
    try:
        while True:
            print("Main program running other tasks...")
            # Simulate other tasks, such as MPPIC calculations or telemetry monitoring
            time.sleep(1)  # Adjust sleep timing as necessary for your main loop
    except KeyboardInterrupt:
        print("Main program interrupted. Exiting...")
