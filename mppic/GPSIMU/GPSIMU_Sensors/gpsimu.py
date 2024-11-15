import subprocess
import threading
import time

class GPSIMUReader:
    def __init__(self, gpsimu_executable="/home/afvokstate/AFV/mppic/GPSIMU/GPSIMU_Sensors/normal/gpsimu_reader", port="/dev/ttyUSB1"):
        """
        Initializes the GPSIMUReader.

        Args:
            gpsimu_executable (str): Path to the GPSIMU reader executable.
            port (str): Serial port to which the GPSIMU device is connected.
        """
        self.gpsimu_executable = gpsimu_executable
        self.port = port
        self.shared_data = {'latest_output': None}
        self.data_lock = threading.Lock()
        self.thread = None
        self.running = False

    def _run_gpsimu(self):
        """
        Internal method to run the GPSIMU reader subprocess and update the latest output.
        """
        # Start the GPSIMU reader subprocess
        process = subprocess.Popen(
            ["stdbuf", "-oL", self.gpsimu_executable, self.port],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        self.running = True

        try:
            # Read each line of output from the subprocess
            for line in process.stdout:
                line = line.strip()
                with self.data_lock:
                    self.shared_data['latest_output'] = line

                # Check if we need to stop
                if not self.running:
                    break
        except Exception as e:
            print(f"Error in GPSIMUReader thread: {e}")
        finally:
            # Terminate the subprocess if it's still running
            process.terminate()
            process.wait()
            self.running = False

    def start(self):
        """
        Starts the GPSIMU data extraction thread.
        """
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self._run_gpsimu, daemon=True)
            self.thread.start()

    def stop(self):
        """
        Stops the GPSIMU data extraction thread.
        """
        self.running = False
        if self.thread is not None:
            self.thread.join()
            self.thread = None

    def get_latest_output(self):
        """
        Returns the latest GPSIMU output.

        Returns:
            str or None: The most recent output from the GPSIMU reader, or None if no data is available.
        """
        with self.data_lock:
            str =  self.shared_data['latest_output']
            data = str.split(',')
            float_values = [float(value) for value in data]
            values = [float_values[13], float_values[14], float_values[5]]
            return values


    def clear_latest_output(self):
        """
        Clears the latest GPSIMU output.
        """
        with self.data_lock:
            self.shared_data['latest_output'] = None

    def is_running(self):
        """
        Checks if the GPSIMU reader is currently running.

        Returns:
            bool: True if running, False otherwise.
        """
        return self.running

    def __enter__(self):
        """
        Enables use of the class with the 'with' statement.
        """
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Ensures the GPSIMU reader is stopped when exiting the 'with' block.
        """
        self.stop()
