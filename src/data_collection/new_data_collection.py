import os
import time
import datetime
import csv
import serial
import termios
import sys
import tty
import threading

# --- Helper function for non-blocking keyboard input on Linux/macOS ---
def get_key():
    """Gets a single key press from the user without needing to press Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class DataCollector:
    """
    Handles keyboard input and data logging for control values in a thread-safe manner.
    """
    def __init__(self, base_path, max_steering):
        self.max_steering = max_steering
        
        # --- Control values ---
        self.steering = 0
        self.speed = 0
        self.left_speed = 0
        self.right_speed = 0

        # --- Control settings ---
        self.SPEED_STEP = 10
        self.MAX_SPEED = 180

        # --- Threading lock for safe access to control values ---
        self.lock = threading.Lock()

        # --- Setup data directories ---
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d_%H%M%S")
        self.session_path = os.path.join(base_path, timestamp)
        os.makedirs(self.session_path, exist_ok=True)
        print(f"✅ Data will be saved in: {self.session_path}")
        
        # --- Setup CSV log file ---
        log_file_path = os.path.join(self.session_path, "driving_log.csv")
        self.csv_file = open(log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        header = ['steering', 'left_speed', 'right_speed']
        self.csv_writer.writerow(header)

    def update_controls(self, key):
        """Updates steering and speed based on keyboard input."""
        with self.lock:
            if key == 'w':
                self.speed = min(self.MAX_SPEED, self.speed + self.SPEED_STEP)
            elif key == 's':
                self.speed = max(-self.MAX_SPEED, self.speed - self.SPEED_STEP)
            elif key == 'a':
                self.steering = max(-self.max_steering, self.steering - 1)
            elif key == 'd':
                self.steering = min(self.max_steering, self.steering + 1)
            elif key == 'x':
                self.speed = 0
                self.steering = 0
            
            # Enforce that left and right speeds are always identical
            self.left_speed = self.speed
            self.right_speed = self.speed

    def log_data(self):
        """Logs the current control data to the CSV file."""
        # Get a consistent snapshot of the values
        control_values = self.get_control_values()
        log_data = [control_values['steering'], control_values['left_speed'], control_values['right_speed']]
        self.csv_writer.writerow(log_data)

    def get_control_values(self):
        """Returns the current control values in a thread-safe way."""
        with self.lock:
            return {
                "steering": self.steering,
                "left_speed": self.left_speed,
                "right_speed": self.right_speed
            }

    def cleanup(self):
        """Closes the CSV file."""
        self.csv_file.close()
        print("Resources cleaned up.")

def keyboard_listener(collector, stop_event):
    """A function to be run in a separate thread to listen for keyboard input."""
    while not stop_event.is_set():
        key = get_key()
        if key == 'q':
            stop_event.set()
            break
        collector.update_controls(key)

def main():
    # --- CONFIGURATION ---
    SAVE_FREQUENCY_HZ = 50
    SAVE_INTERVAL_S = 1.0 / SAVE_FREQUENCY_HZ

    real_path = os.path.dirname(os.path.realpath(__file__))
    DATA_PATH = os.path.dirname(real_path) + '/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets' 
    SERIAL_PORT = "/dev/ttyUSB0"
    MAX_STEERING = 7
    
    # --- Initialize collector and serial port ---
    try:
        data_collector = DataCollector(base_path=DATA_PATH, max_steering=MAX_STEERING)
        ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
        time.sleep(1) # Wait for serial to initialize
    except Exception as e:
        print(f"Error during initialization: {e}")
        return

    # --- Setup and start keyboard listener thread ---
    stop_event = threading.Event()
    listener_thread = threading.Thread(target=keyboard_listener, args=(data_collector, stop_event), daemon=True)
    listener_thread.start()

    # --- Main Loop ---
    print("\n--- Starting Time-Based Data Collection ---")
    print(f"Saving data at {SAVE_FREQUENCY_HZ} Hz. Press 'q' to quit.")
    print("---------------------------------------------")
    
    try:
        while not stop_event.is_set():
            loop_start_time = time.time()
            
            # Log the current control data
            data_collector.log_data()

            # Get current control values to send
            control_values = data_collector.get_control_values()

            # Format and send the message via serial
            message = f"s{control_values['steering']}l{control_values['left_speed']}r{control_values['right_speed']}\n"
            ser.write(message.encode())
            
            # Print status
            status = f"Steering: {control_values['steering']:>2}, Speed: {control_values['left_speed']:>4}"
            sys.stdout.write('\r' + status)
            sys.stdout.flush()
            
            # Wait for the next cycle
            elapsed_time = time.time() - loop_start_time
            sleep_time = max(0, SAVE_INTERVAL_S - elapsed_time)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nProgram interrupted (Ctrl+C).")
    finally:
        stop_event.set() # Signal the listener thread to exit
        # Gracefully stop the robot and clean up
        print("\nStopping robot...")
        ser.write(b"s0l0r0\n")
        time.sleep(0.1)
        ser.close()
        data_collector.cleanup()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
