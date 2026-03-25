import serial
import time
import os
import csv

def find_latest_dataset(base_path):
    """Scans the base_path for directories and returns the name of the most recent one."""
    if not os.path.isdir(base_path):
        return None, f"Error: Base directory not found at '{base_path}'"
    
    all_dirs = [d for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))]
    
    if not all_dirs:
        return None, f"Error: No dataset directories found in '{base_path}'"
    
    latest_dir = sorted(all_dirs)[-1]
    return latest_dir, None

def main():
    # --- CONFIGURATION ---
    LOG_FILE_NAME = "driving_log.csv"
    SERIAL_PORT = "/dev/ttyUSB0"
    BAUD_RATE = 115200
    REPLAY_DELAY_S = 0.02 # 100ms delay

    # --- SCRIPT LOGIC ---
    real_path = os.path.dirname(os.path.realpath(__file__))
    data_path_base = os.path.dirname(real_path) + '/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets'

    # Automatically find the latest dataset
    latest_data_subdir, error = find_latest_dataset(data_path_base)
    if error:
        print(error)
        return
    
    #print(f"✅ Automatically selected the latest dataset: '{latest_data_subdir}'")
    dataset_name = "2025_09_27_110517"
    replay_data_path = os.path.join(data_path_base, dataset_name)
    #replay_data_path = os.path.join(data_path_base, latest_data_subdir)
    log_file_path = os.path.join(replay_data_path, LOG_FILE_NAME)

    if not os.path.exists(log_file_path):
        print(f"Error: Log file '{LOG_FILE_NAME}' not found inside '{replay_data_path}'")
        return

    # Initialize serial connection
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Opened serial port {SERIAL_PORT}")
    time.sleep(2)

    try:
        print(f"Starting replay of data from {latest_data_subdir}...")
        
        with open(log_file_path, mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            
            for row in csv_reader:
                try:
                    steering = int(row['steering'])
                    left_speed = int(row['left_speed'])
                    # Enforce that right_speed is identical to left_speed during replay
                    right_speed = left_speed

                    message = f"s{steering}l{left_speed}r{right_speed}\n"
                    ser.write(message.encode())
                    print(f"Replaying -> Sent: {message.strip()}")
                    time.sleep(REPLAY_DELAY_S)

                except (ValueError, KeyError) as e:
                    print(f"Skipping row due to error: {e}. Row: {row}")
                    continue

        print("\nReplay finished.")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        print("Stopping device...")
        stop_message = b"s0l0r0\n"
        ser.write(stop_message)
        time.sleep(0.1)
        ser.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()