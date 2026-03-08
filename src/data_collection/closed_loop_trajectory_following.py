#!/usr/bin/env python
import os
import time
import datetime
import csv
import serial
import sys
import tty
import termios
import threading
import math
import rospy
from sensor_msgs.msg import NavSatFix

# --- Helper function for non-blocking keyboard input on Linux/macOS ---
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- GPS Math Helper Functions ---
def haversine_distance(lat1, lon1, lat2, lon2):
    """ Calculate distance between two GPS coordinates in meters. """
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """ Calculate bearing (heading) from point 1 to point 2 in degrees. """
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def normalize_angle(angle):
    """ Normalize an angle to be between -180 and 180 degrees. """
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

class VehicleState:
    """ Thread-safe class to store the vehicle's live state. """
    def __init__(self):
        self.lock = threading.Lock()
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0
        self.last_pos = None

    def update_gps(self, lat, lon):
        with self.lock:
            current_pos = (lat, lon)
            if self.last_pos and haversine_distance(self.last_pos[0], self.last_pos[1], lat, lon) > 0.1:
                # Estimate heading based on movement
                self.heading = calculate_bearing(self.last_pos[0], self.last_pos[1], lat, lon)
            self.latitude = lat
            self.longitude = lon
            self.last_pos = current_pos
            
    def get_state(self):
        with self.lock:
            return self.latitude, self.longitude, self.heading

class TrajectoryFollower:
    """ Manages the state and logic for following a trajectory. """
    def __init__(self, max_steering):
        # --- Controller Gains and Parameters ---
        self.STEERING_KP = 1.5  # Proportional gain for steering. TUNE THIS VALUE!
        self.MAX_STEERING = max_steering
        self.SPEED_STEP = 10
        self.MAX_SPEED = 100
        
        # --- Shared State ---
        self.lock = threading.Lock()
        self.mode = 'AUTO'
        self.steering = 0
        self.speed = 0

    def switch_mode(self):
        with self.lock:
            self.mode = 'MANUAL' if self.mode == 'AUTO' else 'AUTO'
            print(f"\nSwitched to [{self.mode}] mode.")

    def calculate_auto_controls(self, current_lat, current_lon, current_heading, target_lat, target_lon, target_speed):
        """ Calculates steering and speed to follow the trajectory. """
        with self.lock:
            if self.mode == 'MANUAL': return

            # Calculate the required heading to get to the next target waypoint
            target_bearing = calculate_bearing(current_lon, current_lat, target_lon, target_lat)
            
            # Calculate the difference between where we are heading and where we should be heading
            heading_error = normalize_angle(target_bearing - current_heading)

            # --- Proportional Controller for Steering ---
            # The new steering angle is proportional to the heading error.
            steering_adjustment = self.STEERING_KP * heading_error
            
            # Clamp the steering to the maximum allowed values
            self.steering = max(-self.MAX_STEERING, min(self.MAX_STEERING, steering_adjustment))
            
            # For now, simply use the speed from the log file
            self.speed = target_speed

    def update_from_keyboard(self, key):
        """ Updates controls from keyboard if in MANUAL mode. """
        with self.lock:
            if self.mode != 'MANUAL': return
            if key == 'w': self.speed = min(self.MAX_SPEED, self.speed + self.SPEED_STEP)
            elif key == 's': self.speed = max(-self.MAX_SPEED, self.speed - self.SPEED_STEP)
            elif key == 'a': self.steering = max(-self.MAX_STEERING, self.steering - 1)
            elif key == 'd': self.steering = min(self.MAX_STEERING, self.steering + 1)
            elif key == 'x': self.speed, self.steering = 0, 0
    
    def get_final_controls(self):
        with self.lock:
            return { "steering": self.steering, "speed": self.speed, "mode": self.mode }

def keyboard_listener(follower, stop_event):
    while not stop_event.is_set():
        key = get_key()
        if key == 'q': stop_event.set(); break
        elif key == 'm': follower.switch_mode()
        else: follower.update_from_keyboard(key)

def gps_callback(data, vehicle_state):
    """ ROS callback function to update vehicle state from NavSatFix message. """
    # STATUS_FIX means the GPS has a 2D or 3D fix.
    if data.status.status >= 0:
        vehicle_state.update_gps(data.latitude, data.longitude)

def find_latest_dataset(base_path):
    if not os.path.isdir(base_path): return None, "Base directory not found"
    dirs = [d for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))]
    return sorted(dirs)[-1] if dirs else (None, "No dataset directories found")

def main():
    # --- CONFIGURATION ---
    REPLAY_FREQUENCY_HZ = 50
    CONTROL_SERIAL_PORT = "/dev/ttyUSB0"
    BAUD_RATE = 115200
    MAX_STEERING = 7

    # --- SETUP ---
    follower = TrajectoryFollower(max_steering=MAX_STEERING)
    vehicle_state = VehicleState()
    base_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'camera_perception_pkg', 'camera_perception_pkg', 'lib', 'Collected_Datasets')
    
    source_subdir, error = find_latest_dataset(base_path)
    if error: return print(error)
    source_log_path = os.path.join(base_path, source_subdir, "driving_log.csv")

    timestamp = datetime.datetime.now().strftime("%Y_%m_%d_%H%M%S_corrected")
    corrected_path = os.path.join(base_path, timestamp)
    os.makedirs(corrected_path, exist_ok=True)
    corrected_log_path = os.path.join(corrected_path, "driving_log.csv")
    print(f"✅ Reading trajectory from: {source_subdir}")
    print(f"💾 Saving corrected data to: {timestamp}")

    try:
        ser = serial.Serial(CONTROL_SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(1)
    except serial.SerialException as e: return print(f"Error opening control port: {e}")
    
    # --- ROS SETUP ---
    rospy.init_node('trajectory_follower', anonymous=True)
    rospy.Subscriber('/ublox_gps_node/fix', NavSatFix, lambda data: gps_callback(data, vehicle_state))
    print("✅ Subscribed to /ublox_gps_node/fix")

    # --- Start Background Threads ---
    stop_event = threading.Event()
    threading.Thread(target=keyboard_listener, args=(follower, stop_event), daemon=True).start()
    
    print("Waiting for first GPS fix from ROS topic...")
    time.sleep(2)

    # --- Main Control Loop ---
    try:
        with open(source_log_path, 'r') as source, open(corrected_log_path, 'w', newline='') as corrected:
            reader = csv.DictReader(source)
            writer = csv.writer(corrected)
            writer.writerow(['steering', 'left_speed', 'right_speed', 'latitude', 'longitude'])

            print("\n--- Starting Closed-Loop Trajectory Following ---")
            print("Press 'm' to toggle AUTO/MANUAL, 'q' to quit.")

            for row in reader:
                if stop_event.is_set() or rospy.is_shutdown(): break
                loop_start = time.time()
                
                target_lat, target_lon = float(row['latitude']), float(row['longitude'])
                target_speed = int(row['left_speed'])
                
                current_lat, current_lon, current_heading = vehicle_state.get_state()
                if current_lat == 0.0:
                    sys.stdout.write('\rWaiting for GPS fix...')
                    sys.stdout.flush()
                    time.sleep(0.5)
                    continue

                follower.calculate_auto_controls(current_lat, current_lon, current_heading, target_lat, target_lon, target_speed)
                
                controls = follower.get_final_controls()
                steering, speed = controls['steering'], controls['speed']
                
                message = f"s{int(steering)}l{speed}r{speed}\n"
                ser.write(message.encode())
                
                writer.writerow([steering, speed, speed, target_lat, target_lon])
                
                status = f"[{controls['mode']}] Tgt:({target_lat:.4f},{target_lon:.4f}) Cur:({current_lat:.4f},{current_lon:.4f}) Head:{current_heading:.1f}° Str:{steering:.1f}"
                sys.stdout.write('\r' + status)
                sys.stdout.flush()

                time.sleep(max(0, (1.0/REPLAY_FREQUENCY_HZ) - (time.time() - loop_start)))

    except Exception as e: print(f"\nAn error occurred: {e}")
    finally:
        stop_event.set()
        print("\n\nStopping device and saving corrected data...")
        ser.write(b"s0l0r0\n")
        time.sleep(0.1)
        ser.close()
        print("Done.")

if __name__ == "__main__":
    main()

