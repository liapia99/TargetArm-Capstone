import numpy as np
from rplidar import RPLidar, RPLidarException
import serial
import time

arduino_port = "/dev/ttyACM0"
arduino_baudrate = 9600
ser = serial.Serial(arduino_port, arduino_baudrate, timeout=1)

def get_data():
    try:
        lidar = RPLidar('/dev/ttyUSB0')
        start_time = time.time()  # Start the timer
        shortest_distance = float('inf')  # Initialize shortest_distance to infinity
        shortest_distance_time = 0  # Initialize shortest_distance_time

        for scan in lidar.iter_scans(max_buf_meas=500):
            current_time = time.time() - start_time  # Calculate current time
            if current_time >= 120:  # Check if 120 seconds have passed
                break
            
            distances = [measurement[2] for measurement in scan]
            min_distance = min(distances) / 25.4  
            
            if min_distance < 12:  # Check if distance is less than 12 inches
                print(f"Object detected at {min_distance:.2f} inches.")
                lidar.stop()
                lidar.disconnect()
                send_command_to_arduino(min_distance, current_time)
                break
            
            if min_distance < shortest_distance:
                shortest_distance = min_distance
                shortest_distance_time = current_time

        print(f"Shortest Distance: {shortest_distance:.2f} inches")
        print(f"Time when shortest distance was achieved: {shortest_distance_time:.2f} seconds")
        
        return scan
    except RPLidarException as e:
        print(f"Error: {e}")
        return []

def send_command_to_arduino(distance, shortest_distance_time):
    try:
        print(f"Sending Distance: {distance:.2f} + Time: {shortest_distance_time:.2f}")
        ser.write(f"{distance},{shortest_distance_time}\n".encode())  # Send the distance and time over serial
    except serial.serialutil.SerialException as e:
        print(f"Error sending command to Arduino: {e}")

def plot_lidar_data(scan_data):
    angles = np.array([np.radians(measurement[1]) for measurement in scan_data])
    distances = np.array([measurement[2] for measurement in scan_data])

    x = distances * np.cos(angles)
    y = distances * np.sin(angles)

if __name__ == "__main__":
    lidar_data = get_data()

    try:
        ser.close()
    except serial.serialutil.SerialException as e:
        print(f"Error closing serial port: {e}")
