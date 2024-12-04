import numpy as np
import socket
import struct
import time
import serial
from pyvelodyne import VelodyneReader

# Arduino serial connection setup
arduino_port = "/dev/ttyACM0"
arduino_baudrate = 9600
ser = serial.Serial(arduino_port, arduino_baudrate, timeout=1)

def get_data():
    # Create a UDP socket for Velodyne VLP-16
    UDP_IP = "192.168.1.201"  # Change to your VLP-16's IP
    UDP_PORT = 2368  # Default VLP-16 port
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    # Read the data from the VLP-16
    lidar_reader = VelodyneReader(sock)
    
    start_time = time.time()  # Start the timer
    shortest_distance = float('inf')  # Initialize shortest_distance to infinity
    shortest_distance_time = 0  # Initialize shortest_distance_time

    try:
        for scan in lidar_reader.iter_points():
            current_time = time.time() - start_time  # Calculate current time
            if current_time >= 120:  # Check if 120 seconds have passed
                break
            
            # Extract points in XYZ format (x, y, z)
            distances = [point[0]**2 + point[1]**2 + point[2]**2 for point in scan]
            min_distance = min(distances) ** 0.5  # Find the shortest distance (Euclidean distance)
            if min_distance < shortest_distance:
                shortest_distance = min_distance
                shortest_distance_time = current_time

        # Close the socket
        sock.close()

        print(f"Shortest Distance: {shortest_distance:.2f} meters")
        print(f"Time when shortest distance was achieved: {shortest_distance_time:.2f} seconds")
        
        return scan
    except Exception as e:
        print(f"Error: {e}")
        sock.close()
        return []

def check_distance(scan_data, shortest_distance_time):
    if scan_data: 
        # Find the first distance (or another relevant distance)
        distance = scan_data[0][0]  # Assuming [0] is the x-axis distance
        print(f"Sending Distance: {distance:.2f} + Z_angle: {shortest_distance_time:.2f}")  # Debug print
        ser.write(f"{distance},{shortest_distance_time}\n".encode())  # Send the distance and Z angle over serial
    else:
        print("No Lidar data available.")

if __name__ == "__main__":
    lidar_data = get_data()

    if lidar_data:
        shortest_distance_time = ...  # Calculate this based on your logic (could be directly taken from get_data)
        check_distance(lidar_data, shortest_distance_time)  # Pass shortest_distance_time

    try:
        ser.close()
    except serial.serialutil.SerialException as e:
        print(f"Error closing serial port: {e}")
