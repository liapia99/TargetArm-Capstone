import numpy as np
from rplidar import RPLidar, RPLidarException
import serial
import time

arduino_port = "/dev/ttyACM0"
arduino_baudrate = 115200
ser = serial.Serial(arduino_port, arduino_baudrate, timeout=1)

def get_data():
    try:
        lidar = RPLidar('/dev/ttyUSB0')
        ser.write("Time Start\n".encode()) 
        print("Time Start")  # Letting Arduino know that timer started
        time.sleep(1)  # Add a delay after writing to Arduino
        start_time = time.time()
        shortest_distance = float('inf')
        shortest_distance_time = 0

        for scan in lidar.iter_scans(max_buf_meas=200):
            current_time = time.time() - start_time
            if current_time >= 60:
                break
            
            distances = [measurement[2] for measurement in scan]
            min_distance = min(distances) / 25.4  
            if min_distance < shortest_distance:
                shortest_distance = min_distance
                shortest_distance_time = current_time

        lidar.stop()
        lidar.disconnect()
        
        print(f"Shortest Distance: {shortest_distance:.2f} inches")
        print(f"Time when shortest distance was achieved: {shortest_distance_time:.2f} seconds")
        
        return shortest_distance, shortest_distance_time
    except RPLidarException as e:
        print(f"Error: {e}")
        return None, 0

def check_distance(shortest_distance, shortest_distance_time):
    if shortest_distance is not None: 
        data_string = f"{shortest_distance:.2f},{shortest_distance_time:.2f}\n"
        print(f"Sending Data: {data_string.strip()}")
        ser.write(data_string.encode())
    else:
        print("No shortest distance available.")

if __name__ == "__main__":
    shortest_distance, shortest_distance_time = get_data()

    if shortest_distance is not None:
        check_distance(shortest_distance, shortest_distance_time)
    
    try:
        ser.close()
    except serial.serialutil.SerialException as e:
        print(f"Error closing serial port: {e}")
