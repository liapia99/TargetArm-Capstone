import numpy as np
from rplidar import RPLidar, RPLidarException
import serial

arduino_port = "/dev/ttyACM0"  # Change this to your Arduino port - Linux: /ttyACM0 and Mac - /cu.usbmodem2101
arduino_baudrate = 9600
ser = serial.Serial(arduino_port, arduino_baudrate, timeout=1)

def get_data():
    try:
        lidar = RPLidar('/dev/ttyUSB0')
        for scan in lidar.iter_scans(max_buf_meas=500):
            break
        lidar.stop()
        lidar.disconnect()
        return scan
    except RPLidarException as e:
        print(f"Error: {e}")
        return []

def plot_lidar_data(scan_data):
    angles = np.array([np.radians(measurement[1]) for measurement in scan_data])
    distances = np.array([measurement[2] for measurement in scan_data])


    x = distances * np.cos(angles)
    y = distances * np.sin(angles)

    

def check_distance(scan_data):
    with open('lidar_results.txt', 'w') as file:
        for measurement in scan_data:
            angle = measurement[1]
            distance = measurement[2] / 25.4  # Convert distance from millimeters to inches
            file.write(f"Object detected at {distance:.2f} inches.\n")
            print(f"{distance:.2f} ")

if __name__ == "__main__":
    lidar_data = get_data()

    if lidar_data:
        plot_lidar_data(lidar_data)
        check_distance(lidar_data)
    ser.close()
    try:
        ser.close()
    except serial.serialutil.SerialException as e:
        print(f"Error closing serial port: {e}")