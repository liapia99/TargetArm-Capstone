import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar, RPLidarException
import Jetson.GPIO as GPIO
import time

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

    plt.figure(figsize=(10, 10))
    plt.scatter(x, y, s=5, color='blue', alpha=0.5)
    plt.title('X and Y Graph')
    plt.xlabel('X (inches)')
    plt.ylabel('Y (inches)')
    plt.grid(True)
    plt.show()

def check_distance(scan_data):
    with open('lidar_results.txt', 'w') as file:
        for measurement in scan_data:
            angle = measurement[1]
            distance = measurement[2] / 25.4  # Convert distance from millimeters to inches
            if distance < 12:
                file.write(f"Object detected at {distance:.2f} inches.\n")
                print(f"Object detected at {distance:.2f} inches.")

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)  # Set GPIO pin numbering mode to BOARD
    GPIO.setup(green_led_pin, GPIO.OUT)  # Set green LED pin as output
    GPIO.setup(red_led_pin, GPIO.OUT)  # Set red LED pin as output

def control_leds(distances):
    if any(distance < 12 for distance in distances):        
        GPIO.output(red_led_pin, GPIO.HIGH)
        GPIO.output(green_led_pin, GPIO.LOW)
    else:
        GPIO.output(green_led_pin, GPIO.HIGH)
        GPIO.output(red_led_pin, GPIO.LOW)

if __name__ == "__main__":
    green_led_pin = 12  # GPIO pin for green LED
    red_led_pin = 16  # GPIO pin for red LED

    setup_gpio()

    lidar_data = get_data()

    if lidar_data:
        plot_lidar_data(lidar_data)
        check_distance(lidar_data)
        distances = [measurement[2] / 25.4 for measurement in lidar_data]  # Convert distances to inches
        control_leds(distances)
