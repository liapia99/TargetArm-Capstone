import numpy as np
from rplidar import RPLidar, RPLidarException

class LiDARScanner:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port

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

   # def calculate_phi(self, seconds=2):
        # Calculate phi for every 2 seconds (30 degrees)
    #    degrees_per_second = 30 / seconds
     #   phi = np.radians(degrees_per_second)
     #   return phi
      
    def plot_lidar_data(self, scan_data):
        angles = np.array([np.radians(measurement[1]) for measurement in scan_data])
        distances = np.array([measurement[2] for measurement in scan_data])
        phi = np.array([measurement[3] for measurement in scan_data])

        x = distances * np.cos(angles) * np.cos(30)
        y = distances * np.sin(angles)
        z = distances * np.cos(angles) * np.sin(30)

    def check_distance(self, scan_data):
        with open('lidar_results.txt', 'w') as file:
            for measurement in scan_data:
                angle = measurement[1]
                distance = measurement[2] / 304.8  # millimeters to feet
                phi = measurement[3]


                if distance < 5:
                    file.write(f"Object detected at {distance:.2f} feet.\n")
                    print(f"Object detected at {distance:.2f} feet.")
                    print(f"Phi is: {phi}")


if __name__ == "__main__":
    lidar_data = get_data()

    if lidar_data:
        plot_lidar_data(lidar_data)
        check_distance(lidar_data)
