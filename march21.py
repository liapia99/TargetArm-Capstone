import numpy as np
from rplidar import RPLidar, RPLidarException

class LiDARScanner:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port

    def get_data(self, max_scans=10):
        try:
            lidar = RPLidar(self.port)
            scan_count = 0
            for scan in lidar.iter_scans(max_buf_meas=500):
                yield scan
                scan_count += 1
                if scan_count >= max_scans:
                    break
            lidar.stop()
            lidar.disconnect()
        except RPLidarException as e:
            print(f"Error: {e}")

    def calculate_phi(self, seconds=2):
        # Calculate phi for every 2 seconds (30 degrees)
        degrees_per_second = 30 / seconds
        phi = np.radians(degrees_per_second)
        return phi
      
    def plot_lidar_data(self, scan_data, phi):
        angles = np.array([np.radians(measurement[1]) for measurement in scan_data])
        distances = np.array([measurement[2] for measurement in scan_data])
        phi = np.array([measurement[3] for measurement in phi])

        x = distances * np.cos(angles) * np.cos(phi)
        y = distances * np.sin(angles)
        z = distances * np.cos(angles) * np.sin(phi)

    def check_distance(self, scan_data):
        with open('lidar_results.txt', 'w') as file:
            for measurement in scan_data and phi:
                angle = measurement[1]
                distance = measurement[2] / 304.8  # millimeters to feet
                phi = measurement[3]


                if distance < 5:
                    file.write(f"Object detected at {distance:.2f} feet.\n")
                    print(f"Object detected at {distance:.2f} feet.")
        print(f"Phi is: {phi}")



if __name__ == "__main__":
    lidar_scanner = LiDARScanner()
    lidar_data = lidar_scanner.get_data(max_scans=10)

    if lidar_data:
        phi = lidar_scanner.calculate_phi()
        for scan_data in lidar_data:
            lidar_scanner.plot_lidar_data(scan_data, phi)
            lidar_scanner.check_distance(scan_data, phi)
