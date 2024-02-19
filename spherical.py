import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar, RPLidarException

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

plot_interval = 1  # Plot every iteration
max_iterations = 10000000

for i in range(max_iterations):
    if (i % plot_interval == 0):
        azimuths = []  # Azimuthal angle
        polar_angles = []  # Polar angle
        distances = []  # Radius
        intensity = []  # Intensity

    print(i)
    current_data = get_data()
    for point in current_data:
        x = point[2] * np.cos(np.radians(point[1]))  # Convert to Cartesian coordinates
        y = point[2] * np.sin(np.radians(point[1]))
        z = 0  # Assuming lidar data is 2D (no height information)

        azimuth = np.arctan2(y, x)  # Azimuthal angle
        polar_angle = np.arccos(z / np.sqrt(x**2 + y**2 + z**2))  # Polar angle
        radius = np.sqrt(x**2 + y**2 + z**2)  # Radius

        azimuths.append(azimuth)
        polar_angles.append(polar_angle)
        distances.append(radius)
        intensity.append(point[1])  # Use distance (or another appropriate value) as intensity

    if i % plot_interval == 0 and azimuths and polar_angles and distances:
        plt.clf()
        ax = plt.subplot(111, polar=True)
        sc = ax.scatter(azimuths, distances, c=intensity, cmap='viridis', marker='o', alpha=0.5)
        plt.title('RPLidar Data (Spherical Coordinates)')
        plt.colorbar(sc, label='Intensity')
        plt.grid(True)
        plt.pause(0.1)

plt.show(block=False)
plt.pause(10)  # Adjust as needed to keep the plot visible for some time
plt.close()
