import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
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
tilt_angles = np.arange(-30, 31, 1)  # Tilt angles from -30 to 30 degrees

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter([], [], [], c=[], cmap='viridis', marker='o', alpha=0.5)
ax.set_xlabel('Azimuthal Angle')
ax.set_ylabel('Polar Angle')
ax.set_zlabel('Radius')
plt.title('RPLidar Data (Spherical Coordinates)')

# Create and display color bar
cbar = plt.colorbar(sc, label='Intensity')
plt.show(block=False)

for i in range(max_iterations):
    if (i % plot_interval == 0):
        azimuths = []  # Azimuthal angle
        polar_angles = []  # Polar angle
        distances = []  # Radius
        intensity = []  # Intensity

    print(i)
    current_data = get_data()
    for idx, point in enumerate(current_data):
        x = point[2] * np.cos(np.radians(point[1]))  # Convert to Cartesian coordinates
        y = point[2] * np.sin(np.radians(point[1]))
        tilt_angle = tilt_angles[idx]  # Get corresponding tilt angle
        z = point[2] * np.tan(np.radians(tilt_angle))  # Compute z-coordinate based on tilt angle

        azimuth = np.arctan2(y, x)  # Azimuthal angle
        polar_angle = np.arccos(z / np.sqrt(x**2 + y**2 + z**2))  # Polar angle
        radius = np.sqrt(x**2 + y**2 + z**2)  # Radius

        azimuths.append(azimuth)
        polar_angles.append(polar_angle)
        distances.append(radius)
        intensity.append(point[1])  # Use distance (or another appropriate value) as intensity

    if i % plot_interval == 0 and azimuths and polar_angles and distances:
        sc.remove()  # Remove the previous scatter plot
        sc = ax.scatter(azimuths, polar_angles, distances, c=intensity, cmap='viridis', marker='o', alpha=0.5)
        cbar.update_normal(sc)
        plt.pause(0.1)

plt.pause(10)  # Adjust as needed to keep the plot visible for some time
plt.close()
