import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import 3D plotting tools
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

plot_interval = 7  # Plot every 7 iterations
max_iterations = 1000000

for i in range(max_iterations):
    if (i % plot_interval == 0):
        x_coordinates = []  # Initialize as a Python list
        y_coordinates = []  # Initialize as a Python list
        z_coordinates = []  # Initialize as a Python list
        intensity = []  # Initialize as a Python list
    print(i)
    current_data = get_data()
    for point in current_data:
        if len(point) >= 3 and point[0] == 15:
            x_coordinates.append(point[1] * np.cos(np.radians(point[0])))  # Convert polar to Cartesian x
            y_coordinates.append(point[1] * np.sin(np.radians(point[0])))  # Convert polar to Cartesian y
            z_coordinates.append(point[2] * 0.3048)  # Convert distance from feet to meters (1 ft = 0.3048 meters)
            intensity.append(point[2])  # Use distance (or another appropriate value) as intensity

    if i % plot_interval == 0 and x_coordinates and y_coordinates and z_coordinates:
        plt.clf()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(x_coordinates, y_coordinates, z_coordinates, c=intensity, cmap='viridis', marker='o', alpha=0.5)
        ax.set_xlabel('X Coordinates')
        ax.set_ylabel('Y Coordinates')
        ax.set_zlabel('Z Coordinates (meters)')
        plt.title('RPLidar Data (3D Cartesian Plot)')
        plt.colorbar(sc, label='Intensity')
        plt.grid(True)
        plt.pause(0.1)

plt.show(block=False)
plt.pause(10)  # Adjust as needed to keep the plot visible for some time
plt.close()
