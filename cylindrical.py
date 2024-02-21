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

plot_interval = 1 
max_iterations = 10000000


for i in range(max_iterations):
    if i % plot_interval == 0:
        angles = []  # Initialize as a Python list
        distances = []  # Initialize as a Python list
        print(i)
        current_data = get_data()
      
        for point in current_data:
            # Filter points based on angle and depth range
            if angle_min <= np.radians(point[1]) <= angle_max and point[2] <= max_depth:
                angles.append(np.radians(point[1]))  # Convert angle to radians
                distances.append(point[2])
                

        if angles and distances:
            plt.clf()
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(angles, distances, cmap='greens', marker='o', alpha=0.5)
            ax.set_xlabel('Theta')
            ax.set_ylabel('Distance')
            plt.title('RPLidar Data (Cylindrical Plot)')
            plt.show(block=False)
            plt.pause(0.1)

plt.pause(10)  # Adjust as needed to keep the plot visible for some time
plt.close()
