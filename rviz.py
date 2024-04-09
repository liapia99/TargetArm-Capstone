import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import numpy as np

def plot_lidar_data(scan_data):
    laser_projector = LaserProjection()

    # Convert scan data to LaserScan message
    scan_msg = LaserScan()
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.angle_min = -np.pi
    scan_msg.angle_max = np.pi
    scan_msg.angle_increment = 2 * np.pi / len(scan_data)
    scan_msg.time_increment = 0  # Not used
    scan_msg.scan_time = 0  # Not used
    scan_msg.range_min = 0.0
    scan_msg.range_max = 10.0  # Adjust this based on your lidar's range
    scan_msg.ranges = [measurement[2] for measurement in scan_data]

    # Convert LaserScan message to PointCloud2
    point_cloud = laser_projector.projectLaser(scan_msg)

    # Publish PointCloud2 message
    pub = rospy.Publisher('/lidar_point_cloud', PointCloud2, queue_size=10)
    pub.publish(point_cloud)

if __name__ == "__main__":
    rospy.init_node('lidar_to_point_cloud', anonymous=True)
    lidar_data = get_data()
    if lidar_data:
        plot_lidar_data(lidar_data)
    rospy.spin()
