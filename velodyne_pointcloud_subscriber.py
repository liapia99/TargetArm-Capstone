import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class VelodynePointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('velodyne_pointcloud_subscriber')

        # Subscribe to the Velodyne PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',  # Topic name
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Convert PointCloud2 message to a numpy array
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc_data))

        # Perform object detection or any processing you need here
        self.detect_objects(points)

    def detect_objects(self, points):
        # Create an Open3D point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Visualize the point cloud (optional, for debugging)
        o3d.visualization.draw_geometries([pcd])

        # Apply a simple clustering algorithm to detect objects (e.g., DBSCAN)
        labels = self.dbscan_clustering(pcd)

        # Visualize clusters (optional)
        self.visualize_clusters(pcd, labels)

    def dbscan_clustering(self, pcd):
        # Apply DBSCAN for clustering
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        labels = np.array(pcd.cluster_dbscan(eps=0.3, min_points=10, print_progress=True))
        return labels

    def visualize_clusters(self, pcd, labels):
        colors = plt.get_cmap("tab20")(labels / max(labels))  # Use different colors for each cluster
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])  # RGB values
        o3d.visualization.draw_geometries([pcd])

def main(args=None):
    rclpy.init(args=args)
    node = VelodynePointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
