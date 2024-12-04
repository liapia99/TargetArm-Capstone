import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',  # Topic name
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Convert PointCloud2 message to (x, y, z) points
        point_list = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Process each point (x, y, z) - e.g., object detection
        for point in point_list:
            x, y, z = point
            self.get_logger().info(f'Point: x={x}, y={y}, z={z}')
            # Here you can add your object detection algorithm

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    rclpy.spin(pointcloud_subscriber)
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
