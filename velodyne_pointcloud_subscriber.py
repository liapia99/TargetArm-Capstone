import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from velodyne_msgs.msg import VelodynePacket
from sensor_msgs import point_cloud2
import std_msgs.msg
import math

class VelodynePointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('velodyne_pointcloud_subscriber')

        # Subscriber for Velodyne UDP packets
        self.create_subscription(VelodynePacket, '/velodyne_packets', self.velodyne_callback, 10)

        # Subscriber for the PointCloud2 data
        self.create_subscription(PointCloud2, '/velodyne_points', self.point_cloud_callback, 10)

    def velodyne_callback(self, msg):
        # This callback will process the Velodyne UDP packet data.
        # Typically, we use the driver to convert packets to PointCloud2, 
        # but here we directly show how you can access packet info.
        self.get_logger().info('Received a Velodyne packet')

        # Process the packet and convert to PointCloud2 here (you would typically use velodyne_driver)
        # For now, this is just a placeholder.
        pass

    def point_cloud_callback(self, msg):
        # Callback to process PointCloud2 data.
        # Extract points from the PointCloud2 message
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(gen)
        
        # Print out the distances of objects detected
        for point in points:
            x, y, z = point[0], point[1], point[2]
            distance = math.sqrt(x**2 + y**2 + z**2)
            self.get_logger().info(f'Detected object at distance: {distance} meters')

def main(args=None):
    rclpy.init(args=args)

    # Create the node and start receiving messages
    node = VelodynePointCloudSubscriber()

    # Spin the node to keep it alive and handle incoming messages
    rclpy.spin(node)

    # Clean up and shut down the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
