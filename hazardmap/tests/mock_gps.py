import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import math


class MockGpsPublisher(Node):
    def __init__(self):
        super().__init__('mock_gps_publisher')

        self.publisher = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            10
        )

        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_gps)

        # Starting position (e.g., somewhere in New York)
        self.latitude = 40.748817
        self.longitude = -73.985428
        self.altitude = 10.0

        self.tick = 0
        self.get_logger().info('Mock GPS publisher started.')

    def publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'

        # Simulate slow drift (small circular path)
        self.latitude += 0.00001 * math.sin(self.tick * 0.1)
        self.longitude += 0.00001 * math.cos(self.tick * 0.1)
        self.altitude = 10.0 + 0.5 * math.sin(self.tick * 0.05)

        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude

        # Status: GPS fix
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Covariance (diagonal, in meters^2)
        msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 2.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}'
        )
        self.tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockGpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()