import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import math


class MockCameraPublisher(Node):
    def __init__(self):
        super().__init__('mock_camera_publisher')

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_image)

        self.width = 640
        self.height = 480
        self.tick = 0

        self.get_logger().info('Mock camera publisher started.')

    def publish_image(self):
        # Generate a synthetic BGR image with time-varying color
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Slowly cycle through colors so avg_color changes visibly
        b = int(127 + 127 * math.sin(self.tick * 0.05))
        g = int(127 + 127 * math.sin(self.tick * 0.05 + 2.0))
        r = int(127 + 127 * math.sin(self.tick * 0.05 + 4.0))

        img[:, :, 0] = b
        img[:, :, 1] = g
        img[:, :, 2] = r

        # Build the ROS Image message manually (no cv_bridge needed)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = self.width * 3  # bytes per row
        msg.data = img.tobytes()

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published image: B={b}, G={g}, R={r}'
        )
        self.tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()