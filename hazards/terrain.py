
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

import numpy as np
import cv2

class ElevationEstimator(Node):
    def __init__(self):
        super().__init__('elevation_estimator')
        # parameters
        self.declare_parameter('topic', '/camera/image_raw')       # meters per cell
        self.topic = self.get_parameter('topic').value

        self.subscription = self.create_subscription(
            Image, self.topic, self.camera_callback, 10)
        
        # Publisher
        self.avg_color_pub = self.create_publisher(
            Float32MultiArray,
            '/camera/avg_color',
            10
        )
        
        self.get_logger().info('Elevation estimator started')


    def camera_callback(self, msg: Image):
        # Convert ROS Image message to OpenCV format
        np_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Compute the average color value per channel (BGR)
        avg_color = np.mean(np_image, axis=(0, 1))
        self.get_logger().info(
            f'Average color (BGR): B={avg_color[0]:.2f}, G={avg_color[1]:.2f}, R={avg_color[2]:.2f}'
        )

        pass
        


def main(args=None):
    rclpy.init(args=args)
    node = ElevationEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Hazard Grid Map Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()