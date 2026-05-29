
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np


class ElevationEstimator(Node):
    def __init__(self):
        super().__init__('elevation_estimator')
        # parameters
        self.declare_parameter('topic', '/imu/data')       # meters per cell
        self.topic = self.get_parameter('topic').value

        self.subscription = self.create_subscription(
            Imu, self.topic, self.imu_callback, 10)
        
        # State variables
        self.velocity_z = 0.0       # Vertical velocity (m/s)
        self.elevation = 0.0        # Elevation change (m)
        self.last_time = None
        self.gravity = np.array([0.0, 0.0, 9.81])  # Gravity in world frame
        
        # Simple high-pass filter state (to mitigate drift)
        self.alpha = 0.98  # Complementary filter weight
        
        self.get_logger().info('Elevation estimator started')

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion (x, y, z, w) to rotation matrix."""
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),       1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x**2 + y**2)]
        ])

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Compute dt
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        
        if dt <= 0 or dt > 0.5:  # Skip bad dt values
            return
        
        # Step 1: Get orientation as rotation matrix
        q = [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w]
        R = self.quaternion_to_rotation_matrix(q)
        
        # Step 2: Transform acceleration from body frame to world frame
        accel_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        accel_world = R @ accel_body
        
        # Step 3: Remove gravity to get motion-only acceleration
        accel_motion = accel_world - self.gravity
        
        # Step 4: Extract vertical (z) component
        az = accel_motion[2]
        
        # Step 5: Apply threshold to reduce noise (dead zone)
        if abs(az) < 0.05:  # Threshold below noise floor
            az = 0.0
        
        # Step 6: Integrate for velocity and position
        self.velocity_z += az * dt
        self.elevation += self.velocity_z * dt
        
        # Step 7: Apply velocity decay to fight drift (ZUPT-like)
        # This assumes the robot isn't constantly ascending
        self.velocity_z *= 0.999  # Gentle decay
        
        self.get_logger().info(
            f'Elevation: {self.elevation:.3f} m | '
            f'Vel_z: {self.velocity_z:.3f} m/s | '
            f'Accel_z: {az:.3f} m/s²')


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