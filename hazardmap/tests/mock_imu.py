
#!/usr/bin/env python3
"""
Mock IMU Publisher Node
=======================
A ROS2 debug node that publishes simulated sensor_msgs/Imu data
for testing the HazardMap pipeline.

Modes:
  - "static"    : Constant gravity vector (flat, no tilt)
  - "tilt"      : Sinusoidal tilt oscillation around one axis
  - "spin"      : Continuous rotation (gyro data ramps)
  - "random"    : Random noise around gravity
  - "drop"      : Simulates free-fall (zero accel) briefly

Usage:
  ros2 run hazardmap mock_imu --ros-args -p mode:=tilt -p tilt_amplitude_deg:=20.0
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (radians) to geometry_msgs/Quaternion."""
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MockImuNode(Node):
    """
    Publishes simulated IMU data on /imu/data for debugging.
    """

    def __init__(self):
        super().__init__('mock_imu_node')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)            # Hz (typical IMU rate)
        self.declare_parameter('mode', 'tilt')                  # static|tilt|spin|random|drop
        self.declare_parameter('tilt_amplitude_deg', 20.0)      # max tilt in degrees
        self.declare_parameter('tilt_period_sec', 4.0)          # oscillation period
        self.declare_parameter('tilt_axis', 'pitch')            # roll|pitch|both
        self.declare_parameter('noise_stddev', 0.02)            # m/s^2 noise on accel
        self.declare_parameter('gyro_noise_stddev', 0.005)      # rad/s noise on gyro
        self.declare_parameter('drop_interval_sec', 10.0)       # time between drops
        self.declare_parameter('drop_duration_sec', 0.5)        # free-fall duration

        # Read parameters
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('publish_rate').value
        self.mode = self.get_parameter('mode').value
        self.tilt_amp = math.radians(self.get_parameter('tilt_amplitude_deg').value)
        self.tilt_period = self.get_parameter('tilt_period_sec').value
        self.tilt_axis = self.get_parameter('tilt_axis').value
        self.noise_std = self.get_parameter('noise_stddev').value
        self.gyro_noise_std = self.get_parameter('gyro_noise_stddev').value
        self.drop_interval = self.get_parameter('drop_interval_sec').value
        self.drop_duration = self.get_parameter('drop_duration_sec').value

        # Gravity constant
        self.G = 9.80665

        # Time tracking
        self.start_time = self.get_clock().now()
        self.msg_count = 0

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.imu_pub = self.create_publisher(Imu, self.topic, qos)

        # Timer
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.publish_imu)

        self.get_logger().info(
            f'Mock IMU started: mode="{self.mode}", rate={self.rate} Hz, '
            f'topic="{self.topic}", frame="{self.frame_id}"'
        )
        if self.mode == 'tilt':
            self.get_logger().info(
                f'  Tilt: amplitude={math.degrees(self.tilt_amp):.1f} deg, '
                f'period={self.tilt_period:.1f} s, axis={self.tilt_axis}'
            )

    # ======================================================================
    # PUBLISHER CALLBACK
    # ======================================================================
    def publish_imu(self):
        """Generate and publish a simulated Imu message."""
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        # Compute simulated values based on mode
        roll, pitch, yaw = 0.0, 0.0, 0.0
        angular_vel = Vector3(x=0.0, y=0.0, z=0.0)
        linear_accel = Vector3(x=0.0, y=0.0, z=self.G)  # default: flat

        if self.mode == 'static':
            # Perfectly flat — gravity along z only
            pass

        elif self.mode == 'tilt':
            # Sinusoidal tilt
            phase = (2.0 * math.pi / self.tilt_period) * elapsed

            if self.tilt_axis in ('pitch', 'both'):
                pitch = self.tilt_amp * math.sin(phase)
            if self.tilt_axis in ('roll', 'both'):
                roll = self.tilt_amp * math.cos(phase * 0.7)  # slight phase offset

            # Accelerometer sees gravity rotated into body frame
            linear_accel.x = -self.G * math.sin(pitch)
            linear_accel.y = self.G * math.sin(roll) * math.cos(pitch)
            linear_accel.z = self.G * math.cos(roll) * math.cos(pitch)

            # Angular velocity (derivative of tilt)
            omega = 2.0 * math.pi / self.tilt_period
            if self.tilt_axis in ('pitch', 'both'):
                angular_vel.y = self.tilt_amp * omega * math.cos(phase)
            if self.tilt_axis in ('roll', 'both'):
                angular_vel.x = -self.tilt_amp * omega * 0.7 * math.sin(phase * 0.7)

        elif self.mode == 'spin':
            # Constant rotation around z-axis (yaw), accelerating
            yaw_rate = 0.5 * elapsed  # rad/s, linearly increasing
            angular_vel.z = yaw_rate
            yaw = 0.5 * 0.5 * elapsed ** 2  # integrated angle (for orientation)
            # Centripetal effect (small — mostly for show)
            linear_accel.z = self.G

        elif self.mode == 'random':
            # Random perturbations around gravity
            linear_accel.x = np.random.normal(0.0, self.noise_std * 10)
            linear_accel.y = np.random.normal(0.0, self.noise_std * 10)
            linear_accel.z = self.G + np.random.normal(0.0, self.noise_std * 5)
            angular_vel.x = np.random.normal(0.0, self.gyro_noise_std * 5)
            angular_vel.y = np.random.normal(0.0, self.gyro_noise_std * 5)
            angular_vel.z = np.random.normal(0.0, self.gyro_noise_std * 5)

        elif self.mode == 'drop':
            # Periodic free-fall simulation
            cycle_time = elapsed % self.drop_interval
            if cycle_time < self.drop_duration:
                # Free-fall: zero acceleration
                linear_accel.x = 0.0
                linear_accel.y = 0.0
                linear_accel.z = 0.0
            else:
                # Normal: gravity
                linear_accel.z = self.G

        else:
            self.get_logger().warn(f'Unknown mode: "{self.mode}". Using static.')

        # ------------------------------------------------------------------
        # Add sensor noise
        # ------------------------------------------------------------------
        linear_accel.x += np.random.normal(0.0, self.noise_std)
        linear_accel.y += np.random.normal(0.0, self.noise_std)
        linear_accel.z += np.random.normal(0.0, self.noise_std)
        angular_vel.x += np.random.normal(0.0, self.gyro_noise_std)
        angular_vel.y += np.random.normal(0.0, self.gyro_noise_std)
        angular_vel.z += np.random.normal(0.0, self.gyro_noise_std)

        # ------------------------------------------------------------------
        # Build Imu message
        # ------------------------------------------------------------------
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = self.frame_id

        # Orientation (quaternion from Euler)
        imu_msg.orientation = euler_to_quaternion(roll, pitch, yaw)

        # Covariance matrices (diagonal — -1 in [0] means unknown)
        # Orientation covariance
        imu_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        # Angular velocity
        imu_msg.angular_velocity = angular_vel
        imu_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]

        # Linear acceleration
        imu_msg.linear_acceleration = linear_accel
        imu_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        # Publish
        self.imu_pub.publish(imu_msg)
        self.msg_count += 1

        # Periodic log (every 5 seconds)
        if self.msg_count % int(self.rate * 5) == 0:
            tilt_deg = math.degrees(math.sqrt(roll**2 + pitch**2))
            self.get_logger().info(
                f'[{elapsed:.1f}s] Published {self.msg_count} msgs | '
                f'accel=({linear_accel.x:.3f}, {linear_accel.y:.3f}, {linear_accel.z:.3f}) m/s² | '
                f'tilt={tilt_deg:.1f}°'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MockImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mock IMU Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()