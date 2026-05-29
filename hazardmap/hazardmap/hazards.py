
from hazard_obj import Hazard
#from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import numpy as np
import cv2

class ImuTiltHazard(Hazard):
    """Concrete hazard that converts IMU tilt readings into hazard scores."""

    def __init__(self, hazard_dict, node, tilt_threshold=0.3):
        super().__init__(hazard_dict, node)
        self.tilt_threshold = tilt_threshold

    def process_data(self, msg):
        """
        Process an IMU message and return hazard data if tilt exceeds threshold.

        Args:
            msg: sensor_msgs/Imu message

        Returns:
            Tilt magnitude float, or None if below threshold.
        """
        # Example: compute tilt magnitude from linear acceleration
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.atan2(sinr_cosp, cosr_cosp)

        # Pitch (clamped to avoid gimbal lock)
        sinp = 2.0 * (w * y - z * x)
        pitch = np.copysign(np.pi / 2, sinp) if abs(sinp) >= 1 else np.asin(sinp)
        tilt_magntitude = np.sqrt(roll**2 + pitch**2)

        self.monitor_hazard(tilt_magntitude)
        return tilt_magntitude

    def monitor_hazard(self, data):
        if data >= self.tilt_threshold:
            self._node.get_logger().debug(f"[{self.name}] Tilt detected: {data:.3f}")


class TerrainHazard(Hazard):
    """Concrete hazard that samples color from camera."""

    def __init__(self, hazard_dict, node):
        super().__init__(hazard_dict, node)

    def imgmsg_to_cv2(self, msg):
        dtype = np.uint8
        n_channels = 3 if 'bgr8' in msg.encoding or 'rgb8' in msg.encoding else 1
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, n_channels)
        
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    
    def bgr_to_grid_map_float(self, b: int, g: int, r: int) -> float:
        """
        Encode a BGR pixel (0-255 per channel) into a single float32
        for use in a grid_map color layer.
        
        The encoding packs RGB into a 24-bit integer (0x00RRGGBB)
        and reinterprets the bits as a float32.
        """
        # Pack as 0x00RRGGBB (grid_map convention)
        color_int = np.uint32((int(r) << 16) | (int(g) << 8) | int(b))
        # Reinterpret bits as float32 (no conversion, just raw bit cast)
        color_float = color_int.view(np.float32)
        return float(color_float)

    def process_data(self, msg):
        """
        Process an Image message and return average terrain data.

        Args:
            msg: sensor_msgs/Image message

        Returns:
            Avg. terrain color
        """
        # Convert ROS Image message to OpenCV format
        np_image = self.imgmsg_to_cv2(msg)
        # Compute the average color value per channel (BGR)
        avg_channels = np.mean(np_image, axis=(0, 1))
        avg_color = self.bgr_to_grid_map_float(avg_channels[0], avg_channels[1], avg_channels[2])
        return avg_color

    def monitor_hazard(self, data):
        self._node.get_logger().info(f"[{self.name}] Terrain detected: {data:.3f}")

class GPSElevation(Hazard):
    """Concrete hazard that converts IMU tilt readings into hazard scores."""

    def __init__(self, hazard_dict, node, tilt_threshold=0.3):
        super().__init__(hazard_dict, node)
        self.tilt_threshold = tilt_threshold

    def process_data(self, msg):
        """
        Process an IMU message and return hazard data if tilt exceeds threshold.

        Args:
            msg: sensor_msgs/Imu message

        Returns:
            Tilt magnitude float, or None if below threshold.
        """
        # Example: compute tilt magnitude from linear acceleration
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        magnitude = (ax**2 + ay**2) ** 0.5

        if magnitude < self.tilt_threshold:
            return None  # skip publishing

        self.monitor_hazard(magnitude)
        return magnitude

    def monitor_hazard(self, data):
        self._node.get_logger().info(f"[{self.name}] Tilt detected: {data:.3f}")