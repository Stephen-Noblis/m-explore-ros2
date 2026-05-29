#!/usr/bin/env python3
"""
Hazard Map Grid Map Node
========================
A ROS2 node that:
  1. Initializes a grid_map (using grid_map_msgs)
  2. Subscribes to /HazardMessage for hazard data (e.g., IMU tilt readings)
  3. Updates the grid map cells based on received hazard location/rules
  4. Publishes the updated grid map on /hazard_grid_map

Dependencies:
  - grid_map_msgs (install via: sudo apt install ros-humble-grid-map-msgs)
  - hazardmap_msgs (your custom message package)
  - numpy
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Header
from geometry_msgs.msg import Pose, Point, Quaternion
from grid_map_msgs.msg import GridMap, GridMapInfo
from nav_msgs.msg import OccupancyGrid

# Import your custom message
from hazardmap_msgs.msg import HazardMap


class HazardGridMapNode(Node):
    """
    ROS2 Node that manages a grid map for hazard visualization.

    Subscribes to /HazardMessage and updates a grid_map layer with
    hazard severity values derived from IMU tilt readings.
    """

    def __init__(self):
        super().__init__('hazard_grid_map_node')

        # ------------------------------------------------------------------
        # Declare parameters (configurable at launch)
        # ------------------------------------------------------------------
        self.declare_parameter('map_resolution', 0.1)       # meters per cell
        self.declare_parameter('map_length_x', 10.0)        # map extent in x (meters)
        self.declare_parameter('map_length_y', 10.0)        # map extent in y (meters)
        self.declare_parameter('map_frame', 'map')          # TF frame id
        self.declare_parameter('publish_rate', 1.0)         # Hz
        self.declare_parameter('costmap_topic', '/j100_0000/local_costmap/costmap')         # Hz

        self.declare_parameter('map_center_lat', 0.0)   # reference latitude (degrees)
        self.declare_parameter('map_center_lon', 0.0)   # reference longitude (degrees)
        self.declare_parameter('auto_center', True)      # auto-set center from first message

        # Read parameters
        self.ref_lat = self.get_parameter('map_center_lat').value
        self.ref_lon = self.get_parameter('map_center_lon').value
        self.auto_center = self.get_parameter('auto_center').value
        self.map_centered = not self.auto_center  # if manual center, mark as ready
        self.resolution = self.get_parameter('map_resolution').value
        self.length_x = self.get_parameter('map_length_x').value
        self.length_y = self.get_parameter('map_length_y').value
        self.frame_id = self.get_parameter('map_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.costmap_topic = self.get_parameter('costmap_topic').value

        # ------------------------------------------------------------------
        # Compute grid dimensions
        # ------------------------------------------------------------------
        self.cols = int(self.length_x / self.resolution)  # number of cells in x
        self.rows = int(self.length_y / self.resolution)  # number of cells in y

        self.get_logger().info(
            f'Initializing hazard grid map: {self.cols}x{self.rows} cells, '
            f'resolution={self.resolution} m, '
            f'size=({self.length_x} x {self.length_y}) m'
        )

        # ------------------------------------------------------------------
        # Initialize grid map data layers (stored as numpy arrays)
        # ------------------------------------------------------------------
        # Layer: "hazard" — normalized severity [0.0, 1.0]
        self.layers = {}

        # Map center position in world frame
        self.map_position_x = 0.0
        self.map_position_y = 0.0

        # ------------------------------------------------------------------
        # ROS2 Pub/Sub setup
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Publisher: grid_map_msgs/GridMap
        self.grid_map_pub = self.create_publisher(
            GridMap, '/hazard_grid_map', qos
        )

        # Subscriber: custom HazardMessage
        self.hazard_sub = self.create_subscription(
            HazardMap,
            '/HazardMessage',
            self.hazard_callback,
            10
        )

        # Subscribe to Nav2 local costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            10
        )

        # Timer for periodic publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_grid_map)

        self.get_logger().info('Hazard Grid Map Node is ready.')

    # ======================================================================
    # CALLBACK: Process incoming hazard messages
    # ======================================================================
    def hazard_callback(self, msg: HazardMap):
        """
        Process a HazardMessage and update the grid map.

        Expected HazardMessage fields:
            |- pose (geometry_msgs): 
            |  |--position (geometry_msgs/Point):
            |  |--orientation (geometry_msgs/Quaternion):
            
            |- rule       (string):  fill rule — "max", "replace", or "average"
        """
        # Collect data from HazardMap msg 
        haz_data = float(msg.data[0])
        haz_position = msg.gps_position
        haz_names = msg.layer_names
        radius = 1

        # Auto-center on first message if enabled
        if self.auto_center and not self.map_centered:
            self.ref_lat = haz_position.latitude
            self.ref_lon = haz_position.longitude
            self.map_centered = True
            self.get_logger().info(
                f'Map centered on GPS: ({self.ref_lat:.6f}, {self.ref_lon:.6f})'
            )

        # Convert GPS to local meters
        local_x, local_y = self._gps_to_local(
            haz_position.latitude, haz_position.longitude
        )

        self.get_logger().debug(
            f'Hazard received: GPS=({haz_position.latitude:.6f}, {haz_position.longitude:.6f}), '
            f'local=({local_x:.2f}, {local_y:.2f}) m, data={haz_data:.2f}'
        )

        # Now use local meter coordinates (unchanged from here)
        affected_indices = self._get_cells_in_radius(local_x, local_y, radius)

        for (row, col) in affected_indices:
            self._apply_rule(row, col, haz_names, haz_data, 'replace')

    # ======================================================================
    # CALLBACK: Process incoming costmaps 
    # ======================================================================

    def costmap_callback(self, msg: OccupancyGrid):
        """Convert OccupancyGrid to a grid_map layer."""

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Convert data: OccupancyGrid is row-major, values 0-100 or -1
        raw = np.array(msg.data, dtype=np.float32).reshape((height, width))

        # Normalize: map [0, 100] → [0.0, 1.0], treat -1 (unknown) as 0.5
        normalized = np.where(raw < 0, 0.5, raw / 100.0)

        # Flip to match grid_map convention (column-major, top-left origin)
        layer_data = np.flipud(normalized)
        self.layers["costmap"] = layer_data

    # ======================================================================
    # GRID MAP UTILITIES
    # ======================================================================
    def _world_to_index(self, world_x: float, world_y: float):
        """
        Convert world coordinates (meters) to grid indices (row, col).

        The grid map is centered at (map_position_x, map_position_y).
        Index (0, 0) is at the top-left corner of the map.
        """
        # Offset from map origin (top-left corner)
        origin_x = self.map_position_x - self.length_x / 2.0
        origin_y = self.map_position_y - self.length_y / 2.0

        col = int((world_x - origin_x) / self.resolution)
        row = int((world_y - origin_y) / self.resolution)

        return row, col

    def _is_valid_index(self, row: int, col: int) -> bool:
        """Check if index is within grid bounds."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def _get_cells_in_radius(self, x: float, y: float, radius: float):
        """
        Return list of (row, col) indices within `radius` meters of (x, y).
        """
        center_row, center_col = self._world_to_index(x, y)
        radius_cells = int(math.ceil(radius / self.resolution))

        indices = []
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                r, c = center_row + dr, center_col + dc
                if not self._is_valid_index(r, c):
                    continue
                # Check actual Euclidean distance
                dist = math.sqrt((dr * self.resolution) ** 2 + (dc * self.resolution) ** 2)
                if dist <= radius:
                    indices.append((r, c))

        return indices

    def _apply_rule(self, row: int, col: int, haz_names: float, tilt_mag: float, rule: str):
        """
        Apply the fill rule to a single grid cell.

        Supported rules:
            - "replace" : overwrite cell value
            - "max"     : keep the maximum severity seen
            - "average" : running average (simple blend with existing)
        """
        for haz in haz_names: 
            if haz not in self.layers: self.layers[haz] = np.full((self.rows, self.cols), 0, dtype=np.float32)
            if rule == 'replace':
                self.layers[haz][row, col] = tilt_mag

            # elif rule == 'max':
            #     current = self.layer_hazard[row, col]
            #     if np.isnan(current) or severity > current:
            #         self.layer_hazard[row, col] = severity
            #         self.layer_tilt[row, col] = tilt_mag

            # elif rule == 'average':
            #     current = self.layer_hazard[row, col]
            #     if np.isnan(current):
            #         self.layer_hazard[row, col] = severity
            #         self.layer_tilt[row, col] = tilt_mag
            #     else:
            #         # Simple exponential moving average (alpha = 0.5)
            #         alpha = 0.5
            #         self.layer_hazard[row, col] = alpha * severity + (1 - alpha) * current
            #         self.layer_tilt[row, col] = alpha * tilt_mag + (1 - alpha) * self.layer_tilt[row, col]

            # else:
            #     self.get_logger().warn(f'Unknown fill rule: "{rule}". Defaulting to replace.')
            #     self.layer_hazard[row, col] = severity
            #     self.layer_tilt[row, col] = tilt_mag

    # ======================================================================
    # PUBLISHER: Convert numpy data to grid_map_msgs/GridMap and publish
    # ======================================================================
    def publish_grid_map(self):
        """
        Convert internal numpy layers to a grid_map_msgs/GridMap message
        and publish it.
        """
        # if no layers yet, continue
        if len(self.layers) == 0: return 

        now = self.get_clock().now().to_msg()

        grid_map_msg = GridMap()

        # --- Header ---
        grid_map_msg.header = Header()
        grid_map_msg.header.stamp = now
        grid_map_msg.header.frame_id = self.frame_id

        # --- Grid Map Info ---
        grid_map_msg.info = GridMapInfo()
        grid_map_msg.info.resolution = self.resolution
        grid_map_msg.info.length_x = self.length_x
        grid_map_msg.info.length_y = self.length_y
        grid_map_msg.info.pose = Pose(
            position=Point(x=self.map_position_x, y=self.map_position_y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        # --- Layers ---
        grid_map_msg.layers = (self.layers.keys())
        grid_map_msg.basic_layers = (self.layers.keys())

        # Convert each numpy layer to Float32MultiArray
        grid_map_msg.data = [
            self._numpy_to_multiarray(self.layers[haz]) for haz in self.layers
        ]

        # Circular buffer indices (no offset since we don't use circular buffer)
        grid_map_msg.outer_start_index = 0
        grid_map_msg.inner_start_index = 0

        # Publish
        self.grid_map_pub.publish(grid_map_msg)

    def _numpy_to_multiarray(self, data: np.ndarray) -> Float32MultiArray:
        """
        Convert a 2D numpy array to std_msgs/Float32MultiArray.

        grid_map stores data in column-major order (Eigen default).
        The MultiArray is structured as (cols x rows) with column-major layout.
        """
        multiarray = Float32MultiArray()

        # grid_map convention: data is stored column-major (Fortran order)
        multiarray.layout = MultiArrayLayout()
        multiarray.layout.dim = [
            MultiArrayDimension(label='column_index', size=self.cols, stride=self.cols * self.rows),
            MultiArrayDimension(label='row_index', size=self.rows, stride=self.rows),
        ]
        multiarray.layout.data_offset = 0

        # Flatten in column-major (Fortran) order to match Eigen's default storage
        multiarray.data = data.flatten(order='F').tolist()

        return multiarray

    # ======================================================================
    # UTILITY: Reset / clear the map
    # ======================================================================
    def clear_map(self):
        """Reset all layers to NaN (unknown)."""
        self.layer_hazard[:] = np.nan
        self.layer_tilt[:] = np.nan
        self.get_logger().info('Grid map cleared.')

    def _gps_to_local(self, lat: float, lon: float) -> tuple:
        """
        Convert GPS (lat, lon) in degrees to local (x, y) in meters
        relative to the reference point (map center).
        
        Uses equirectangular projection — valid for small areas.
        """
        METERS_PER_DEG_LAT = 111320.0  # ~constant
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.ref_lat))

        local_x = (lon - self.ref_lon) * meters_per_deg_lon
        local_y = (lat - self.ref_lat) * METERS_PER_DEG_LAT

        return local_x, local_y


def main(args=None):
    rclpy.init(args=args)
    node = HazardGridMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Hazard Grid Map Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()