#!/usr/bin/env python
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import rclpy
import yaml
# custom packages
from hazard_obj import Hazard
from hazards import ImuTiltHazard, TerrainHazard, GPSElevation

class BlockListenerNode(Node):
    
    """
    ROS node that reads a YAML config and subscribes to each block's topic.
    """

    def __init__(self):
        super().__init__('hazard_grid_map_node')
        # ------------------------------------------------------------------
        # Declare parameters (configurable at launch)
        # ------------------------------------------------------------------
        self.declare_parameter('config_file', '/ws/src/m-explore-ros2/hazardmap/config/params.yaml')       # meters per cell

        # --- Parse YAML ---
        self.config_file = self.get_parameter('config_file').value
        self.get_logger().info(
            f"Loading hazard config from: {self.config_file}")
        
        self.blocks, self.rover_info = self._load_blocks(self.config_file )
        self.get_logger().info(f"Loaded {len(self.blocks)} hazards(s) from config.")

        # --- Store latest messages per block (optional, useful for processing) ---
        self.latest_msgs = {}
        self.latest_gps = None

        # --- Create a subscriber for each hazard ---
        self.subscribers = []
        for block in self.blocks:
            self.get_logger().info(
                f"Subscribing to '{block.topic}' [{block.msg_type_str}] "
                f"(hazard: {block.name})"
            )
            sub = self.create_subscription(
                block.msg_class,
                self.rover_info['topic'] + block.topic,
                self._make_callback(block),
                10,
            )
            self.subscribers.append(sub)
        
        # Subscriber: GPS
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            self.rover_info['topic'] + self.rover_info['gps_topic'],  # Change to your GPS topic
            self.gps_callback,
            10
        )

    # ------------------------------------------------------------------ #
    #  YAML Loading & Validation
    # ------------------------------------------------------------------ #

    def _load_blocks(self, config_path):
        """Read the YAML file and return a list of BlockConfig objects."""
        with open(config_path, "r") as f:
            raw = yaml.safe_load(f)

        if raw is None or "hazards" not in raw or "rover" not in raw:
            raise ValueError(
                f"Config file '{config_path}' must contain a top-level 'rover' or 'hazard' keys."
            )
        
        # Collect the rover name and
        rover_dict = raw["rover"]
        for key in ("name", "topic", "gps_topic"):
                if key not in rover_dict:
                    raise KeyError(
                        f"Rover YAML is missing required key '{key}'."
                    )

        blocks = []
        for i, entry in enumerate(raw["hazards"]):
            # Validate required keys
            for key in ("name", "topic", "msg_type"):
                if key not in entry:
                    raise KeyError(
                        f"Block #{i} is missing required key '{key}'."
                    )
            
            if entry['name'] == 'terrain': blocks.append(TerrainHazard(self, entry))
            elif entry['name'] == 'tilt': blocks.append(ImuTiltHazard(self, entry))
            elif entry['name'] == 'elevation': blocks.append(GPSElevation(self, entry))
            elif entry['name'].startswith('generic'): blocks.append(Hazard(entry))
            else: raise Exception (f"Hazard {entry['name']} not supported.")

        return blocks, rover_dict

    # ------------------------------------------------------------------ #
    #  Callback Factory
    # ------------------------------------------------------------------ #

    def _make_callback(self, block):
        """
        Returns a closure that captures the block's metadata so each
        subscriber callback knows which block it belongs to.
        """
        def callback(msg):
            self.latest_msgs[block.name] = msg
            self._process_message(block, msg)

        return callback
    
    def gps_callback(self, msg):
        self.latest_gps = msg
        self.get_logger().debug(
            f'GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}'
        )

    def _process_message(self, block, msg):
        """
        Central processing hook — customize this to do whatever you need
        with the incoming data + metadata.
        """
        self.get_logger().debug(
            f"[{block.name}] Received msg on '{block.topic}' | "
            f"metadata: {block.metadata}"
        )

        # ----- Example: access metadata fields -----
        sensor_id = block.metadata.get("sensor_id", "unknown")
        self.get_logger().debug(f"  sensor_id = {sensor_id}")

        # ----- processing logic -----
        block._callback(msg)

# ------------------------------------------------------------------ #
#  Run
# ------------------------------------------------------------------ #

def main(args=None):
    rclpy.init(args=args)
    node = BlockListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Hazard Grid Map Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()