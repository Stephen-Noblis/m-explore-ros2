#!/usr/bin/env python
"""
ROS node that dynamically subscribes to topics defined in a YAML config file.
Each 'block' in the YAML specifies a topic, message type, and metadata.
"""

import importlib
from abc import ABC, abstractmethod
# custom imports
from hazardmap_msgs.msg import HazardMap


class Hazard:
    """Parsed representation of a single block from the YAML config."""

    def __init__(self, node, hazard_dict):
        self._node = node

        self.name = hazard_dict["name"]
        self.topic = hazard_dict["topic"]
        self.msg_type_str = hazard_dict["msg_type"]  # e.g. "sensor_msgs/Image"
        self.metadata = hazard_dict.get("metadata", {})
        self.msg_class = self._resolve_msg_class(self.msg_type_str)

        # Publisher (publishes the processed HazardMap)
        self._publisher = self._node.create_publisher(HazardMap, '/HazardMessage', 10)

        # Subscriber (subscribes to the input topic)
        self._subscription = self._node.create_subscription(
            self.msg_class,
            self.topic,
            self._callback,
            10,
        )

    # ------------------------------------------------------------------
    # Pipeline: subscribe (_callback) -> process (process_data) -> publish
    # ------------------------------------------------------------------

    def _callback(self, msg):
        """Internal subscriber callback. Runs the process -> publish pipeline."""
        if self._node.latest_gps is None: return

        processed = self.process_data(msg)
        if processed is not None:
            hazard_msg = self.to_hazardmsg(processed, msg)
            self._publisher.publish(hazard_msg)
            
    
    @abstractmethod
    def process_data(self, msg):
        """
        Override this method in subclasses to define custom processing logic.

        Args:
            msg: The incoming ROS message (type matches self.msg_class).

        Returns:
            Processed data to be wrapped into a HazardMap message,
            or None to skip publishing this cycle.
        """
        return msg.data


    @staticmethod
    def _resolve_msg_class(msg_type_str):
        """
        Dynamically import a ROS message class from a 'package/MessageType' string.
        e.g. 'sensor_msgs/Image' -> sensor_msgs.msg.Image
        """
        if "/" not in msg_type_str:
            raise ValueError(
                f"msg_type must be in 'package/MessageType' format, got: {msg_type_str}"
            )
        package, msg_name = msg_type_str.split("/", 1)
        module = importlib.import_module(f"{package}.msg")
        try:
            return getattr(module, msg_name)
        except AttributeError:
            raise ImportError(
                f"Message '{msg_name}' not found in '{package}.msg'. "
                f"Is the message defined and the package built?"
            )

    def __repr__(self):
        return (
            f"BlockConfig(name={self.name!r}, topic={self.topic!r}, "
            f"msg_type={self.msg_type_str!r}, metadata={self.metadata})"
        )
    
    def to_hazardmsg(self, data, msg):
        new_msg = HazardMap()
        new_msg.layer_names = [self.name]
        new_msg.gps_position = self._node.latest_gps
        new_msg.data = [data]
        return new_msg

