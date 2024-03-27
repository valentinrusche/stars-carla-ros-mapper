#!/usr/bin/env python
#
# Copyright (c) 2024 Valentin Rusche
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from typing import Callable
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from carla_msgs.msg import CarlaStatus
from stars_msgs.msg import StarsSimulationStatus


class CarlaGeneralInfoForwarder(Node):

    def __init__(self, node_name: str, callback_group) -> None:
        super().__init__(node_name)

        status_callback: Callable[[CarlaStatus], None] = lambda msg: self.__handle_status(msg = msg)

        self.create_subscription(
            msg_type = CarlaStatus, topic = f"/carla/status",
            callback = status_callback,
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.status_publisher = self.create_publisher(
            msg_type = StarsSimulationStatus, topic = f"/stars/general/simulation_status",
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.get_logger().info(message=f"Successfully created. Starting forwarding of general simulation info.")


    def destroy_node(self) -> None:
        super().destroy_node()

    def __handle_status(self, msg) -> None:
        status = StarsSimulationStatus()
        status.frame = msg.frame
        status.fixed_delta_seconds = msg.fixed_delta_seconds
        status.synchronous_mode = msg.synchronous_mode
        self.status_publisher.publish(status)


