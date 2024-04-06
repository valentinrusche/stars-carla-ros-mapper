#!/usr/bin/env python

"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

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


