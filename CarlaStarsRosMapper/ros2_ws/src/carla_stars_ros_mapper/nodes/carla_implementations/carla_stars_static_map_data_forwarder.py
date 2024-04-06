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
from carla_msgs.msg import CarlaWorldInfo
from stars_msgs.msg import StarsWorldInfo
from carla_waypoint_types.srv import GetAllWaypoints
from .carla_waypoint_client_forwarder import CarlaWaypointClientForwarder


class CarlaStaticMapDataForwarder(Node):

    def __init__(self, node_name: str, polling_rate: int, callback_group) -> None:
        """Creates a ROS2 topic subscription listening for the current map data and calling _write_static_data_to_file
            to write it to disk"""
        super().__init__(node_name)

        callback: Callable[[CarlaWorldInfo], None] = lambda world_info: self.__republish_world_info(world_info=world_info)
        self.create_subscription(
            msg_type=CarlaWorldInfo, topic="/carla/world_info",
            callback=callback,
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.publisher = self.create_publisher(
            msg_type=StarsWorldInfo, topic="/stars/static/world_info",
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.waypoint_client: CarlaWaypointClientForwarder = CarlaWaypointClientForwarder(node_name = 'Carla_Waypoint_Client_Forwarder', message_type = GetAllWaypoints,
                                                                    topic_name = '/carla_waypoint_publisher/ego_vehicle/get_all_waypoints',
                                                                    callback_group = callback_group, timeout_sec = polling_rate)


        self.get_logger().info(message=f"Successfully created. Starting forwarding of static map data.")

    def __republish_world_info(self, world_info: CarlaWorldInfo) -> None:
        self.get_logger().info(message="Received newest world info.")
        self.world_info: CarlaWorldInfo = world_info
        stars_world_info = StarsWorldInfo()
        stars_world_info.map_name = world_info.map_name
        stars_world_info.map_data = world_info.opendrive
        stars_world_info.map_data = "xodr"
        self.publisher.publish(stars_world_info)

