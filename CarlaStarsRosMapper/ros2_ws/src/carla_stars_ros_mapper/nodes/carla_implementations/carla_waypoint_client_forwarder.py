#!/usr/bin/env python

"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""

from typing import List, Union

from carla_waypoint_types.msg import CarlaWaypoint
from stars_msgs.msg import StarsWaypoint
from stars_msgs.srv import StarsGetAllWaypoints
from ..async_service_client import AsyncServiceClient


class CarlaWaypointClientForwarder(AsyncServiceClient):

    def __init__(self, node_name: str, message_type, topic_name: str, callback_group, timeout_sec: float = 10.0) -> None:
        """Requests all generated waypoints by the Carla-ROS-Bridge waypoint publisher service asynchronously
            and publishes them to the system when they are available"""
        super().__init__(node_name=node_name, message_type=message_type, topic_name=topic_name, callback_group=callback_group, timeout_sec=timeout_sec)

        self.get_all_waypoints_service = self.create_service(
            StarsGetAllWaypoints,
            '/stars/static/waypoints/get_all_waypoints',
            self.get_map_waypoints)

        self.get_logger().info(message=f"Successfully created. Starting forwarding of waypoints.")

    def get_map_waypoints(self, req=None, response=None):
        """
        Get all waypoints for the current map
        """
        self.result: Union[CarlaWaypoint, None] = self.send_request()

        # waypoints array inside the waypoints result from the service
        self.carla_waypoints = self.result.waypoints.waypoints if self.result is not None else None
        self.stars_waypoints: List[StarsWaypoint] = []
        if self.carla_waypoints is not None:
            self.get_logger().info(message=f"Received {len(self.waypoints)} waypoints for the current map.")

            for waypoint in self.carla_waypoints:
                stars_waypoint = StarsWaypoint()
                stars_waypoint.road_id = waypoint.road_id
                stars_waypoint.section_id = waypoint.section_id
                stars_waypoint.lane_id = waypoint.lane_id
                stars_waypoint.is_junction = waypoint.is_junction
                stars_waypoint.pose = waypoint.pose

                self.stars_waypoints.append(stars_waypoint)
        response = StarsGetAllWaypoints()
        response.waypoints.waypoints = self.stars_waypoints
        return response
