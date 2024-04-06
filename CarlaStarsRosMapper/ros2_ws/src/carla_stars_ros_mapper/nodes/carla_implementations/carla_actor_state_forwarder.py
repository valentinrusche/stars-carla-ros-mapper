#!/usr/bin/env python

"""
Copyright (C) 2024 Valentin Rusche

This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
"""


"""
Publishes current state information about the actor vehicle as a service
"""
from typing import Callable, List
from ..async_service_client import AsyncServiceClient

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile

from carla_msgs.msg import CarlaStatus, CarlaActorList

from stars_msgs.msg import StarsActorList, StarsActorInfo, StarsActorState, StarsWaypoint
from stars_msgs.srv import StarsGetActorState


class CarlaActorStateForwarder(AsyncServiceClient):

    def __init__(self, node_name: str, message_type, topic_name: str, callback_group, timeout_sec: float = 10.0):
        """
        Constructor
        """
        super().__init__(
            node_name=node_name,
            message_type=message_type,
            topic_name=topic_name,
            callback_group=callback_group,
            timeout_sec=timeout_sec)
        self.vehicle_actors = set()
        self.actor_infos: List[StarsActorList] = []
        self.actor_list: List[CarlaActorList] = []

        # initialize ros service
        self.get_state_service = self.create_subscription(
            StarsActorState,
            '/stars/dynamic/get_actor_state',
            self.get_actor_state,
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.carla_status = CarlaStatus()

        actor_callback: Callable[[CarlaActorList], None] = lambda list: self.__handle_actors(actors = list.actors)

        self.all_actor_subscriber = self.create_subscription(
            msg_type = CarlaActorList, topic = f"/carla/all_vehicle_actors",
            callback = actor_callback,
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.all_actor_publisher = self.create_publisher(
            msg_type = StarsActorList, topic = f"/stars/dynamic/all_vehicle_actors",
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.get_state_service = self.create_service(
            StarsGetActorState,
            '/stars/dynamic/get_actor_state',
            self.get_actor_state)

        self.get_logger().info(message=f"Successfully created. Starting forwarding of actor state data.")

    def destroy(self) -> None:
        """
        Destructor
        """
        self.ego_vehicle = None

    def get_actor_state(self, req, response=None):
        """
        Convenience method to get the waypoint for an actor
        """
        forwarded_request = self.message_type.Request()
        forwarded_request.id = req.id
        response: Future = self.client.call_async(request=msg)
        rclpy.spin_until_future_complete(node=self, future=response)
        result = response.result()
        state = result.actor_state if result is not None else None

        forwarded_response = StarsActorState()
        if state:
            actor_info = StarsActorInfo()
            actor_info.id = response.vehicle_info.id
            actor_info.type = response.vehicle_info.type
            actor_info.rolename = response.vehicle_info.rolename
            actor_info.parent_id = 0

            acceleration = response.vehicle_status.acceleration
            orientation = response.vehicle_status.orientation
            velocity = response.vehicle_status.velocity

            waypoint = StarsWaypoint()
            waypoint.road_id = response.current_waypoint.road_id
            waypoint.section_id = response.current_waypoint.section_id
            waypoint.lane_id = response.current_waypoint.lane_id
            waypoint.is_junction = response.current_waypoint.is_junction
            waypoint.pose = response.current_waypoint.pose

            forwarded_response.actor_info = actor_info
            forwarded_response.acceleration = acceleration
            forwarded_response.orientation = orientation
            forwarded_response.velocity = velocity
            forwarded_response.is_ego_vehicle = response.is_ego_vehicle
            forwarded_response.current_waypoint = waypoint
            forwarded_response.geo_latitude = response.geo_latitude
            forwarded_response.geo_longitude = response.geo_longitude
            forwarded_response.geo_altitude = response.geo_altitude
            forwarded_response.current_tick = response.current_tick

            forwarded_response.header = response.header

        return forwarded_response

    def __handle_actors(self, actors) -> None:
        for actor in actors:
            msg = StarsActorInfo()
            msg.id = actor.id
            msg.type = actor.type
            msg.pose = actor.pose
            msg.rolename = actor.rolename
            self.actor_infos.append(msg)

        actor_list_msg = StarsActorList()
        actor_list_msg.actors = self.actor_infos

        self.all_actor_publisher.publish(actor_list_msg)
