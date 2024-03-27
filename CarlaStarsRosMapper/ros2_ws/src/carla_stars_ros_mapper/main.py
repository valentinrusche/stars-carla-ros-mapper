#!/usr/bin/env python
#
# Copyright (c) 2024 Valentin Rusche
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from re import T
from .nodes.carla_implementations.carla_actor_state_forwarder import CarlaActorStateForwarder
import rclpy
from typing import Union
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from .nodes.carla_implementations.carla_general_info_forwarder import CarlaGeneralInfoForwarder
from .nodes.carla_implementations.carla_stars_static_map_data_forwarder import CarlaStaticMapDataForwarder
from carla_actor_state_types.srv import GetActorState


def main() -> None:
    """Runs this projects primary nodes in ROS2"""
    rclpy.init()

    # we declare all objects here to use them in the except
    node_executor: Union[MultiThreadedExecutor, None] = None
    carla_general_data_forwarder: Union[CarlaGeneralInfoForwarder, None] = None
    carla_static_map_forwarder: Union[CarlaStaticMapDataForwarder, None] = None
    carla_actor_state_forwarder: Union[CarlaActorStateForwarder, None] = None

    callback_group: ReentrantCallbackGroup = ReentrantCallbackGroup() # allows for the concurrent execution of nodes

    try:
        node_executor: Union[MultiThreadedExecutor, None] = MultiThreadedExecutor(num_threads=5) # needed to allow multiple nodes to be run inside ROS2

        carla_static_map_forwarder: Union[CarlaStaticMapDataForwarder, None] = CarlaStaticMapDataForwarder(node_name = 'Carla_Static_Map_Data_Forwarder', polling_rate = 10,
                                                                            callback_group = callback_group)
        node_executor.add_node(node=carla_static_map_forwarder)

        carla_general_data_forwarder: Union[CarlaGeneralInfoForwarder, None] = CarlaGeneralInfoForwarder(node_name = 'Carla_General_Data_Forwarder',
                                                                                                         callback_group = callback_group)
        node_executor.add_node(node=carla_general_data_forwarder)

        carla_actor_state_forwarder: Union[CarlaActorStateForwarder, None] = CarlaActorStateForwarder(node_name = 'Carla_Actor_State_Forwarder',
                                                                    message_type=GetActorState, topic_name='/carla_actor_state_publisher/get_actor_state',
                                                                    callback_group = callback_group,
                                                                    timeout_sec = 10.0)
        node_executor.add_node(node=carla_actor_state_forwarder)

        node_executor.spin() # Run all added node callbacks until the program terminates
    finally:
        rclpy.shutdown() # shutdown ROS2

        if node_executor is not None:
            node_executor.shutdown()

if __name__ == "__main__":
    main()
