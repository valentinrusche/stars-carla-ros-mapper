import rclpy
from typing import Union
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from .nodes.carla_implementations.carla_dynamic_info_forwarder import CarlaDynamicInfoForwarder
from .nodes.carla_implementations.carla_stars_static_map_data_forwarder import CarlaStaticMapDataForwarder


def main() -> None:
    """Runs this projects primary nodes in ROS2"""
    rclpy.init()

    # we declare all objects here to use them in the except
    node_executor: Union[MultiThreadedExecutor, None] = None
    carla_dynamic_data_forwarder: Union[CarlaDynamicInfoForwarder, None] = None
    carla_static_map_reader: Union[CarlaStaticMapDataForwarder, None] = None

    callback_group: ReentrantCallbackGroup = ReentrantCallbackGroup() # allows for the concurrent execution of nodes

    try:
        node_executor: Union[MultiThreadedExecutor,None] = MultiThreadedExecutor(num_threads=5) # needed to allow multiple nodes to be run inside ROS2

        carla_static_map_reader: Union[CarlaStaticMapDataForwarder, None] = CarlaStaticMapDataForwarder(node_name = 'Carla_Static_Map_Data_Forwarder', polling_rate = 10,
                                                                            callback_group = callback_group)
        node_executor.add_node(node=carla_static_map_reader)

        carla_dynamic_data_forwarder: Union[CarlaDynamicInfoForwarder, None] = CarlaDynamicInfoForwarder(node_name = 'Carla_Dynamic_Data_Forwarder',
                                                                callback_group = callback_group)
        node_executor.add_node(node=carla_dynamic_data_forwarder)

        node_executor.spin() # Run all added node callbacks until the program terminates
    finally:
        rclpy.shutdown() # shutdown ROS2

        if node_executor is not None:
            node_executor.shutdown()

if __name__ == "__main__":
    main()
