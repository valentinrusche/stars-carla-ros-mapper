import rclpy
from typing import Union
from carla_actor_state_types.srv import GetActorState
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from .util.helpers import bool_val_of_env_key
from .nodes.carla_implementations.carla_dynamic_info_forwarder import CarlaDynamicInfoForwarder
from .nodes.carla_implementations.carla_stars_static_map_forwarder import CarlaStaticMapReader


def main() -> None:
    """Runs this projects primary nodes in ROS2"""
    rclpy.init()

    # we declare all objects here to use them in the except
    node_executor: Union[MultiThreadedExecutor, None] = None
    carla_dynamic_data_forwarder: Union[CarlaDynamicInfoForwarder, None] = None
    carla_static_map_reader: Union[CarlaStaticMapReader, None] = None

    callback_group = ReentrantCallbackGroup() # allows for the concurrent execution of nodes

    try:
        node_executor: Union[MultiThreadedExecutor,None] = MultiThreadedExecutor(num_threads=5) # needed to allow multiple nodes to be run inside ROS2

        if bool_val_of_env_key(env_key="ENABLED_FLAG_STATIC_DATA_LOGGING", default="True"):
            carla_static_map_reader: Union[CarlaStaticMapReader, None] = CarlaStaticMapReader(node_name = 'Carla_Static_Map_Reader', polling_rate = 10,
                                                                                callback_group = callback_group)
            node_executor.add_node(node=carla_static_map_reader)

        if bool_val_of_env_key(env_key="ENABLED_FLAG_DYNAMIC_DATA_LOGGING", default="True"):
            carla_dynamic_data_forwarder: Union[CarlaDynamicInfoForwarder,None] = CarlaDynamicInfoForwarder(node_name = 'Carla_Dynamic_Data_Client', message_type = GetActorState,
                                                                    topic_name = '/carla_actor_state_publisher/get_actor_state',
                                                                    callback_group = callback_group)
            node_executor.add_node(node=carla_dynamic_data_forwarder)

        node_executor.spin() # Run all added node callbacks until the program terminates
    except (SystemExit, KeyboardInterrupt):
        if bool_val_of_env_key(env_key="ENABLED_FLAG_DYNAMIC_DATA_LOGGING", default="True") and carla_dynamic_data_client is not None:
            carla_dynamic_data_client.destroy_node()
    finally:
        rclpy.shutdown() # shutdown ROS2
        
        if node_executor is not None:
            node_executor.shutdown()

if __name__ == "__main__":
    main()
