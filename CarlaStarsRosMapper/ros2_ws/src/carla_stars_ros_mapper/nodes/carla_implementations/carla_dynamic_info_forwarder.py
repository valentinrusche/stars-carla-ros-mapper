from typing import List, Callable
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from carla_msgs.msg import CarlaActorList
from stars_msgs.msg import StarsActorList, StarsActorInfo


class CarlaDynamicInfoForwarder():

    def __init__(self, node_name: str, callback_group) -> None:
        super().__init__(node_name=node_name, parameter_overrides=[])
        self.is_polling = True
        self.actor_infos: List[StarsActorList] = []
        self.actor_list: List[CarlaActorList] = []

        callback: Callable[[CarlaActorList], None] = lambda list: self.__handle_actors(actors = list.actors)

        self.create_subscription(
            msg_type = CarlaActorList, topic = f"/carla/all_vehicle_actors",
            callback = callback,
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.all_actor_publisher = self.create_publisher(
            msg_type = StarsActorList, topic = f"/stars/dynamic/all_vehicle_actors",
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)


    def destroy_node(self) -> None:
        super().destroy_node()

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

