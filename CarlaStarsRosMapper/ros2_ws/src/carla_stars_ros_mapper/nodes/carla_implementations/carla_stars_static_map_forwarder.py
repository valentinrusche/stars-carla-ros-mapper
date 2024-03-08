import os
from typing import Callable
from pathlib import Path
from threading import Thread
import time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile
from rclpy.impl.rcutils_logger import RcutilsLogger
from carla_msgs.msg import CarlaWorldInfo
from carla_waypoint_types.srv import GetAllWaypoints
from crdesigner.map_conversion.map_conversion_interface import commonroad_to_lanelet
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_commonroad
from commonroad.scenario.scenario import Tag, Scenario
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from ...stars import dataclass_to_json_converter
from .carla_waypoint_client import CarlaWaypointClient


class CarlaStaticMapReader(Node):

    def __init__(self, node_name: str, polling_rate: int, callback_group) -> None:
        """Creates a ROS2 topic subscription listening for the current map data and calling _write_static_data_to_file
            to write it to disk"""
        super().__init__(node_name=node_name, parameter_overrides=[])
        self.polling_rate: int = polling_rate

        self.is_exporting_done = False
        self.received_world_info = False

        callback: Callable[[CarlaWorldInfo], None] = lambda world_info: self.__save_world_info(world_info=world_info)
        self.create_subscription(
            msg_type=CarlaWorldInfo, topic="/carla/world_info",
            callback=callback,
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group = callback_group)

        self.waypoint_client: CarlaWaypointClient = CarlaWaypointClient(node_name = 'Carla_Waypoint_Client', message_type = GetAllWaypoints,
                                                                    topic_name = '/carla_waypoint_publisher/ego_vehicle/get_all_waypoints',
                                                                    callback_group = callback_group, timeout_sec = 10.0)

        self.thread = Thread(target=self.__update_thread)

        self.thread.start()

        self.get_logger().info(message=f"Successfully created. Starting processing of static map data.")

    def __save_world_info(self, world_info: CarlaWorldInfo) -> None:
        self.get_logger().info(message="Received newest world info.")
        self.world_info: CarlaWorldInfo = world_info
        self.received_world_info = True

    def __update_thread(self) -> None:
        """
        execution loop for async mode actor discovery
        """
        while not self.is_exporting_done:
            if self.received_world_info and self.waypoint_client.waypoints is not None:
                self.__write_static_data_to_file(map_name=Path(self.world_info.map_name), map_data=self.world_info.opendrive, logger = self.get_logger())
            time.sleep(self.polling_rate)

        self.get_logger().info(message="Done exporting static map data.")
        # Work was finished so we can stop the node from spinning infinitely
        self.destroy_node()

    def __write_static_data_to_file(self, map_name: Path, map_data: str, logger: RcutilsLogger) -> None:
        """Creates a map file containing the read opendrive data xml string creating the desired path if it
        not yet exists. xml_dir is the name of the OpenDrive map dir under the CARLA_MAP_FILE_DIR path. 
        json_dir ist the name of the STARS compatible JSON file under the CARLA_STARS_STATIC_FILE_DIR path.
        """

        stars_json_dir: Path = Path(os.getenv(key="CARLA_STARS_STATIC_FILE_DIR")) / map_name.parent # type: ignore
        stars_json_file: Path = stars_json_dir / f"{str(map_name.name)}.json"

        self.__save_xodr_data(map_name=map_name, data=map_data, logger = logger)

        self.__save_as_stars_json(map_name=map_name, data=map_data, json_dir=stars_json_dir, json_file=stars_json_file)
        self.is_exporting_done = True

    def __save_as_stars_json(self, map_name: Path, data: str, json_dir: Path, json_file: Path) -> None: 
        dataclass_to_json_converter.export_to_json(map_name=map_name, data=data, json_dir=json_dir, json_file=json_file, logger = self.get_logger(), waypoints=self.waypoint_client.waypoints)

    def __save_xodr_data(self, map_name: Path, data: str, logger: RcutilsLogger) -> None:

        xodr_dir: Path = Path(os.getenv(key="CARLA_MAP_FILE_DIR")) / map_name.parent # type: ignore
        xodr_file: Path = xodr_dir / f"{str(map_name.name)}.xodr"

        if not xodr_dir.exists():
            self.get_logger().info(message=f"Dir {str(xodr_dir)} does not exist yet. Creating it.")
            xodr_dir.mkdir(parents=True, exist_ok=True)

        if not xodr_file.exists():
            self.get_logger().info(message=f"File {str(xodr_file)} does not exist yet. Creating it.")
            xodr_file.touch()
            with open(file=xodr_file, mode="w") as f:
                f.write(data)
            self.get_logger().info(message=f"Successfully saved map {str(map_name.name)} to {str(xodr_file)} in OpenDrive format.")

        self.__convert_opendrive_to_lanelet2(map_name = map_name, logger = logger)


    def __convert_opendrive_to_lanelet2(self, map_name: Path, logger: RcutilsLogger) -> None:
        """Converts opendrive map data to lanelet2 via commonroad"""
        self.__convert_opendrive_to_commonroad(map_name = map_name, logger = logger)
        self.__convert_commonroad_to_lanelet2(map_name = map_name, logger = logger)

    def __convert_opendrive_to_commonroad(self, map_name: Path, logger: RcutilsLogger) -> None:
        """See https://commonroad-scenario-designer.readthedocs.io/en/latest/details/open_drive/#implementation-details
        Converts the map from the opendrive to the linked commonroad type"""
        xodr_file_dir: Path = Path(os.getenv(key="CARLA_MAP_FILE_DIR")) / map_name.parent # type: ignore
        xodr_file_path: Path = xodr_file_dir / f"{map_name.name}.xodr"
        if not xodr_file_dir.exists():
            raise FileNotFoundError(f"Could not find OpenDrive map dir at {xodr_file_dir}!")
        if not xodr_file_path.exists():
            raise FileNotFoundError(f"Could not find OpenDrive map data at {xodr_file_path}!")

        common_road_file_dir: Path = Path(os.getenv(key="COMMON_ROAD_MAP_FILE_DIR")) / map_name.parent # type: ignore
        common_road_file_path: Path = common_road_file_dir / f"{map_name.name}_common_road.xml"

        if not common_road_file_dir.exists():
            common_road_file_dir.mkdir(parents=True, exist_ok=True)
        if not common_road_file_path.exists():
            common_road_file_path.touch()
        
            logger.info(message=f"Converting OpenDrive XML files to CommonRoad format in {common_road_file_dir}")

            # load OpenDRIVE file, parse it, and convert it to a CommonRoad scenario
            scenario: Scenario = opendrive_to_commonroad(input_file=str(xodr_file_path))

            # store converted file as CommonRoad scenario
            writer = CommonRoadFileWriter(
                scenario=scenario,
                planning_problem_set=PlanningProblemSet(),
                source="CommonRoad Scenario Designer",
                author=os.getenv(key="COMMON_ROAD_CONVERTER_AUTHOR"), # type: ignore
                affiliation=os.getenv(key="COMMON_ROAD_CONVERTER_AFFILIATION"), # type: ignore
                tags={Tag.URBAN},
            )
            writer.write_to_file(filename=str(object=common_road_file_path),
                                overwrite_existing_file=OverwriteExistingFile.ALWAYS)
            logger.info(message=f"Wrote CommonRoad format to disk at {common_road_file_path}")

    def __convert_commonroad_to_lanelet2(self, map_name: Path, logger: RcutilsLogger) -> None:
        """See https://commonroad-scenario-designer.readthedocs.io/en/latest/details/lanelet2/
        Converts the map from the linked format to the lanelet2 compatible type"""
        common_road_file_dir: Path = Path(os.getenv(key="COMMON_ROAD_MAP_FILE_DIR")) / map_name.parent # type: ignore
        common_road_file_path: Path = common_road_file_dir / f"{map_name.name}_common_road.xml"

        lanelet2_road_file_dir: Path = Path(os.getenv(key="LANELET2_ROAD_MAP_FILE_DIR")) / map_name.parent # type: ignore
        lanelet2_road_file_path: Path = lanelet2_road_file_dir / f"{map_name.name}_lanelet2.osm"

        if not lanelet2_road_file_dir.exists():
            lanelet2_road_file_dir.mkdir(parents=True, exist_ok=True)
        if not lanelet2_road_file_path.exists():
            lanelet2_road_file_path.touch()
        
            logger.info(message=f"Converting CommonRoad XML files to lanelet2 in {lanelet2_road_file_dir}")
            try:
                commonroad_to_lanelet(input_file=str(common_road_file_path), output_name=str(lanelet2_road_file_path), proj=None) # type: ignore
                logger.info(message=f"Wrote Lanelet2 format to disk at {lanelet2_road_file_path}")
            except Exception as e:
                logger.error(message=getattr(e, 'message', repr(e)))
