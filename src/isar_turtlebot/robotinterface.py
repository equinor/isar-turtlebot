import logging
import os
from logging import Logger
from pathlib import Path
from queue import Queue
from threading import Thread
from typing import List, Sequence

from alitra import MapAlignment, Transform, align_maps
from robot_interface.models.initialize import InitializeParams
from robot_interface.models.inspection.inspection import Inspection
from robot_interface.models.mission.mission import Mission
from robot_interface.models.mission.status import MissionStatus, RobotStatus, StepStatus
from robot_interface.models.mission.step import InspectionStep, Step
from robot_interface.robot_interface import RobotInterface
from robot_interface.telemetry.mqtt_client import MqttTelemetryPublisher

from isar_turtlebot.config import settings
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge, RosBridgeInterface
from isar_turtlebot.turtlebot import Turtlebot


class Robot(RobotInterface):
    def __init__(self, bridge: RosBridgeInterface = RosBridge):
        map_alignment: MapAlignment = MapAlignment.from_config(
            Path(
                os.path.dirname(os.path.realpath(__file__)),
                f"config/maps/{settings.TURTLEBOT_MAP}.json",
            )
        )
        transform: Transform = align_maps(
            map_alignment.map_from, map_alignment.map_to, rot_axes="z"
        )
        self.turtlebot: Turtlebot = Turtlebot(bridge=bridge(), transform=transform)
        self.logger: Logger = logging.getLogger("isar_turtlebot")

    def initiate_mission(self, mission: Mission) -> None:
        self.logger.error("An invalid interface function was called")
        raise NotImplementedError

    def mission_status(self) -> MissionStatus:
        self.logger.error("An invalid interface function was called")
        raise NotImplementedError

    def initiate_step(self, step: Step) -> bool:
        self.turtlebot.publish_step(step=step)
        return True

    def step_status(self) -> StepStatus:
        return self.turtlebot.get_step_status()

    def stop(self) -> bool:
        return self.turtlebot.cancel_step()

    def get_inspections(self, step: InspectionStep) -> Sequence[Inspection]:
        return self.turtlebot.get_inspections(step.id)

    def initialize(self, params: InitializeParams) -> None:
        if params.initial_pose:
            self.turtlebot.set_initial_pose(params.initial_pose)

    def get_telemetry_publishers(
        self, queue: Queue, isar_id: str, robot_name: str
    ) -> List[Thread]:
        publisher_threads: List[Thread] = []

        pose_publisher: MqttTelemetryPublisher = MqttTelemetryPublisher(
            mqtt_queue=queue,
            telemetry_method=self.turtlebot.get_pose_telemetry,
            topic=f"isar/{isar_id}/pose",
            interval=1,
            retain=False,
        )
        pose_thread: Thread = Thread(
            target=pose_publisher.run,
            args=[isar_id, robot_name],
            name="ISAR Turtlebot Pose Publisher",
            daemon=True,
        )
        publisher_threads.append(pose_thread)

        battery_publisher: MqttTelemetryPublisher = MqttTelemetryPublisher(
            mqtt_queue=queue,
            telemetry_method=self.turtlebot.get_battery_telemetry,
            topic=f"isar/{isar_id}/battery",
            interval=1,
            retain=False,
        )
        battery_thread: Thread = Thread(
            target=battery_publisher.run,
            args=[isar_id, robot_name],
            name="ISAR Turtlebot Battery Publisher",
            daemon=True,
        )
        publisher_threads.append(battery_thread)

        return publisher_threads

    def robot_status(self) -> RobotStatus:
        if not self.turtlebot.bridge.is_connected():
            return RobotStatus.Offline
        if self.step_status() == StepStatus.InProgress:
            return RobotStatus.Busy
        return RobotStatus.Available
