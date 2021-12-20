import logging
from io import BytesIO
from logging import Logger
from typing import Optional, Sequence
from uuid import UUID

import numpy as np
import PIL.Image as PILImage
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.inspection.inspection import Image, Inspection, ThermalImage
from robot_interface.models.mission import InspectionTask, Task, TaskStatus
from robot_interface.robot_interface import RobotInterface

from isar_turtlebot.config import config
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge, RosBridgeInterface
from isar_turtlebot.turtlebot import Turtlebot


class Robot(RobotInterface):
    def __init__(self, bridge: RosBridgeInterface = RosBridge):
        self.logger: Logger = logging.getLogger("robot")

        self.turtlebot: Turtlebot = Turtlebot(bridge=bridge())

    def schedule_task(self, task: Task) -> bool:
        self.turtlebot.publish_task(task=task)
        return True

    def mission_scheduled(self) -> bool:
        return False

    def task_status(self, task_id: Optional[UUID]) -> TaskStatus:
        return self.turtlebot.get_task_status()

    def abort_mission(self) -> bool:
        return True

    def log_status(self, task_status: TaskStatus, current_task: Task):
        self.logger.info(f"Task Status: {task_status}")
        self.logger.info(f"Current Task: {current_task}")

    def get_inspection_references(
        self, inspection_task: InspectionTask
    ) -> Sequence[Inspection]:
        try:
            return self.turtlebot.set_inspection_references(inspection_task)
        except TypeError as e:
            raise TypeError from e

    def download_inspection_result(self, inspection: Inspection) -> Inspection:
        if isinstance(inspection, (Image, ThermalImage)):
            try:
                image_data = self.turtlebot.read_image(inspection_id=inspection.id)
                inspection.data = image_data
            except (KeyError, TypeError, FileNotFoundError) as e:
                self.logger.error("Failed to retrieve inspection result", e)
                raise FileNotFoundError("No inspection was found") from e

        return inspection

    def robot_pose(self) -> Pose:
        return self.turtlebot.get_robot_pose()
