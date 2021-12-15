import logging
from datetime import datetime
from io import BytesIO
from logging import Logger
from typing import Optional, Sequence
from uuid import UUID

import numpy as np
import PIL.Image as PILImage
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.inspection.inspection import (
    Image,
    ImageMetadata,
    Inspection,
    ThermalImage,
    ThermalImageMetadata,
    TimeIndexedPose,
)
from robot_interface.models.mission import (
    InspectionTask,
    TakeImage,
    TakeThermalImage,
    Task,
    TaskStatus,
)
from robot_interface.robot_interface import RobotInterface

from isar_turtlebot.config import config
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.turtlebot import Turtlebot


class Robot(RobotInterface):
    def __init__(self):
        self.logger: Logger = logging.getLogger("robot")

        self.turtlebot: Turtlebot = Turtlebot(bridge=RosBridge())

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
        now: datetime = datetime.utcnow()
        pose: Pose = self.turtlebot.get_robot_pose()

        if isinstance(inspection_task, TakeImage):
            image_metadata: ImageMetadata = ImageMetadata(
                start_time=now,
                time_indexed_pose=TimeIndexedPose(pose=pose, time=now),
                file_type=config.get("metadata", "image_filetype"),
            )
            image: Image = Image(metadata=image_metadata)

            self.turtlebot.bridge.visual_inspection.register_inspection_id(
                inspection_id=image.id
            )
            return [image]

        if isinstance(inspection_task, TakeThermalImage):
            image_metadata: ThermalImageMetadata = ThermalImageMetadata(
                start_time=now,
                time_indexed_pose=TimeIndexedPose(pose=pose, time=now),
                file_type=config.get("metadata", "thermal_image_filetype"),
            )
            thermal_image: ThermalImage = ThermalImage(metadata=image_metadata)

            self.turtlebot.bridge.visual_inspection.register_inspection_id(
                inspection_id=thermal_image.id
            )

            return [thermal_image]

        raise TypeError(
            f"Current task {inspection_task} is not a valid inspection task"
        )

    def download_inspection_result(self, inspection: Inspection) -> Inspection:
        if isinstance(inspection, Image):
            try:
                image_data = self.turtlebot.bridge.visual_inspection.read_image(
                    inspection_id=inspection.id
                )
                inspection.data = image_data
            except (KeyError, TypeError, FileNotFoundError) as e:
                self.logger.error("Failed to retrieve inspection result", e)
                raise FileNotFoundError("No inspection was found") from e

        elif isinstance(inspection, ThermalImage):
            try:
                image_data = self.turtlebot.bridge.visual_inspection.read_image(
                    inspection_id=inspection.id
                )
                image = PILImage.open(BytesIO(image_data))
                image_array = np.asarray(image)
                image_red = image_array[:, :, 0]
                image_grey = PILImage.fromarray(image_red)
                image_array_io = BytesIO()
                image_grey.save(image_array_io, format=inspection.metadata.file_type)

                inspection.data = image_array_io.getvalue()
            except Exception as e:
                self.logger.error("Failed to retrieve inspection result", e)
                raise FileNotFoundError("Failed to retrieve inspection result") from e
        return inspection

    def robot_pose(self) -> Pose:
        return self.turtlebot.get_robot_pose()
