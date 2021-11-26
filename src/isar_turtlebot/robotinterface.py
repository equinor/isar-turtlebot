import logging
import time
from datetime import datetime
from io import BytesIO
from logging import Logger
from typing import Any, Optional, Sequence, Tuple

import numpy as np
import PIL.Image as PILImage
from robot_interface.models.geometry.frame import Frame
from robot_interface.models.geometry.joints import Joints
from robot_interface.models.geometry.orientation import Orientation
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.geometry.position import Position
from robot_interface.models.inspection.formats.image import Image, ThermalImage
from robot_interface.models.inspection.inspection import (
    Inspection,
    InspectionResult,
    TimeIndexedPose,
)
from robot_interface.models.inspection.metadata import ImageMetadata
from robot_interface.models.inspection.references import ImageReference
from robot_interface.models.inspection.references.image_reference import (
    ImageReference,
    ThermalImageReference,
)
from robot_interface.models.mission import (
    MissionStatus,
    Step,
    TakeImage,
    TakeThermalImage,
)
from robot_interface.robot_interface import RobotInterface

from isar_turtlebot.config import config
from isar_turtlebot.models.turtlebot_status import TurtlebotStatus
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.task_manager.task_manager import TaskManager


class Robot(RobotInterface):
    def __init__(self):
        self.logger: Logger = logging.getLogger("robot")

        self.bridge: RosBridge = RosBridge()

        self.task_manager: TaskManager = TaskManager(self.bridge)

    def schedule_step(self, step: Step) -> Tuple[bool, Optional[Any], Optional[Joints]]:
        run_id: str = self.task_manager.publish_task(step=step)
        return True, run_id, None

    def mission_scheduled(self) -> bool:
        return False

    def mission_status(self, mission_id: Any) -> MissionStatus:
        mission_status: MissionStatus = TurtlebotStatus.get_mission_status(
            status=self.task_manager.task_status()
        )
        return mission_status

    def abort_mission(self) -> bool:
        return True

    def log_status(
        self, mission_id: Any, mission_status: MissionStatus, current_step: Step
    ):
        self.logger.info(f"Mission ID: {mission_id}")
        self.logger.info(f"Mission Status: {mission_status}")
        self.logger.info(f"Current Step: {current_step}")

    def get_inspection_references(
        self, vendor_mission_id: Any, current_step: Step
    ) -> Sequence[Inspection]:
        now: datetime = datetime.utcnow()

        if isinstance(current_step, TakeImage):
            self.task_manager.filenames[
                vendor_mission_id
            ] = self.task_manager.current_filename
            pose: Pose = self.robot_pose()
            image_metadata: ImageMetadata = ImageMetadata(
                start_time=now,
                time_indexed_pose=TimeIndexedPose(pose=pose, time=now),
                file_type=config.get("metadata", "image_filetype"),
            )
            image_ref: ImageReference = ImageReference(
                id=vendor_mission_id, metadata=image_metadata
            )
        elif isinstance(current_step, TakeThermalImage):
            self.task_manager.filenames[
                vendor_mission_id
            ] = self.task_manager.current_filename
            pose: Pose = self._get_robot_pose()
            image_metadata: ImageMetadata = ImageMetadata(
                start_time=now,
                time_indexed_pose=TimeIndexedPose(pose=pose, time=now),
                file_type=config.get("metadata", "thermal_image_filetype"),
            )
            image_ref: ThermalImageReference = ThermalImageReference(
                id=vendor_mission_id, metadata=image_metadata
            )

        return [image_ref]

    def download_inspection_result(
        self, inspection: Inspection
    ) -> Optional[InspectionResult]:
        if isinstance(inspection, ImageReference):
            try:
                image_data = self.read_image(run_id=inspection.id)

                inspection_result = Image(
                    id=inspection.id, metadata=inspection.metadata, data=image_data
                )
            except (KeyError, TypeError, FileNotFoundError) as e:
                self.logger("Failed to retreive inspection result", e)
                inspection_result = None
        elif isinstance(inspection, ThermalImageReference):
            try:
                image_data = self.read_image(run_id=inspection.id)
                image = PILImage.open(BytesIO(image_data))
                image_array = np.asarray(image)
                image_red = image_array[:, :, 0]
                image_grey = PILImage.fromarray(image_red)
                image_array_io = BytesIO()
                image_grey.save(image_array_io, format=inspection.metadata.file_type)
                inspection_result = ThermalImage(
                    id=inspection.id,
                    metadata=inspection.metadata,
                    data=image_array_io.getvalue(),
                )
            except Exception as e:
                self.logger.error("Failed to retreive inspection result", e)
                inspection_result = None
        return inspection_result

    def robot_pose(self) -> Pose:
        return self._get_robot_pose()

    def _get_robot_pose(self) -> Pose:

        pose_message: dict = self.bridge.pose.get_value()
        position_message: dict = pose_message["pose"]["pose"]["position"]
        orientation_message: dict = pose_message["pose"]["pose"]["orientation"]

        pose: Pose = Pose(
            position=Position(
                x=position_message["x"],
                y=position_message["y"],
                z=position_message["z"],
                frame=Frame.Robot,
            ),
            orientation=Orientation(
                x=orientation_message["x"],
                y=orientation_message["y"],
                z=orientation_message["z"],
                w=orientation_message["w"],
                frame=Frame.Robot,
            ),
            frame=Frame.Robot,
        )
        return pose

    def robot_pose(self) -> Pose:
        return self.task_manager.get_robot_pose()

    def read_image(self, run_id: str) -> bytes:
        filename: Path = self.task_manager.filenames[run_id]
        with open(filename, "rb") as image_file:
            image_data = image_file.read()

        return image_data
