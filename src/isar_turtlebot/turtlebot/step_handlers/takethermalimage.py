import base64
import logging
import time
from datetime import datetime
from io import BytesIO
from logging import Logger
from pathlib import Path
from typing import Optional
from uuid import uuid4

import PIL.Image as PILImage
import numpy as np
from alitra import Pose, Position, Transform
from robot_interface.models.exceptions.robot_exceptions import (
    RobotCommunicationException,
    RobotInfeasibleStepException,
)
from robot_interface.models.inspection.inspection import (
    ThermalImage,
    ThermalImageMetadata,
)
from robot_interface.models.mission.step import TakeThermalImage

from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.settings import settings
from isar_turtlebot.turtlebot.step_handlers.stephandler import StepHandler
from isar_turtlebot.utilities.inspection_pose import get_inspection_pose
from isar_turtlebot.utilities.pose_message import (
    decode_pose_message,
    encode_pose_message,
)


class TakeThermalImageHandler(StepHandler):
    def __init__(
        self,
        bridge: RosBridge,
        transform: Transform,
        storage_folder: Path = Path(settings.STORAGE_FOLDER),
        thermal_image_filetype: str = settings.THERMAL_IMAGE_FILETYPE,
        publishing_timeout: float = settings.PUBLISHING_TIMEOUT,
        inspection_pose_timeout: float = settings.INSPECTION_POSE_TIMEOUT,
    ) -> None:
        self.logger: Logger = logging.getLogger(settings.LOGGER_NAME)

        self.bridge: RosBridge = bridge
        self.transform: Transform = transform
        self.storage_folder: Path = storage_folder
        self.thermal_image_filetype: str = thermal_image_filetype
        self.publishing_timeout: float = publishing_timeout
        self.inspection_pose_timeout: float = inspection_pose_timeout

        self.status: Optional[Status] = None

        self.filename: Optional[Path] = None
        self.inspection: Optional[ThermalImage] = None

    def start(
        self,
        step: TakeThermalImage,
    ) -> None:
        self.status = Status.Active
        current_pose: Pose = self._get_robot_pose()
        target: Position = self.transform.transform_position(
            positions=step.target,
            from_=self.transform.to_,
            to_=self.transform.from_,
        )
        inspection_pose: Pose = get_inspection_pose(
            current_pose=current_pose, target=target
        )

        pose_message: dict = encode_pose_message(pose=inspection_pose)
        goal_id: Optional[str] = self._goal_id()
        self.bridge.execute_step.publish(message=pose_message)

        start_time: float = time.time()
        while self._goal_id() == goal_id:
            time.sleep(0.1)
            if (time.time() - start_time) > self.publishing_timeout:
                self.status = Status.Failure
                error_description: str = "Publishing navigation message timed out"
                self.logger.error(error_description)
                raise RobotCommunicationException(error_description=error_description)

        start_time = time.time()
        while self._move_status() is not Status.Succeeded:
            time.sleep(0.1)
            execution_time: float = time.time() - start_time
            if execution_time > self.inspection_pose_timeout:
                self.status = Status.Failure
                error_description = "Navigation to inspection pose timed out"
                self.logger.error(error_description)
                raise RobotInfeasibleStepException(error_description=error_description)

        self._write_image_bytes()

        if not self.filename.is_file():
            self.status = Status.Failure
            return

        pose: Pose = self.transform.transform_pose(
            pose=self._get_robot_pose(),
            from_=self.transform.from_,
            to_=self.transform.to_,
        )
        timestamp: datetime = datetime.utcnow()
        image_metadata: ThermalImageMetadata = ThermalImageMetadata(
            pose=pose,
            start_time=timestamp,
            file_type=settings.THERMAL_IMAGE_FILETYPE,
        )

        self.inspection = ThermalImage(metadata=image_metadata)
        self.status = Status.Succeeded

    def get_status(self) -> Status:
        return self.status

    def get_inspection(self) -> ThermalImage:
        return self.inspection

    def get_filename(self) -> Path:
        return self.filename

    def _get_robot_pose(self) -> Pose:
        pose_message: dict = self.bridge.pose.get_value()
        pose: Pose = decode_pose_message(pose_message=pose_message)
        return pose

    def _goal_id(self) -> Optional[str]:
        goal_id: str = self.goal_id_from_message(
            message=self.bridge.step_status.get_value()
        )
        return goal_id

    def _move_status(self) -> Status:
        move_status: Status = self.status_from_message(
            message=self.bridge.step_status.get_value()
        )
        return move_status

    def _write_image_bytes(self) -> None:
        encoded_image_data: bytes = self.bridge.visual_inspection.get_image()
        image_bytes: bytes = base64.b64decode(encoded_image_data)
        image_bytes = self._convert_to_thermal(image_bytes)

        self.filename = Path(
            f"{self.storage_folder.as_posix()}/{str(uuid4())}"
            f".{self.thermal_image_filetype}"
        )

        self.filename.parent.mkdir(exist_ok=True)

        with open(self.filename, "wb") as image_file:
            image_file.write(image_bytes)

    def _convert_to_thermal(self, image_bytes: bytes) -> bytes:
        image = PILImage.open(BytesIO(image_bytes))
        image_array = np.asarray(image)
        image_gray_array = image_array[:, :, 0]
        image_gray_image = PILImage.fromarray(image_gray_array)

        image_array_io = BytesIO()
        image_gray_image.save(image_array_io, format=self.thermal_image_filetype)

        return image_array_io.getvalue()
