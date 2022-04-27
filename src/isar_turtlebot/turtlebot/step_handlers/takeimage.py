import base64
import time
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import uuid4

from alitra import Pose, Position, Transform
from robot_interface.models.inspection.inspection import (
    Image,
    ImageMetadata,
    TimeIndexedPose,
)
from robot_interface.models.mission import TakeImage

from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.settings import settings
from isar_turtlebot.turtlebot.step_handlers.stephandler import StepHandler
from isar_turtlebot.utilities.inspection_pose import get_inspection_pose
from isar_turtlebot.utilities.pose_message import (
    decode_pose_message,
    encode_pose_message,
)


class TakeImageHandler(StepHandler):
    def __init__(
        self,
        bridge: RosBridge,
        transform: Transform,
        storage_folder: Path = Path(settings.STORAGE_FOLDER),
        image_filetype: str = settings.IMAGE_FILETYPE,
        publishing_timeout: float = settings.PUBLISHING_TIMEOUT,
        inspection_pose_timeout: float = settings.INSPECTION_POSE_TIMEOUT,
    ) -> None:
        self.bridge: RosBridge = bridge
        self.transform: Transform = transform
        self.storage_folder: Path = storage_folder
        self.image_filetype: str = image_filetype
        self.publishing_timeout: float = publishing_timeout
        self.inspection_pose_timeout: float = inspection_pose_timeout

        self.status: Optional[Status] = None

        self.filename: Optional[Path] = None
        self.inspection: Optional[Image] = None

    def start(self, step: TakeImage) -> None:

        self.status = Status.Active

        current_pose: Pose = self._get_robot_pose()
        target: Position = self.transform.transform_position(
            positions=step.target,
            from_=self.transform.from_,
            to_=self.transform.to_,
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
                raise TimeoutError("Publishing navigation message timed out.")

        start_time = time.time()
        while self._move_status() is not Status.Succeeded:
            time.sleep(0.1)
            execution_time: float = time.time() - start_time
            if execution_time > self.inspection_pose_timeout:
                self.status = Status.Failure
                raise TimeoutError("Navigation to inspection pose timed out.")

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
        image_metadata: ImageMetadata = ImageMetadata(
            start_time=timestamp,
            time_indexed_pose=TimeIndexedPose(pose=pose, time=timestamp),
            file_type=settings.THERMAL_IMAGE_FILETYPE,
        )

        self.inspection = Image(metadata=image_metadata)

        self.status = Status.Succeeded

    def get_status(self) -> Status:
        return self.status

    def get_inspection(self) -> Image:
        return self.inspection

    def get_filename(self) -> Path:
        return self.filename

    def _get_robot_pose(self) -> Pose:
        pose_message: dict = self.bridge.pose.get_value()
        return decode_pose_message(pose_message=pose_message)

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

    def _write_image_bytes(self):
        image_data: str = self.bridge.visual_inspection.get_image()
        image_bytes: bytes = base64.b64decode(image_data)
        self.filename: Path = Path(
            f"{self.storage_folder.as_posix()}/{str(uuid4())}.{self.image_filetype}"
        )

        self.filename.parent.mkdir(exist_ok=True)

        with open(self.filename, "wb") as image_file:
            image_file.write(image_bytes)
