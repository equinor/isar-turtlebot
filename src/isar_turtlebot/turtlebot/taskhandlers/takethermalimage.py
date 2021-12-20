import base64
import time
from io import BytesIO
from pathlib import Path
from typing import Optional
from uuid import uuid4

import numpy as np
import PIL.Image as PILImage
from isar_turtlebot.config import config
from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.turtlebot.taskhandlers.taskhandler import TaskHandler
from isar_turtlebot.utilities.pose_message import encode_pose_message
from robot_interface.models.geometry.pose import Pose


class TakeThermalImageHandler(TaskHandler):
    def __init__(
        self,
        bridge: RosBridge,
        storage_folder: Path = Path(config.get("storage", "storage_folder")),
        thermal_image_filetype: str = config.get("metadata", "thermal_image_filetype"),
        publishing_timeout: float = config.getfloat("mission", "publishing_timeout"),
        inspection_pose_timeout: float = config.getfloat(
            "mission", "inspection_pose_timeout"
        ),
    ) -> None:
        self.bridge = bridge
        self.storage_folder = storage_folder
        self.thermal_image_filetype = thermal_image_filetype
        self.publishing_timeout = publishing_timeout
        self.inspection_pose_timeout = inspection_pose_timeout

        self.status: Optional[Status] = None

        self.filename: Optional[Path] = None

    def start(
        self,
        task_input: Pose,
    ) -> None:

        self.status = Status.Active
        inspection_pose: Pose = task_input

        pose_message: dict = encode_pose_message(pose=inspection_pose)
        goal_id: Optional[str] = self._goal_id()
        self.bridge.execute_task.publish(message=pose_message)

        start_time: float = time.time()
        while self._goal_id() == goal_id:
            time.sleep(0.1)
            if (time.time() - start_time) > self.publishing_timeout:
                self.status = Status.Failure
                raise TimeoutError("Publishing navigation message timed out.")

        start_time: float = time.time()
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

        self.status = Status.Succeeded

    def get_status(self) -> Status:
        return self.status

    def _goal_id(self) -> Optional[str]:
        goal_id: str = self.goal_id_from_message(
            message=self.bridge.task_status.get_value()
        )
        return goal_id

    def _move_status(self) -> Status:
        move_status: Status = self.status_from_message(
            message=self.bridge.task_status.get_value()
        )
        return move_status

    def _write_image_bytes(self):
        encoded_image_data: bytes = self._get_image_data()
        image_bytes: bytes = base64.b64decode(encoded_image_data)
        image_bytes: bytes = self._convert_to_thermal(image_bytes)

        self.filename: Path = Path(
            f"{self.storage_folder.as_posix()}/{str(uuid4())}.{self.thermal_image_filetype}"
        )

        self.filename.parent.mkdir(exist_ok=True)

        with open(self.filename, "wb") as image_file:
            image_file.write(image_bytes)

    def _get_image_data(self):
        encoded_image_data: bytes = self.bridge.visual_inspection.get_image()
        return encoded_image_data

    def _convert_to_thermal(self, image_bytes: bytes) -> bytes:
        image = PILImage.open(BytesIO(image_bytes))
        image_array = np.asarray(image)
        image_gray_array = image_array[:, :, 0]
        image_gray_image = PILImage.fromarray(image_gray_array)

        image_array_io = BytesIO()
        image_gray_image.save(image_array_io, format=self.thermal_image_filetype)

        return image_array_io.getvalue()
