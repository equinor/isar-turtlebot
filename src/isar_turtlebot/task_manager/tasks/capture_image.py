import base64
import logging
import time
from pathlib import Path
from typing import Optional
from uuid import uuid4

from isar_turtlebot.config import config
from isar_turtlebot.models.turtlebot_status import TurtlebotStatus
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge


class CaptureImage:
    def __init__(
        self,
        bridge: RosBridge,
        storage_folder: Path = Path(config.get("storage", "storage_folder")),
    ):
        self.name = "captureimage"

        self.bridge: RosBridge = bridge
        self.logger = logging.getLogger("robot")

        self.storage_folder: Path = storage_folder
        self.filename: Optional[str] = None
        self.task_status: Optional[TurtlebotStatus] = None
        self.inspection_timeout = config.getfloat("mission", "inspection_timeout")

    def run(self):
        self.task_status: TurtlebotStatus = TurtlebotStatus.Active

        self._write_image_bytes()
        if not self.filename.is_file():
            self.task_status = TurtlebotStatus.Failure
            raise CaptureImageError
        self.task_status = TurtlebotStatus.Succeeded
        return self.filename

    def _write_image_bytes(self):
        image_bytes = base64.b64decode(self._get_image_data())
        self.filename: Path = Path(
            f"{self.storage_folder.as_posix()}/{str(uuid4())}.jpeg"
        )
        self.filename.parent.mkdir(exist_ok=True)

        with open(self.filename, "wb") as image_file:
            image_file.write(image_bytes)

    def _get_image_data(self):
        self.bridge.compressed_image.subscribe()
        try:
            image_data = self._read_image_data()
        except TimeoutError:
            image_data = None
            self.task_status = TurtlebotStatus.Failure
        self.bridge.compressed_image.unsubscribe()
        return image_data

    def _read_image_data(self) -> str:
        start_time: float = time.time()
        while not self.bridge.compressed_image.get_image():
            time.sleep(0.01)
            execution_time: float = time.time() - start_time
            if execution_time > self.inspection_timeout:
                raise TimeoutError(f"Unable to read image data from topic")
        return self.bridge.compressed_image.get_image()


class CaptureImageError(BaseException):
    pass
