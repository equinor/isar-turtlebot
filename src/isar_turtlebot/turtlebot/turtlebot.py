from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence
from uuid import UUID

from isar_turtlebot.config import config
from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge import RosBridge
from isar_turtlebot.turtlebot.taskhandlers import (
    DriveToHandler,
    TakeImageHandler,
    TakeThermalImageHandler,
)
from isar_turtlebot.turtlebot.taskhandlers.taskhandler import TaskHandler
from isar_turtlebot.utilities.inspection_pose import get_inspection_pose
from isar_turtlebot.utilities.pose_message import decode_pose_message
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.inspection.inspection import (
    Image,
    ImageMetadata,
    Inspection,
    ThermalImage,
    ThermalImageMetadata,
    TimeIndexedPose,
)
from robot_interface.models.mission.status import TaskStatus
from robot_interface.models.mission.task import (
    DriveToPose,
    InspectionTask,
    TakeImage,
    TakeThermalImage,
    Task,
)


class Turtlebot:
    """Task manager for Turtlebot."""

    def __init__(self, bridge: RosBridge) -> None:

        self.bridge: RosBridge = bridge
        self.status: Optional[Status] = None

        self.task_handlers = {
            "DriveToPose": DriveToHandler(bridge=self.bridge),
            "TakeImage": TakeImageHandler(bridge=self.bridge),
            "TakeThermalImage": TakeThermalImageHandler(bridge=self.bridge),
        }

        self.task_handler: Optional[TaskHandler] = None

        self.filenames: dict = dict()

    def publish_task(self, task: Task) -> None:
        task_input: Pose
        if isinstance(task, DriveToPose):
            task_input = task.pose
        elif isinstance(task, (TakeImage, TakeThermalImage)):
            task_input = get_inspection_pose(
                target=task.target, current_pose=self.get_robot_pose()
            )

        self.task_handler = self.task_handlers[type(task).__name__]
        self.task_handler.start(task_input)

    def get_task_status(self) -> TaskStatus:
        if self.task_handler:
            status: Status = self.task_handler.get_status()
            return Status.map_to_task_status(status=status)

    def get_robot_pose(self) -> Pose:
        pose_message: dict = self.bridge.pose.get_value()
        pose: Pose = decode_pose_message(pose_message=pose_message)
        return pose

    def register_inspection_id(self, inspection_id: UUID) -> None:
        self.filenames[inspection_id] = self.task_handler.filename

    def read_image(self, inspection_id: UUID) -> bytes:
        filename: Path = self.filenames[inspection_id]
        with open(filename, "rb") as image_file:
            image_data = image_file.read()

        return image_data

    def set_inspection_references(
        self, inspection_task: InspectionTask
    ) -> Sequence[Inspection]:
        now: datetime = datetime.utcnow()
        pose: Pose = self.get_robot_pose()

        if isinstance(inspection_task, TakeImage):
            image_metadata: ImageMetadata = ImageMetadata(
                start_time=now,
                time_indexed_pose=TimeIndexedPose(pose=pose, time=now),
                file_type=config.get("metadata", "image_filetype"),
            )
            image: Image = Image(metadata=image_metadata)

            self.register_inspection_id(inspection_id=image.id)
            return [image]

        if isinstance(inspection_task, TakeThermalImage):
            image_metadata: ThermalImageMetadata = ThermalImageMetadata(
                start_time=now,
                time_indexed_pose=TimeIndexedPose(pose=pose, time=now),
                file_type=config.get("metadata", "thermal_image_filetype"),
            )
            thermal_image: ThermalImage = ThermalImage(metadata=image_metadata)
            self.register_inspection_id(inspection_id=thermal_image.id)

            return [thermal_image]

        raise TypeError(
            f"Current task {inspection_task} is not a valid inspection task"
        )
