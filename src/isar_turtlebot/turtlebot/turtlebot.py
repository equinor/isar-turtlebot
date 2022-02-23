import logging
from logging import Logger
from pathlib import Path
from typing import Optional, Sequence
from uuid import UUID

from isar.services.coordinates.transformation import Transformation
from robot_interface.models.exceptions import RobotException
from robot_interface.models.inspection.inspection import Inspection
from robot_interface.models.mission.status import TaskStatus
from robot_interface.models.mission.task import InspectionTask, Task

from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge import RosBridge
from isar_turtlebot.turtlebot.taskhandlers import (
    DriveToHandler,
    TakeImageHandler,
    TakeThermalImageHandler,
)
from isar_turtlebot.turtlebot.taskhandlers.taskhandler import TaskHandler


class Turtlebot:
    """Task manager for Turtlebot."""

    def __init__(self, bridge: RosBridge, transform: Transformation) -> None:

        self.logger: Logger = logging.getLogger("robot")
        self.bridge: RosBridge = bridge
        self.transform: Transformation = transform
        self.status: Optional[Status] = None

        self.task_handlers = {
            "DriveToPose": DriveToHandler(bridge=self.bridge, transform=transform),
            "TakeImage": TakeImageHandler(bridge=self.bridge, transform=transform),
            "TakeThermalImage": TakeThermalImageHandler(
                bridge=self.bridge, transform=transform
            ),
        }

        self.task_handler: Optional[TaskHandler] = None

        self.filenames: dict = dict()
        self.inspections: dict = dict()

    def publish_task(self, task: Task) -> None:
        self.task_handler = self.task_handlers[type(task).__name__]
        try:
            self.task_handler.start(task)
        except TimeoutError as e:
            raise RobotCommunicationException from e

        if isinstance(task, InspectionTask):
            self.filenames[task.id] = self.task_handler.get_filename()
            self.inspections[task.id] = self.task_handler.get_inspection()

    def get_task_status(self) -> TaskStatus:
        if self.task_handler:
            status: Status = self.task_handler.get_status()
            return Status.map_to_task_status(status=status)

    def get_inspections(self, id: UUID) -> Sequence[Inspection]:
        try:
            inspection: Inspection = self.inspections[id]
        except KeyError as e:
            self.logger.warning(f"No inspection connected to task: {id}!")
            raise RobotException from e
        try:
            inspection.data = self._read_data(id)
        except FileNotFoundError as e:
            self.logger.warning(f"No data file connected to task: {id}!")
            raise RobotException from e
        return [inspection]

    def _read_data(self, inspection_id: UUID) -> bytes:
        filename: Path = self.filenames[inspection_id]
        with open(filename, "rb") as image_file:
            image_data = image_file.read()
        return image_data
