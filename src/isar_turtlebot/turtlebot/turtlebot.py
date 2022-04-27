import logging
from logging import Logger
from pathlib import Path
from typing import Optional, Sequence
from uuid import UUID

from alitra import Pose, Transform
from robot_interface.models.exceptions import (
    RobotCommunicationException,
    RobotException,
)
from robot_interface.models.inspection.inspection import Inspection
from robot_interface.models.mission import InspectionStep, Step, StepStatus

from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge import RosBridge
from isar_turtlebot.turtlebot.step_handlers import (
    DriveToHandler,
    TakeImageHandler,
    TakeThermalImageHandler,
)
from isar_turtlebot.turtlebot.step_handlers.stephandler import StepHandler
from isar_turtlebot.utilities.pose_message import decode_pose_message


class Turtlebot:
    """Step manager for Turtlebot."""

    def __init__(self, bridge: RosBridge, transform: Transform) -> None:

        self.logger: Logger = logging.getLogger("robot")
        self.bridge: RosBridge = bridge
        self.transform: Transform = transform
        self.status: Optional[Status] = None

        self.step_handlers = {
            "DriveToPose": DriveToHandler(bridge=self.bridge, transform=transform),
            "TakeImage": TakeImageHandler(bridge=self.bridge, transform=transform),
            "TakeThermalImage": TakeThermalImageHandler(
                bridge=self.bridge, transform=transform
            ),
        }

        self.step_handler: Optional[StepHandler] = None

        self.filenames: dict = dict()
        self.inspections: dict = dict()

    def get_pose(self) -> Pose:
        pose_message: dict = self.bridge.pose.get_value()
        return decode_pose_message(pose_message=pose_message)

    def publish_step(self, step: Step) -> None:
        self.step_handler = self.step_handlers[type(step).__name__]
        try:
            self.step_handler.start(step)
        except TimeoutError as e:
            raise RobotCommunicationException from e

        if isinstance(step, InspectionStep):
            self.filenames[step.id] = self.step_handler.get_filename()
            self.inspections[step.id] = self.step_handler.get_inspection()

    def get_step_status(self) -> StepStatus:
        if self.step_handler:
            status: Status = self.step_handler.get_status()
            return Status.map_to_step_status(status=status)

    def get_inspections(self, id: UUID) -> Sequence[Inspection]:
        try:
            inspection: Inspection = self.inspections[id]
        except KeyError as e:
            self.logger.warning(f"No inspection connected to step: {id}!")
            raise RobotException from e
        try:
            inspection.data = self._read_data(id)
        except FileNotFoundError as e:
            self.logger.warning(f"No data file connected to step: {id}!")
            raise RobotException from e
        return [inspection]

    def _read_data(self, inspection_id: UUID) -> bytes:
        filename: Path = self.filenames[inspection_id]
        with open(filename, "rb") as image_file:
            image_data = image_file.read()
        return image_data
