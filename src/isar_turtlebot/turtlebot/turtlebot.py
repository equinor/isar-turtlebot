from typing import Optional

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
from robot_interface.models.mission.status import TaskStatus
from robot_interface.models.mission.task import (
    DriveToPose,
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

    def publish_task(self, task: Task) -> None:
        self.task_handler = self.task_handlers[type(task).__name__]
        task_input: Pose
        if isinstance(task, DriveToPose):
            task_input = task.pose
        elif isinstance(task, (TakeImage, TakeThermalImage)):
            task_input = get_inspection_pose(
                target=task.target, current_pose=self.get_robot_pose()
            )

        self.task_handler.start(task_input)

    def get_task_status(self) -> TaskStatus:
        if self.task_handler:
            status: Status = self.task_handler.get_status()
            return Status.map_to_task_status(status=status)

    def get_robot_pose(self) -> Pose:
        pose_message: dict = self.bridge.pose.get_value()
        pose: Pose = decode_pose_message(pose_message=pose_message)
        return pose
