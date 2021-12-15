import time
from typing import Optional

from isar_turtlebot.config import config
from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.turtlebot.taskhandlers.taskhandler import TaskHandler
from isar_turtlebot.utilities.pose_message import encode_pose_message
from robot_interface.models.geometry.pose import Pose


class TakeImageHandler(TaskHandler):
    def __init__(
        self,
        bridge: RosBridge,
        publishing_timeout: float = config.getfloat("mission", "publishing_timeout"),
        inspection_pose_timeout: float = config.getfloat(
            "mission", "inspection_pose_timeout"
        ),
        take_image_timeout: float = config.getfloat("mission", "take_image_timeout"),
    ) -> None:
        self.bridge = bridge
        self.publishing_timeout = publishing_timeout
        self.inspection_pose_timeout = inspection_pose_timeout
        self.take_image_timeout = take_image_timeout

        self.status: Optional[Status] = None

    def start(
        self,
        task_input: Pose,
    ) -> None:

        self.status = Status.Active
        inspection_pose: Pose = task_input

        pose_message: dict = encode_pose_message(pose=inspection_pose)
        goal_id: Optional[str] = self.goal_id()
        self.bridge.execute_task.publish(message=pose_message)

        start_time: float = time.time()
        while self.goal_id() == goal_id:
            time.sleep(0.1)
            if (time.time() - start_time) > self.publishing_timeout:
                self.status = Status.Failure
                raise TimeoutError("Publishing navigation message timed out.")

        start_time: float = time.time()
        while self.move_status() is not Status.Succeeded:
            time.sleep(0.1)
            execution_time: float = time.time() - start_time
            if execution_time > self.inspection_pose_timeout:
                self.status = Status.Failure
                raise TimeoutError("Navigation to inspection pose timed out.")

        self.bridge.visual_inspection.take_image()
        start_time: float = time.time()
        while not self.bridge.visual_inspection.stored_image():
            time.sleep(0.1)
            execution_time: float = time.time() - start_time
            if execution_time > self.take_image_timeout:
                self.status = Status.Failure
                raise TimeoutError("Storing image for Turtlebot3 timed out.")

        self.status = Status.Succeeded

    def goal_id(self) -> Optional[str]:
        message: dict = self.bridge.task_status.get_value()

        try:
            return message["status_list"][0]["goal_id"]["id"]
        except (KeyError, IndexError):
            return None

    def get_status(self) -> Status:
        return self.status

    def move_status(self) -> Status:
        message: dict = self.bridge.task_status.get_value()
        return self.status_from_message(message=message)
