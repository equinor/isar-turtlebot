import time
from typing import Optional

from isar_turtlebot.config import config
from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.turtlebot.taskhandlers.taskhandler import TaskHandler
from isar_turtlebot.utilities.pose_message import encode_pose_message
from robot_interface.models.geometry.pose import Pose


class DriveToHandler(TaskHandler):
    def __init__(
        self,
        bridge: RosBridge,
        publishing_timeout: float = config.getfloat("mission", "publishing_timeout"),
    ) -> None:
        self.bridge = bridge
        self.publishing_timeout = publishing_timeout

    def start(
        self,
        task_input: Pose,
    ) -> None:

        goal_pose: Pose = task_input
        goal_id: Optional[str] = self.goal_id()

        pose_message: dict = encode_pose_message(pose=goal_pose)
        self.bridge.execute_task.publish(message=pose_message)

        start_time: float = time.time()
        while self.goal_id() == goal_id:
            time.sleep(0.1)
            if (time.time() - start_time) > self.publishing_timeout:
                raise TimeoutError("Publishing navigation message timed out")

    def goal_id(self) -> Optional[str]:
        message: dict = self.bridge.task_status.get_value()

        try:
            return message["status_list"][0]["goal_id"]["id"]
        except (KeyError, IndexError):
            return None

    def get_status(self) -> Status:
        message: dict = self.bridge.task_status.get_value()
        return self.status_from_message(message=message)
