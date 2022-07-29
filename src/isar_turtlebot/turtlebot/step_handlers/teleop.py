import time
from typing import Optional

from alitra import Frame, Pose, Transform
from robot_interface.models.mission import TeleOp

from isar_turtlebot.models.turtlebot_status import Status
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.settings import settings
from isar_turtlebot.turtlebot.step_handlers.stephandler import StepHandler
from isar_turtlebot.utilities.pose_message import encode_pose_message


class TeleOpHandler(StepHandler):
    def __init__(
        self,
        bridge: RosBridge,
        transform: Transform,
        publishing_timeout: float = settings.PUBLISHING_TIMEOUT,
    ) -> None:
        self.bridge: RosBridge = bridge
        self.transform: Transform = transform
        self.publishing_timeout: float = publishing_timeout

        self.status: Optional[Status] = None

    def start(
        self,
        step: TeleOp,
    ) -> None:

        self.status = Status.Active

        self.bridge.teleop_activate.publish(message={"data": True})

        while self.activated():
            time.sleep(0.1)
        
        self.status = Status.Succeeded
    
    def activated(self):
        message = self.bridge.teleop_activate.get_value()
        if not message:
            return True
        return message["data"]

    def get_status(self) -> Status:
        return self.status
