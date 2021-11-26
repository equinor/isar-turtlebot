import logging
import time
from typing import Optional

from isar_turtlebot.config import config
from isar_turtlebot.models.turtlebot_status import TurtlebotStatus
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from robot_interface.models.geometry.pose import Pose


class Navigate:
    def __init__(
        self,
        bridge: RosBridge,
    ):
        self.name = "navigate"
        self.bridge: RosBridge = bridge
        self.logger = logging.getLogger("robot")

        self.task_status: Optional[TurtlebotStatus] = None
        self.navigation_timeout = config.getfloat("mission", "navigation_timeout")

    def run(self, pose: Pose):
        start_time = time.time()
        self._go_to_pose(pose=pose)
        self.task_status = self.navigation_status()

        while self.task_status is not TurtlebotStatus.Succeeded:
            time.sleep(0.1)
            execution_time = time.time() - start_time

            if self.navigation_status() == TurtlebotStatus.Failure:
                self.logger.error("Navigation Failure")
                raise NavigationError

            elif execution_time > self.navigation_timeout:
                self.task_status = TurtlebotStatus.Failure
                self.logger.error("Navigation Timout")
                raise TimeoutError
            self.task_status = self.navigation_status()

    def navigation_status(self) -> TurtlebotStatus:
        message: dict = self.bridge.mission_status.get_value()
        task_status: TurtlebotStatus = TurtlebotStatus.map_to_turtlebot_status(
            message["status_list"][0]["status"]
        )
        return task_status

    def _go_to_pose(self, pose: Pose):
        pose_message = self._get_pose_message(pose=pose)
        self.bridge.execute_task.publish(message=pose_message)

    def _get_pose_message(self, pose: Pose) -> dict:
        pose_message: dict = {
            "goal": {
                "target_pose": {
                    "header": {
                        "seq": 0,
                        "stamp": {"secs": 1533, "nsecs": 746000000},
                        "frame_id": "map",
                    },
                    "pose": {
                        "position": {
                            "x": pose.position.x,
                            "y": pose.position.y,
                            "z": pose.position.z,
                        },
                        "orientation": {
                            "x": pose.orientation.x,
                            "y": pose.orientation.y,
                            "z": pose.orientation.z,
                            "w": pose.orientation.w,
                        },
                    },
                }
            },
        }
        return pose_message


class NavigationError(BaseException):
    pass
