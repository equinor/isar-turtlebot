import logging
import time
from logging import Logger
from typing import Any, Optional, Sequence, Tuple


from robot_interface.models.geometry.frame import Frame
from robot_interface.models.geometry.joints import Joints
from robot_interface.models.geometry.orientation import Orientation
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.geometry.position import Position
from robot_interface.models.inspection.inspection import (
    Inspection,
    InspectionResult,
)
from robot_interface.models.mission import (
    DriveToPose,
    TakeImage,
    TakeThermalImage,
    MissionStatus,
    Step,
)
from robot_interface.robot_interface import RobotInterface

from isar_turtlebot.models.turtlebot_status import TurtlebotStatus
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge


class Robot(RobotInterface):
    def __init__(self):
        self.logger: Logger = logging.getLogger("robot")

        self.bridge: RosBridge = RosBridge()

    def schedule_step(self, step: Step) -> Tuple[bool, Optional[Any], Optional[Joints]]:

        previous_run_id: str = self.get_run_id()
        self.publish_task(step=step)
        run_id: str = self.wait_for_updated_task(previous_run_id=previous_run_id)
        return True, run_id, None

    def publish_task(self, step: Step) -> None:
        if isinstance(step, DriveToPose):
            self.publish_navigation_task(pose=step.pose)

        elif isinstance(step, (TakeImage, TakeThermalImage)):
            raise TypeError(f"Image step is not  yet implemented on turtlebot")

    def publish_navigation_task(self, pose: Pose) -> None:
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

        self.bridge.execute_task.publish(message=pose_message)

    def get_run_id(self) -> str:
        status_msg: dict = self.bridge.mission_status.get_value()
        try:
            run_id: str = status_msg["status_list"][0]["goal_id"]["id"]
            return run_id
        except KeyError as e:
            self.logger.error(e)
            raise

    def wait_for_updated_task(self, previous_run_id: str, timeout: int = 20) -> str:
        start_time: float = time.time()
        current_run_id: str = self.get_run_id()

        while current_run_id == previous_run_id:
            time.sleep(0.1)
            execution_time: float = time.time() - start_time
            if execution_time > timeout:
                raise TimeoutError(
                    f"Scheduling of task for TurtleBot3 timed out. Run ID: {current_run_id}"
                )
            current_run_id = self.get_run_id()

        return current_run_id

    def mission_scheduled(self) -> bool:
        return False

    def mission_status(self, mission_id: Any) -> MissionStatus:
        message: dict = self.bridge.mission_status.get_value()
        turtle_status: TurtlebotStatus = TurtlebotStatus.map_to_turtlebot_status(
            message["status_list"][0]["status"]
        )
        mission_status: MissionStatus = TurtlebotStatus.get_mission_status(
            status=turtle_status
        )
        return mission_status

    def abort_mission(self) -> bool:
        return True

    def log_status(
        self, mission_id: Any, mission_status: MissionStatus, current_step: Step
    ):
        self.logger.info(f"Mission ID: {mission_id}")
        self.logger.info(f"Mission Status: {mission_status}")
        self.logger.info(f"Current Step: {current_step}")

    def get_inspection_references(
        self, vendor_mission_id: Any, current_step: Step
    ) -> Sequence[Inspection]:
        return []

    def download_inspection_result(
        self, inspection: Inspection
    ) -> Optional[InspectionResult]:
        return None

    def get_robot_pose(self) -> Pose:

        pose_message: dict = self.bridge.pose.get_value()
        position_message: dict = pose_message["pose"]["pose"]["position"]
        orientation_message: dict = pose_message["pose"]["pose"]["orientation"]

        pose: Pose = Pose(
            position=Position(
                x=position_message["x"],
                y=position_message["y"],
                z=position_message["z"],
                frame=Frame.Robot,
            ),
            orientation=Orientation(
                x=orientation_message["x"],
                y=orientation_message["y"],
                z=orientation_message["z"],
                w=orientation_message["w"],
                frame=Frame.Robot,
            ),
            frame=Frame.Robot,
        )
        return pose

    def robot_pose(self) -> Pose:
        return self.get_robot_pose()
