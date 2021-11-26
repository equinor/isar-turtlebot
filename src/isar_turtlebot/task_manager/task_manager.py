import logging
import time
from logging import Logger
from typing import Optional

import numpy as np
from isar_turtlebot.models.turtlebot_status import TurtlebotStatus
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.task_manager.tasks.capture_image import CaptureImage
from isar_turtlebot.task_manager.tasks.navigate import Navigate
from robot_interface.models.geometry.frame import Frame
from robot_interface.models.geometry.orientation import Orientation
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.geometry.position import Position
from robot_interface.models.mission import (
    DriveToPose,
    Step,
    TakeImage,
    TakeThermalImage,
)
from scipy.spatial.transform import Rotation


class TaskManager:
    def __init__(self, bridge: RosBridge) -> None:

        self.logger: Logger = logging.getLogger("robot")
        self.bridge: RosBridge = bridge

        self.current_task: Optional[str] = None
        self.inspection_status: Optional[TurtlebotStatus] = None

        self.current_filename: Optional[str] = None
        self.filenames: dict = dict()

        self.previous_run_id: Optional[str] = None

    def publish_task(self, step: Step) -> str:
        self.previous_run_id = self.get_run_id()
        if isinstance(step, DriveToPose):
            self.current_task = "navigation"
            run_id: str = self.publish_navigation_task(pose=step.pose)

        elif isinstance(step, (TakeImage, TakeThermalImage)):
            self.current_task = "inspection"
            run_id: str = self.publish_inspection_task(target=step.target)
        else:
            raise NotImplementedError(
                f"Scheduled step: {step} is not implemented on {self}"
            )
        self.current_task = None
        return run_id

    def perform_navigation_task(self, pose: Pose):
        navigation_task = Navigate(self.bridge)
        navigation_task.run(pose=pose)
        run_id = self.wait_for_updated_task(previous_run_id=self.previous_run_id)
        return run_id

    def perform_inspection_task(self):
        image_task = CaptureImage(self.bridge)
        self.current_filename = image_task.run()

    def publish_inspection_task(self, target: Position) -> None:
        self.inspection_status = TurtlebotStatus.Active
        inspection_pose: Pose = self.get_inspection_pose(
            current_pose=self.get_robot_pose(), target=target
        )
        run_id: str = self.perform_navigation_task(pose=inspection_pose)

        try:
            self.perform_inspection_task()
        except TimeoutError as e:
            self.logger.error(e)
            self.current_task = None

        self.inspection_status = TurtlebotStatus.Succeeded
        return run_id

    def publish_navigation_task(self, pose: Pose) -> None:
        run_id = self.perform_navigation_task(pose=pose)
        return run_id

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

    def get_run_id(self) -> Optional[str]:
        status_msg: dict = self.bridge.mission_status.get_value()
        try:
            run_id: str = status_msg["status_list"][0]["goal_id"]["id"]
            return run_id
        except (KeyError, IndexError):
            self.logger.info("Failed to get current mission_id returning None")
            return None

    def navigation_status(self) -> TurtlebotStatus:
        message: dict = self.bridge.mission_status.get_value()
        turtle_status: TurtlebotStatus = TurtlebotStatus.map_to_turtlebot_status(
            message["status_list"][0]["status"]
        )
        return turtle_status

    def task_status(self) -> TurtlebotStatus:
        if self.current_task == "inspection":
            return self.inspection_status
        return self.navigation_status()

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

    def get_inspection_pose(self, current_pose: Pose, target: Position) -> Pose:
        direction = np.array(target.to_list()) - np.array(
            current_pose.position.to_list()
        )
        alpha = np.arctan2(direction[1], direction[0])
        rotation = Rotation.from_euler("zyx", [alpha, 0, 0], degrees=False)
        quaternion = rotation.as_quat()

        orientation = Orientation(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
            frame=Frame.Robot,
        )

        pose = Pose(
            position=current_pose.position, orientation=orientation, frame=Frame.Robot
        )
        return pose
