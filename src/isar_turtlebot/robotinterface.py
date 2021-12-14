import logging
import time
from datetime import datetime
from io import BytesIO
from logging import Logger
from typing import Optional, Sequence
from uuid import UUID

import PIL.Image as PILImage
import numpy as np
from robot_interface.models.geometry.frame import Frame
from robot_interface.models.geometry.orientation import Orientation
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.geometry.position import Position
from robot_interface.models.inspection.inspection import (
    Image,
    ImageMetadata,
    Inspection,
    ThermalImage,
    ThermalImageMetadata,
    TimeIndexedPose,
)
from robot_interface.models.mission import (
    DriveToPose,
    InspectionTask,
    TakeImage,
    TakeThermalImage,
    Task,
    TaskStatus,
)
from robot_interface.robot_interface import RobotInterface

from isar_turtlebot.config import config
from isar_turtlebot.models.turtlebot_status import TurtlebotStatus
from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.utilities.inspection_pose import get_inspection_pose


class Robot(RobotInterface):
    def __init__(self):
        self.logger: Logger = logging.getLogger("robot")

        self.bridge: RosBridge = RosBridge()

        self.inspection_task_timeout: float = config.getfloat(
            "mission", "inspection_task_timeout"
        )
        self.current_task: Optional[str] = None
        self.inspection_status: Optional[TurtlebotStatus] = None
        self.goal_id: str = ""

    def schedule_task(self, task: Task) -> bool:
        self.goal_id = self._get_turtlebot_goal_id()
        self._publish_task(task=task)
        return True

    def mission_scheduled(self) -> bool:
        return False

    def task_status(self, task_id: Optional[UUID]) -> TaskStatus:
        task_status: TaskStatus = TurtlebotStatus.get_task_status(
            status=self._task_status()
        )
        return task_status

    def abort_mission(self) -> bool:
        return True

    def log_status(self, task_status: TaskStatus, current_task: Task):
        self.logger.info(f"Task Status: {task_status}")
        self.logger.info(f"Current Task: {current_task}")

    def get_inspection_references(
        self, inspection_task: InspectionTask
    ) -> Sequence[Inspection]:
        now: datetime = datetime.utcnow()
        pose: Pose = self._get_robot_pose()

        if isinstance(inspection_task, TakeImage):
            image_metadata: ImageMetadata = ImageMetadata(
                start_time=now,
                time_indexed_pose=TimeIndexedPose(pose=pose, time=now),
                file_type=config.get("metadata", "image_filetype"),
            )
            image: Image = Image(metadata=image_metadata)

            self.bridge.visual_inspection.register_inspection_id(inspection_id=image.id)
            return [image]

        if isinstance(inspection_task, TakeThermalImage):
            image_metadata: ThermalImageMetadata = ThermalImageMetadata(
                start_time=now,
                time_indexed_pose=TimeIndexedPose(pose=pose, time=now),
                file_type=config.get("metadata", "thermal_image_filetype"),
            )
            thermal_image: ThermalImage = ThermalImage(metadata=image_metadata)

            self.bridge.visual_inspection.register_inspection_id(
                inspection_id=thermal_image.id
            )

            return [thermal_image]

        raise TypeError(
            f"Current task {inspection_task} is not a valid inspection task"
        )

    def download_inspection_result(self, inspection: Inspection) -> Inspection:
        if isinstance(inspection, Image):
            try:
                image_data = self.bridge.visual_inspection.read_image(
                    inspection_id=inspection.id
                )
                inspection.data = image_data
            except (KeyError, TypeError, FileNotFoundError) as e:
                self.logger.error("Failed to retrieve inspection result", e)
                raise FileNotFoundError("No inspection was found") from e

        elif isinstance(inspection, ThermalImage):
            try:
                image_data = self.bridge.visual_inspection.read_image(
                    inspection_id=inspection.id
                )
                image = PILImage.open(BytesIO(image_data))
                image_array = np.asarray(image)
                image_red = image_array[:, :, 0]
                image_grey = PILImage.fromarray(image_red)
                image_array_io = BytesIO()
                image_grey.save(image_array_io, format=inspection.metadata.file_type)

                inspection.data = image_array_io.getvalue()
            except Exception as e:
                self.logger.error("Failed to retrieve inspection result", e)
                raise FileNotFoundError("Failed to retrieve inspection result") from e
        return inspection

    def robot_pose(self) -> Pose:
        return self._get_robot_pose()

    def _get_robot_pose(self) -> Pose:

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

    def _task_status(self) -> TurtlebotStatus:
        if self.current_task == "inspection":
            return self.inspection_status
        return self._navigation_status()

    def _navigation_status(self) -> TurtlebotStatus:
        message: dict = self.bridge.task_status.get_value()
        turtle_status: TurtlebotStatus = TurtlebotStatus.map_to_turtlebot_status(
            message["status_list"][0]["status"]
        )
        return turtle_status

    def _publish_task(self, task: Task):
        if isinstance(task, DriveToPose):
            self.current_task = "navigation"
            self._publish_navigation_task(pose=task.pose)
        elif isinstance(task, (TakeImage, TakeThermalImage)):
            self.current_task = "inspection"
            self._publish_inspection_task(target=task.target)
            try:
                self._do_inspection_task()
            except TimeoutError as e:
                self.logger.error(e)
                self.current_task = None
        else:
            raise NotImplementedError(
                f"Scheduled task: {task} is not implemented on {self}"
            )

    def _do_inspection_task(self):
        start_time: float = time.time()
        while self._navigation_status() is not TurtlebotStatus.Succeeded:
            time.sleep(0.1)
            execution_time: float = time.time() - start_time
            if execution_time > self.inspection_task_timeout:
                self.inspection_status = TurtlebotStatus.Failure
                raise TimeoutError(
                    f"Drive to inspection pose task for TurtleBot3 timed out."
                )

        self.bridge.visual_inspection.take_image()
        while not self.bridge.visual_inspection.stored_image():
            time.sleep(0.1)
            execution_time: float = time.time() - start_time
            if execution_time > self.inspection_task_timeout:
                self.inspection_status = TurtlebotStatus.Failure
                raise TimeoutError(f"Storing image for TurtleBot3 timed out.")
        self.inspection_status = TurtlebotStatus.Succeeded
        self.current_task = None

    def _publish_inspection_task(self, target: Position) -> None:
        self.inspection_status = TurtlebotStatus.Active
        inspection_pose: Pose = get_inspection_pose(
            current_pose=self.robot_pose(), target=target
        )
        self._publish_navigation_task(pose=inspection_pose)

    def _publish_navigation_task(self, pose: Pose) -> None:
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
        start_time: float = time.time()

        while self._get_turtlebot_goal_id() == self.goal_id:
            time.sleep(0.1)
            execution_time: float = time.time() - start_time

            if execution_time > self.inspection_task_timeout:
                self.inspection_status = TurtlebotStatus.Failure
                raise TimeoutError(f"Storing image for TurtleBot3 timed out.")

    def _get_turtlebot_goal_id(self) -> Optional[str]:
        status_msg: dict = self.bridge.task_status.get_value()

        try:
            return status_msg["status_list"][0]["goal_id"]["id"]
        except (KeyError, IndexError):
            self.logger.info("Failed to get current turtlebot_goal_id, returning None")
            return None
