import logging
from logging import Logger
from typing import Sequence

from robot_interface.models.inspection.inspection import Inspection
from robot_interface.models.mission import InspectionTask, Task, TaskStatus
from robot_interface.robot_interface import RobotInterface

from isar_turtlebot.ros_bridge.ros_bridge import RosBridge, RosBridgeInterface
from isar_turtlebot.turtlebot import Turtlebot


class Robot(RobotInterface):
    def __init__(self, bridge: RosBridgeInterface = RosBridge):
        self.turtlebot: Turtlebot = Turtlebot(bridge=bridge())

    def initiate_task(self, task: Task) -> bool:
        self.turtlebot.publish_task(task=task)
        return True

    def task_status(self) -> TaskStatus:
        return self.turtlebot.get_task_status()

    def stop(self) -> bool:
        return True

    def get_inspections(self, task: InspectionTask) -> Sequence[Inspection]:
        return self.turtlebot.get_inspections(task.id)
