import os
from pathlib import Path
from typing import Sequence

from alitra import MapAlignment, Pose, Transform, align_maps
from robot_interface.models.inspection.inspection import Inspection
from robot_interface.models.mission import InspectionStep, Step, StepStatus
from robot_interface.robot_interface import RobotInterface

from isar_turtlebot.ros_bridge.ros_bridge import RosBridge, RosBridgeInterface
from isar_turtlebot.turtlebot import Turtlebot


class Robot(RobotInterface):
    def __init__(self, bridge: RosBridgeInterface = RosBridge):
        map_alignment: MapAlignment = MapAlignment.from_config(
            Path(
                os.path.dirname(os.path.realpath(__file__)),
                "settings/maps/turtleworld.json",
            )
        )
        transform: Transform = align_maps(
            map_alignment.map_from, map_alignment.map_to, rot_axes="z"
        )
        self.turtlebot: Turtlebot = Turtlebot(bridge=bridge(), transform=transform)

    def initiate_step(self, step: Step) -> bool:
        self.turtlebot.publish_step(step=step)
        return True

    def step_status(self) -> StepStatus:
        return self.turtlebot.get_step_status()

    def stop(self) -> bool:
        return True

    def get_inspections(self, step: InspectionStep) -> Sequence[Inspection]:
        return self.turtlebot.get_inspections(step.id)

    def get_pose(self) -> Pose:
        return self.turtlebot.get_pose()
