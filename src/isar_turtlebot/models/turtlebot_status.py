from enum import Enum

from robot_interface.models.mission import MissionStatus


class TurtlebotStatus(str, Enum):
    Active: str = "active"
    Failure: str = "failure"
    Succeeded: str = "success"
    Unexpected: str = "unexpected"

    @classmethod
    def get_mission_status(cls, status: "TurtlebotStatus") -> MissionStatus:
        return {
            TurtlebotStatus.Active: MissionStatus.InProgress,
            TurtlebotStatus.Failure: MissionStatus.Failed,
            TurtlebotStatus.Succeeded: MissionStatus.Completed,
            TurtlebotStatus.Unexpected: MissionStatus.Unexpected,
        }[status]

    @classmethod
    def map_to_turtlebot_status(cls, status_code: int) -> "TurtlebotStatus":
        if status_code == 1:
            return TurtlebotStatus.Active
        elif status_code == 2:
            return TurtlebotStatus.Failure
        elif status_code == 3:
            return TurtlebotStatus.Succeeded
        else:
            return TurtlebotStatus.Unexpected
