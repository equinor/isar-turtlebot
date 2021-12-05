from enum import Enum

from robot_interface.models.mission import TaskStatus


class TurtlebotStatus(str, Enum):
    Active: str = "active"
    Failure: str = "failure"
    Succeeded: str = "success"
    Unexpected: str = "unexpected"

    @classmethod
    def get_task_status(cls, status: "TurtlebotStatus") -> TaskStatus:
        return {
            TurtlebotStatus.Active: TaskStatus.InProgress,
            TurtlebotStatus.Failure: TaskStatus.Failed,
            TurtlebotStatus.Succeeded: TaskStatus.Completed,
            TurtlebotStatus.Unexpected: TaskStatus.Unexpected,
        }[status]

    @classmethod
    def map_to_turtlebot_status(cls, status_code: int) -> "TurtlebotStatus":
        if status_code == 1:
            return TurtlebotStatus.Active
        elif status_code == 2 or status_code == 4:
            return TurtlebotStatus.Failure
        elif status_code == 3:
            return TurtlebotStatus.Succeeded
        else:
            return TurtlebotStatus.Unexpected
