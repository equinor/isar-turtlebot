from enum import Enum

from robot_interface.models.mission import TaskStatus


class Status(str, Enum):
    Active: str = "active"
    Failure: str = "failure"
    Succeeded: str = "success"
    Unexpected: str = "unexpected"

    @classmethod
    def map_to_task_status(cls, status: "Status") -> TaskStatus:
        return {
            Status.Active: TaskStatus.InProgress,
            Status.Failure: TaskStatus.Failed,
            Status.Succeeded: TaskStatus.Completed,
            Status.Unexpected: TaskStatus.Unexpected,
        }[status]

    @classmethod
    def map_to_turtlebot_status(cls, status_code: int) -> "Status":
        if status_code == 1:
            return Status.Active
        elif status_code == 2 or status_code == 4:
            return Status.Failure
        elif status_code == 3:
            return Status.Succeeded
        else:
            return Status.Unexpected
