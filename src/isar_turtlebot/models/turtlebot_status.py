from enum import Enum

from robot_interface.models.mission.step import StepStatus


class Status(str, Enum):
    Active: str = "active"
    Failure: str = "failure"
    Succeeded: str = "success"
    Unexpected: str = "unexpected"

    @classmethod
    def map_to_step_status(cls, status: "Status") -> StepStatus:
        return {
            Status.Active: StepStatus.InProgress,
            Status.Failure: StepStatus.Failed,
            Status.Succeeded: StepStatus.Successful,
            Status.Unexpected: StepStatus.Failed,
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
