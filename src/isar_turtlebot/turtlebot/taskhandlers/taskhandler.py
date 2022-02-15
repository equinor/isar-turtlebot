from abc import ABC, abstractmethod
from typing import Dict

from robot_interface.models.mission.task import Task

from isar_turtlebot.models.turtlebot_status import Status


class TaskHandler(ABC):

    """Baseclass for task handlers"""

    @abstractmethod
    def start(self, task: Task) -> None:
        pass

    @abstractmethod
    def get_status(self) -> Status:
        pass

    @staticmethod
    def goal_id_from_message(message: Dict) -> Status:
        try:
            return message["status_list"][0]["goal_id"]["id"]
        except (KeyError, IndexError):
            return None

    @staticmethod
    def status_from_message(message: Dict) -> Status:
        status_code = message["status_list"][0]["status"]
        if status_code == 1:
            return Status.Active
        elif status_code == 2 or status_code == 4:
            return Status.Failure
        elif status_code == 3:
            return Status.Succeeded
        else:
            return Status.Unexpected
