from abc import ABC, abstractmethod
from typing import Dict

from isar_turtlebot.models.turtlebot_status import Status


class TaskHandler(ABC):

    """Baseclass for task handlers"""

    @abstractmethod
    def start(self, task_input) -> None:
        pass

    @abstractmethod
    def get_status(self) -> Status:
        pass

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
