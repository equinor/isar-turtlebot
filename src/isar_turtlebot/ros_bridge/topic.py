import base64
import logging
import time
from abc import ABC, abstractmethod
from logging import Logger
from pathlib import Path
from typing import Any, Optional
from uuid import uuid4

from isar_turtlebot.config import config
from roslibpy import Message, Ros
from roslibpy import Topic as RosTopic


class TopicInterface(ABC):
    @abstractmethod
    def publish(self, message: Any) -> None:
        pass

    @abstractmethod
    def get_value(self) -> Optional[Any]:
        pass


class ImageTopicInterface(ABC):
    @abstractmethod
    def publish(self, message: Any) -> None:
        pass

    @abstractmethod
    def get_image(self) -> Optional[Any]:
        pass


class Topic(TopicInterface):
    def __init__(
        self,
        client: Ros,
        name: str,
        message_type: str,
        throttle_rate: int = 0,
        queue_size: int = 100,
        queue_length: int = 0,
        log_callbacks: bool = False,
    ) -> None:
        self.name: str = name
        self.topic: RosTopic = RosTopic(
            ros=client,
            name=name,
            message_type=message_type,
            throttle_rate=throttle_rate,
            queue_size=queue_size,
            queue_length=queue_length,
        )

        self.log_callbacks: bool = log_callbacks
        if self.log_callbacks:
            self.logger: Logger = logging.getLogger("turtlebot_bridge")

        self.value: Optional[Any] = None

        self.subscribe()

    def publish(self, message: Any) -> None:
        self.topic.publish(Message(message))

    def get_value(self) -> Optional[Any]:
        return self.value

    def on_message(self, message: dict) -> None:
        self.value = message
        if self.log_callbacks:
            self.logger.debug(f"Updated value for topic {self.name}")

    def subscribe(self) -> None:
        self.topic.subscribe(self.on_message)


class ImageTopic(ImageTopicInterface):
    def __init__(
        self,
        client: Ros,
        name: str,
        message_type: str,
        throttle_rate: int = 1000,
        queue_size: int = 100,
        queue_length: int = 0,
        log_callbacks: bool = False,
    ) -> None:
        self.name: str = name
        self.topic: RosTopic = RosTopic(
            ros=client,
            name=name,
            message_type=message_type,
            throttle_rate=throttle_rate,
            queue_size=queue_size,
            queue_length=queue_length,
        )

        self.log_callbacks: bool = log_callbacks
        if self.log_callbacks:
            self.logger: Logger = logging.getLogger("turtlebot_bridge")

        self.image: Optional[Any] = None

    def publish(self, message: Any) -> None:
        self.topic.publish(Message(message))

    def subscribe(self) -> None:
        self.topic.subscribe(self.on_image)

    def unsubscribe(self) -> None:
        self.image = None
        self.topic.unsubscribe()

    def on_image(self, message: dict) -> None:
        self.image = message["data"].encode("ascii")
        if self.log_callbacks:
            self.logger.debug(f"Updated value for topic {self.name}")

    def get_image(self) -> Optional[str]:
        return self.image
