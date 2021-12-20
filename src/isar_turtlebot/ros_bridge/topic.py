import logging
import time
from abc import ABC, abstractmethod
from logging import Logger
from typing import Any, Optional

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
    def get_image(self) -> bytes:
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
        throttle_rate: int = 0,
        queue_size: int = 100,
        queue_length: int = 0,
        log_callbacks: bool = False,
        get_image_timeout: float = config.getfloat("mission", "get_image_timeout"),
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

        self.image: Optional[bytes] = None
        self.take_image: bool = False
        self.get_image_timeout: float = get_image_timeout

        self.subscribe()

    def publish(self, message: Any) -> None:
        self.topic.publish(Message(message))

    def subscribe(self) -> None:
        self.topic.subscribe(self.on_image)

    def on_image(self, message: dict) -> None:
        if self.take_image:
            self.image = message["data"].encode("ascii")
        else:
            self.image = None

        if self.log_callbacks:
            self.logger.debug(f"Updated value for topic {self.name}")

    def get_image(self) -> Optional[str]:
        self.take_image = True
        start_time = time.time()
        while not self.image:
            time.sleep(0.1)
            execution_time: float = time.time() - start_time
            if execution_time > self.get_image_timeout:
                raise TimeoutError(f"Unable to read image data from topic")
        self.take_image = False
        return self.image
