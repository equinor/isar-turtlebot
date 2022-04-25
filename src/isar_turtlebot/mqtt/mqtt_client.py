import logging
import os
from logging import Logger
from time import sleep
from typing import Dict

from alitra import Frame, Pose, Position
from paho.mqtt.client import Client

from isar_turtlebot.ros_bridge.ros_bridge import RosBridge
from isar_turtlebot.settings import settings
from isar_turtlebot.utilities.inspection_pose import get_distance
from isar_turtlebot.utilities.pose_message import decode_pose_message


class MqttClient:
    def __init__(
        self,
        bridge: RosBridge,
        host: str = settings.MQTT_HOST,
        port: int = settings.MQTT_PORT,
    ) -> None:
        self.logger: Logger = logging.getLogger("robot")

        username: str = settings.MQTT_USERNAME
        password: str = "default"  # ""

        try:
            password = os.environ["ISAR_TURTLEBOT_MQTT_PASSWORD"]
        except KeyError:
            pass

        self.distance_to_goal_topic = "distance_to_goal"

        self.bridge = bridge
        self.client = Client()

        self.client.username_pw_set(username=username, password=password)
        self.client.connect(host=host, port=port, keepalive=60)

    def publish_battery_state(self) -> None:
        pass

    def publish_distance_to_goal(self) -> None:
        published_task: Dict = self.bridge.execute_step.get_value()

        try:
            goal_pose_message: dict = published_task["goal"]["target_pose"]["pose"]

            goal_position: Position = Position(
                x=goal_pose_message["position"]["x"],
                y=goal_pose_message["position"]["y"],
                z=goal_pose_message["position"]["z"],
                frame=Frame("robot"),
            )
        except (IndexError, TypeError):
            return None

        pose_message: dict = self.bridge.pose.get_value()
        pose: Pose = decode_pose_message(pose_message=pose_message)

        distance = get_distance(current_position=pose.position, target=goal_position)

        return distance

    def publish(self) -> None:
        sleep(2)
        while True:
            distance = self.publish_distance_to_goal()
            payload = f"Distance to goal: {distance}"
            self.client.publish(topic=self.distance_to_goal_topic, payload=payload)
            self.logger.info(f"Published through MQTT client:\n{payload}")
            sleep(2.5)
            self.client.loop()
