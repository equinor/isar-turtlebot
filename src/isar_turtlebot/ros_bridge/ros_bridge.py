import logging
from abc import ABC
from logging import Logger

from isar_turtlebot.ros_bridge.topic import ImageTopic, Topic
from isar_turtlebot.settings import settings
from roslibpy import Ros


class RosBridgeInterface(ABC):
    pass


class RosBridge(RosBridgeInterface):
    def __init__(
        self,
        host: str = settings.ROS_BRIDGE_HOST,
        port: int = settings.ROS_BRIDGE_PORT,
    ):
        self.logger: Logger = logging.getLogger("state_machine")

        self.client: Ros = self.connect_client(host=host, port=port)

        self.execute_task: Topic = Topic(
            client=self.client,
            name="/move_base/goal",
            message_type="move_base_msgs/MoveBaseActionGoal",
            throttle_rate=1000,
        )

        self.task_status: Topic = Topic(
            client=self.client,
            name="/move_base/status",
            message_type="actionlib_msgs/GoalStatusArray",
        )

        self.pose: Topic = Topic(
            client=self.client,
            name="/amcl_pose",
            message_type="geometry_msgs/PoseWithCovarianceStamped",
        )

        self.visual_inspection: ImageTopic = ImageTopic(
            client=self.client,
            name="/camera/rgb/image_raw/compressed",
            message_type="sensor_msgs/CompressedImage",
        )

    def connect_client(
        self,
        host: str,
        port: int,
        connection_retries: int = 3,
        connection_timeout: int = 10,
    ) -> Ros:
        client: Ros = Ros(host=host, port=port)

        retries: int = connection_retries
        while retries:
            retries -= 1
            try:
                client.run(timeout=connection_timeout)
                if client.is_connected:
                    self.logger.info(f"Successfully connected to ROS at {host}:{port}.")
                    break
            except Exception as e:
                msg: str = f"RosBridge failed to connect to ROS at {host}:{port} with message: {e}"
                self.logger.warning(
                    f"{msg} - Attempt {connection_retries - retries} of {connection_retries}"
                )

                if not retries:
                    self.logger.error(msg)
                    raise ConnectionError(msg)

        client.run()

        return client
