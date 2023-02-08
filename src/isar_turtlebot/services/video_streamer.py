import base64

from isar_turtlebot.ros_bridge.ros_bridge import RosBridge


class VideoStreamer:
    def __init__(self, bridge: RosBridge) -> None:
        self.bridge: RosBridge = bridge

    def main(self):
        while True:
            image_data: str = self.bridge.video_stream.get_image()
            image_bytes: bytes = base64.b64decode(image_data)
            print("Streaming video")

        return
