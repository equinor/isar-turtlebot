from robot_interface.models.geometry.frame import Frame
from robot_interface.models.geometry.orientation import Orientation
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.geometry.position import Position


def encode_pose_message(pose: Pose) -> dict:
    return {
        "goal": {
            "target_pose": {
                "header": {
                    "seq": 0,
                    "stamp": {"secs": 1533, "nsecs": 746000000},
                    "frame_id": "map",
                },
                "pose": {
                    "position": {
                        "x": pose.position.x,
                        "y": pose.position.y,
                        "z": pose.position.z,
                    },
                    "orientation": {
                        "x": pose.orientation.x,
                        "y": pose.orientation.y,
                        "z": pose.orientation.z,
                        "w": pose.orientation.w,
                    },
                },
            }
        },
    }


def decode_pose_message(pose_message: dict, frame: Frame = Frame.Robot) -> Pose:
    position_message: Dict = pose_message["pose"]["pose"]["position"]
    orientation_message: Dict = pose_message["pose"]["pose"]["orientation"]

    return Pose(
        position=Position(
            x=position_message["x"],
            y=position_message["y"],
            z=position_message["z"],
            frame=frame,
        ),
        orientation=Orientation(
            x=orientation_message["x"],
            y=orientation_message["y"],
            z=orientation_message["z"],
            w=orientation_message["w"],
            frame=frame,
        ),
        frame=frame,
    )
