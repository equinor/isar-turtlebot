import numpy as np
from alitra import Frame, Orientation, Pose, Position
from scipy.spatial.transform import Rotation


def get_inspection_pose(current_pose: Pose, target: Position) -> Pose:

    direction = target.to_array() - current_pose.position.to_array()
    alpha = np.arctan2(direction[1], direction[0])
    rotation = Rotation.from_euler("zyx", [alpha, 0, 0], degrees=False)
    quaternion = rotation.as_quat()

    orientation = Orientation(
        x=quaternion[0],
        y=quaternion[1],
        z=quaternion[2],
        w=quaternion[3],
        frame=Frame("robot"),
    )

    pose = Pose(
        position=current_pose.position, orientation=orientation, frame=Frame("robot")
    )
    return pose


def get_distance(current_position: Position, target: Position) -> float:
    current_position = np.array(current_position.to_array())
    target = np.array(target.to_array())

    distance = np.sqrt(
        (current_position[0] - target[0]) ** 2
        + (current_position[1] - target[1]) ** 2
        + (current_position[2] - target[2]) ** 2
    )

    return distance
