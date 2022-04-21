import numpy as np
from alitra import Frame, Orientation, Pose, Position
from scipy.spatial.transform import Rotation


def get_inspection_pose(current_pose: Pose, target: Position) -> Pose:

    direction = np.array(target.to_list()) - np.array(current_pose.position.to_list())
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
