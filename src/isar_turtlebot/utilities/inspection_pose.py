import numpy as np
from robot_interface.models.geometry.frame import Frame
from robot_interface.models.geometry.orientation import Orientation
from robot_interface.models.geometry.pose import Pose


from scipy.spatial.transform import Rotation


def get_inspection_pose(current_pose, target):

    direction = np.array(target.to_list()) - np.array(current_pose.position.to_list())

    alpha = np.arctan2(direction[0], direction[1])

    rot = Rotation.from_euler("zyx", [alpha, 0, 0], degrees=False)
    rotation = rot.as_quat()

    orientation = Orientation(
        x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3], frame=Frame.Robot
    )

    pose = Pose(
        position=current_pose.position, orientation=orientation, frame=Frame.Robot
    )

    return pose
