import numpy as np

from scipy.spatial.transform import Rotation

from robot_interface.models.geometry.frame import Frame
from robot_interface.models.geometry.orientation import Orientation
from robot_interface.models.geometry.pose import Pose
from robot_interface.models.geometry.position import Position


def get_inspection_pose(current_pose: Pose, target: Position):

    direction = np.array(target.to_list()) - np.array(current_pose.position.to_list())
    alpha = np.arctan2(direction[0], direction[1])
    rotation = Rotation.from_euler("zyx", [alpha, 0, 0], degrees=False)
    quaternion = rotation.as_quat()

    orientation = Orientation(
        x=quaternion[0],
        y=quaternion[1],
        z=quaternion[2],
        w=quaternion[3],
        frame=Frame.Robot,
    )

    pose = Pose(
        position=current_pose.position, orientation=orientation, frame=Frame.Robot
    )
    return pose


def within_heading_tolerance(robot_pose, inspection_pose):
    error_tol = 0.3
    delta_heading = smallest_angle_difference(
        angle1=robot_pose.orientation.yaw(),
        angle2=inspection_pose.orientation.yaw(),
    )
    return np.abs(delta_heading) < error_tol


def smallest_angle_difference(angle1, angle2):
    if angle1 > angle2:
        return angle1 - angle2
    return angle2 - angle1
