from robot_interface.test_robot_interface import interface_test

from isar_turtlebot import Robot
from tests.mocks.ros_bridge import MockRosBridge


def test_robotinterface():
    robot = Robot(bridge=MockRosBridge)
    interface_test(robot)
