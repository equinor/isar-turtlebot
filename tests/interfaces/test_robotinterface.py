from robot_interface.test_robot_interface import interface_test

from isar_turtlebot import Robot


def test_robotinterface():
    interface_test(Robot())
