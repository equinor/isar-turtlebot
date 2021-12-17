import pytest

from tests.mocks.ros_bridge import MockRosBridge


@pytest.fixture()
def bridge():
    return MockRosBridge()
