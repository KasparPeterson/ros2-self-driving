from math import inf

from self_driving.domain import twist_calculator
from self_driving.domain.entities import DesiredTwist
from self_driving.domain.entities import Odometer
from self_driving.domain.entities import Target
from self_driving.domain.entities import Lidar

from self_driving.domain.entities import LidarSectors

EMPTY_LIDAR = Lidar(angle_min=0, angle_max=0, ranges=[])


def setup_function():
    twist_calculator.ROTATION_SPEED = 0.5


def test_x_0_y_0():
    result = twist_calculator.execute(
        target=Target(x=0.0, y=0.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
        lidar=EMPTY_LIDAR,
    )
    assert result == DesiredTwist(x=0.0, orientation_z=0.0)


def test_x_5_y_0():
    result = twist_calculator.execute(
        target=Target(x=5.0, y=0.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
        lidar=EMPTY_LIDAR,
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=0.0)


def test_x_5_y_5():
    result = twist_calculator.execute(
        target=Target(x=5.0, y=5.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
        lidar=EMPTY_LIDAR,
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=0.5)


def test_x_0_y_5():
    result = twist_calculator.execute(
        target=Target(x=0.0, y=5.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
        lidar=EMPTY_LIDAR,
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=0.5)


def test_x_0_y_5_z_override():
    result = twist_calculator.execute(
        target=Target(x=0.0, y=5.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=2.0),
        lidar=EMPTY_LIDAR,
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=-0.42920367320510344)


def test_x_0_y_minus5():
    result = twist_calculator.execute(
        target=Target(x=0.0, y=-5.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
        lidar=EMPTY_LIDAR,
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=-0.5)


def test_x_minus5_y_0():
    result = twist_calculator.execute(
        target=Target(x=-5.0, y=0.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
        lidar=EMPTY_LIDAR,
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=0.5)


def test_get_sectors():
    ranges = [inf] * 640
    ranges[319] = 1.1
    ranges[320] = 1.2
    ranges[70] = 3.1
    ranges[570] = 3.2
    lidar = Lidar(angle_min=-1.392, angle_max=1.392, ranges=ranges)
    result = twist_calculator._get_sectors(lidar)
    assert result == LidarSectors(
        front_left=1.1,
        front_right=1.2,
        left=3.1,
        right=3.2,
        back_right=0,  # TODO
        back_left=0,  # TODO
    )
