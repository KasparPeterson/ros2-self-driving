from self_driving.domain import twist_calculator
from self_driving.domain.entities import DesiredTwist
from self_driving.domain.entities import Odometer
from self_driving.domain.entities import Target


def setup_function():
    twist_calculator.ROTATION_SPEED = 0.5


def test_x_0_y_0():
    result = twist_calculator.execute(
        target=Target(x=0.0, y=0.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
    )
    assert result == DesiredTwist(x=0.0, orientation_z=0.0)


def test_x_5_y_0():
    result = twist_calculator.execute(
        target=Target(x=5.0, y=0.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=0.0)


def test_x_5_y_5():
    result = twist_calculator.execute(
        target=Target(x=5.0, y=5.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=0.5)


def test_x_0_y_5():
    result = twist_calculator.execute(
        target=Target(x=0.0, y=5.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=0.5)


def test_x_0_y_5_z_override():
    result = twist_calculator.execute(
        target=Target(x=0.0, y=5.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=2.0),
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=-0.42920367320510344)


def test_x_0_y_minus5():
    result = twist_calculator.execute(
        target=Target(x=0.0, y=-5.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=-0.5)


def test_x_minus5_y_0():
    result = twist_calculator.execute(
        target=Target(x=-5.0, y=0.0),
        odometer=Odometer(x=0.0, y=0.0, orientation_z=0.0),
    )
    assert result == DesiredTwist(x=twist_calculator.SPEED_MAX, orientation_z=0.5)
