import math
from typing import Optional

from self_driving.domain.entities import DesiredTwist
from self_driving.domain.entities import Odometer

from self_driving.domain.entities import Target

# TODO: add config file
SPEED_MAX = 0.5
SPEED_TARGET_CLOSE = 0.1
TARGET_CLOSE = 1.0
TARGET_REACHED = 0.2

ROTATION_MAX_SPEED = 0.5


def execute(target: Target, odometer: Odometer) -> Optional[DesiredTwist]:
    dx = target.x - odometer.x
    dy = target.y - odometer.y
    desired_angle = math.atan2(dy, dx)

    angle_error = desired_angle - odometer.orientation_z
    if angle_error >= math.pi:
        angle_error = angle_error - (math.pi * 2)
    if angle_error <= -math.pi:
        angle_error = angle_error + (math.pi * 2)

    if angle_error >= 0:
        angle_error = min(angle_error, ROTATION_MAX_SPEED)
    else:
        angle_error = max(angle_error, -ROTATION_MAX_SPEED)

    distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    speed = min(distance, SPEED_MAX)

    if distance < TARGET_REACHED:
        return DesiredTwist(x=0.0, orientation_z=0.0)

    return DesiredTwist(x=speed, orientation_z=angle_error)

