import math
from typing import Optional

from self_driving.domain.entities import DesiredTwist
from self_driving.domain.entities import Odometer

from self_driving.domain.entities import Target

# TODO: add config file
SPEED_MAX = 0.5
SPEED_TARGET_CLOSE = 0.1
TARGET_CLOSE = 1.0
TARGET_REACHED = 0.5

ROTATION_SMOOTHING = 0.2


def execute(target: Target, odometer: Odometer) -> Optional[DesiredTwist]:
    dx = target.x - odometer.x
    dy = target.y - odometer.y

    desired_angle = math.atan2(dy, dx)
    angle = desired_angle - odometer.orientation_z
    angle = angle * ROTATION_SMOOTHING

    target_speed = SPEED_MAX
    if dx < TARGET_REACHED and dy < TARGET_REACHED:
        target_speed = 0.0
        angle = 0.0
    elif dx < TARGET_CLOSE and dy < TARGET_CLOSE:
        target_speed = SPEED_TARGET_CLOSE

    return DesiredTwist(x=target_speed, orientation_z=angle)
