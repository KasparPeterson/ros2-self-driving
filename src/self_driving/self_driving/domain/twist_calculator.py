import math
from typing import Optional

from self_driving.domain.entities import DesiredTwist
from self_driving.domain.entities import Odometer
from self_driving.domain.entities import Lidar
from self_driving.domain.entities import LidarSectors
from self_driving.domain.entities import Target

# TODO: add config file
SPEED_MAX = 0.5
SPEED_TARGET_CLOSE = 0.1
TARGET_CLOSE = 1.0
TARGET_REACHED = 0.2

ROTATION_MAX_SPEED = 0.5


def execute(target: Target, odometer: Odometer, lidar: Lidar) -> Optional[DesiredTwist]:
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


def _get_sectors(lidar: Lidar) -> LidarSectors:
    start_degree = _rad_to_deg(lidar.angle_min)
    end_degree = _rad_to_deg(lidar.angle_max)
    center = len(lidar.ranges) // 2
    degree_increment = (end_degree - start_degree) / len(lidar.ranges)

    deg_60 = int(60 / degree_increment)
    left_start = center - 2 * deg_60
    if left_start < 0:
        left_start = 0

    right_end = center + 2 * deg_60
    if right_end > len(lidar.ranges):
        right_end = len(lidar.ranges)

    return LidarSectors(
        front_right=min(lidar.ranges[center:center + deg_60]),
        front_left=min(lidar.ranges[center - deg_60:center]),
        left=min(lidar.ranges[left_start:center - deg_60]),
        right=min(lidar.ranges[center + deg_60:right_end]),
        back_right=0,
        back_left=0,
    )


def _rad_to_deg(rad):
    return rad * 180 / math.pi
