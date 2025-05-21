import math
from typing import Optional

from self_driving.domain.entities import DesiredTwist
from self_driving.domain.entities import Odometer
from self_driving.domain.entities import Lidar
from self_driving.domain.entities import LidarSectors
from self_driving.domain.entities import Target

# TODO: add config file
SPEED_MAX = 0.5
SPEED_SLOW = 0.2
TARGET_CLOSE = 1.0
TARGET_REACHED = 0.2

ROTATION_MAX_SPEED = 0.3

LIDAR_TOO_CLOSE = 2.0


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

    # Got desired speed and distance but we need to avoid collision
    sectors = _get_sectors(lidar)
    if sectors.front_left < LIDAR_TOO_CLOSE or sectors.front_right < LIDAR_TOO_CLOSE:
        if sectors.front_left < sectors.front_right:
            angle_error = -ROTATION_MAX_SPEED
        else:
            angle_error = ROTATION_MAX_SPEED
        speed = SPEED_SLOW
        # print("FRONT TOO CLOSE!, angle_error:", angle_error)
        # print(f"  front_left: {sectors.front_left}, front_right: {sectors.front_right}")
    elif sectors.left < LIDAR_TOO_CLOSE:
        angle_error = 0.0
        speed = SPEED_SLOW
        # print("LEFT SIDE TOO CLOSE!, angle_error:", angle_error)
    elif sectors.right < LIDAR_TOO_CLOSE:
        angle_error = 0.0
        speed = SPEED_SLOW
        # print("RIGHT SIDE TOO CLOSE!, angle_error:", angle_error)
    # print("\n")

    if distance < TARGET_REACHED:
        return DesiredTwist(x=0.0, orientation_z=0.0)

    return DesiredTwist(x=speed, orientation_z=angle_error)


def _get_sectors(lidar: Lidar) -> LidarSectors:
    if lidar.ranges is None or len(lidar.ranges) == 0:
        return LidarSectors(
            front_left=math.inf,
            front_right=math.inf,
            left=math.inf,
            right=math.inf,
        )

    start_degree = _rad_to_deg(lidar.angle_min)
    end_degree = _rad_to_deg(lidar.angle_max)
    center = len(lidar.ranges) // 2
    degree_increment = (end_degree - start_degree) / len(lidar.ranges)

    deg_60 = int(60 / degree_increment)
    right_start = center - 2 * deg_60
    if right_start < 0:
        right_start = 0

    left_end = center + 2 * deg_60
    if left_end > len(lidar.ranges):
        left_end = len(lidar.ranges)

    return LidarSectors(
        front_left=min(lidar.ranges[center:center + deg_60]),
        front_right=min(lidar.ranges[center - deg_60:center]),
        left=min(lidar.ranges[center + deg_60: left_end]),
        right=min(lidar.ranges[right_start:center - deg_60]),
        back_right=0,
        back_left=0,
    )


def _rad_to_deg(rad):
    return rad * 180 / math.pi
