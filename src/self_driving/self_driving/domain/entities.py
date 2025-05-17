from dataclasses import dataclass
from typing import List


@dataclass
class Target:
    x: float
    y: float


@dataclass
class DesiredTwist:
    x: float
    orientation_z: float


@dataclass
class Odometer:
    x: float
    y: float
    orientation_z: float


@dataclass
class Lidar:
    angle_min: float
    angle_max: float
    ranges: List[float]


@dataclass
class LidarSectors:
    """
    Each sector has 60 degrees. However when the lidar doesn' do the whole 360 then the back_left and back_right might
    be less than 60 degrees. This can also apply for the left and right.
    """
    front_left: float = 0.0
    front_right: float = 0.0
    left: float = 0.0
    right: float = 0.0
    back_left: float = 0.0
    back_right: float = 0.0
