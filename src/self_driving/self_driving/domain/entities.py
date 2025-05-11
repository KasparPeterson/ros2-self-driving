from dataclasses import dataclass


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