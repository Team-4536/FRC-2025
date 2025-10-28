import copy
import math
from wpimath import units
from real import angleWrap
import navx
from navx import AHRS
import ntcore
import rev
import wpilib
from wpilib import SerialPort
from phoenix6.hardware import CANcoder
from timing import TimeData
from ntcore import NetworkTableInstance
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkClosedLoopController,
    ClosedLoopConfig,
    ClosedLoopSlot,
    LimitSwitchConfig,
)
from wpimath.kinematics import SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.units import (
    meters_per_second,
    radians,
    rotationsToRadians,
    degreesToRadians,
)


class RobotHALBuffer:
    def __init__(self) -> None:
        pass

    # angle expected in CCW rads
    

    def update(self) -> None:
        pass