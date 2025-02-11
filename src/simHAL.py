import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive
from timing import TimeData
from wpimath.geometry import Rotation2d, Translation2d


class RobotSimHAL:
    def __init__(self):
        self.prev = RobotHALBuffer()

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        buf.manipulatorSensorForward = buf.simForwardSensorValue
        buf.manipulatorSensorReverse = buf.simReverseSensorValue


    def resetGyroToAngle(self, ang: float) -> None:
        pass
