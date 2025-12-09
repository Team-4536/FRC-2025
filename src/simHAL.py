import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive

from wpimath.geometry import Rotation2d, Translation2d


class RobotSimHAL:
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        

    

    def resetGyroToAngle(self, ang: float) -> None:
        pass
