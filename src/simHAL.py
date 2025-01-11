import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive
from timing import TimeData
from wpimath.geometry import Rotation2d, Translation2d
import wpilib

class RobotSimHAL:
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.startTime = wpilib.getTime()

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        if wpilib.getTime() - self.startTime > 1:
            buf.limitSwitchValue = True
        if wpilib.getTime() - self.startTime > 2:
            buf.limitSwitchValue = False
            self.startTime = wpilib.getTime()

             
            
          

    def resetGyroToAngle(self, ang: float) -> None:
        pass
