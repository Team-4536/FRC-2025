import copy
import math

import navx
import ntcore
import rev
import wpilib
from timing import TimeData


class RobotHALBuffer:
    def __init__(self) -> None:
        
        self.topmotorvolts = 0
        self.bottommotorvolts = 0
        self.laservalue = False
        
        pass


    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        self.topmotorvolts = 0
        self.bottommotorvolts = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()
        self.topmotor = rev.SparkMax(11,rev.SparkMax.MotorType.kBrushless)
        self.bottommotor = rev.SparkMax(12,rev.SparkMax.MotorType.kBrushless)
        self.laser = wpilib.DigitalInput(2)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        buf.laservalue = self.laser.get()
        self.prev = copy.deepcopy(buf)
        self.topmotor.setVoltage(buf.topmotorvolts)
        self.bottommotor.setVoltage(buf.bottommotorvolts)
        
        
