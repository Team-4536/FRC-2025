import copy
import math

import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
from timing import TimeData


class RobotHALBuffer:
    def __init__(self) -> None:
        self.limitSwitchValue = False
        self.blueWheelVoltage = 0
        self.topWheels = 0
        self.alsoTopWheels = 0
        self.bottomWheels = 0

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        self.blueWheelVoltage = 0
        self.topWheels = 0
        self.alsoTopWheels = 0
        self.bottomWheels = 0
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()
        self.limitSwitch = wpilib.DigitalInput(2)
        self.blueWheel = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)
        self.topWheels = rev.SparkMax(11, rev.SparkLowLevel.MotorType.kBrushless)
        self.alsoTopWheels = rev.SparkMax(12, rev.SparkLowLevel.MotorType.kBrushless)
        self.bottomWheels = rev.SparkMax(13, rev.SparkLowLevel.MotorType.kBrushless)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)
        self.topWheels.setVoltage(buf.topWheels)
        self.alsoTopWheels.setVoltage(buf.alsoTopWheels)
        self.bottomWheels.setVoltage(buf.bottomWheels)
        self.blueWheel.setVoltage(buf.blueWheelVoltage)
        buf.limitSwitchValue = self.limitSwitch.get()
