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
        motorBLdrv_Spd = 0.0
        pass

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()
        self.myMotor = rev.SparkMax(1, rev.MotorType.kBrushless)
        self.myPID = self.myMotor.getClosedLoopController()
        self.myConfig = rev.SparkMaxConfig
        self.myConfig.closedLoop.pid(0.1,0,0)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        self.myMotor.setVoltage(1.0)
        self.myMotor.set(0.1)

        prev = self.prev
        self.prev = copy.deepcopy(buf)
