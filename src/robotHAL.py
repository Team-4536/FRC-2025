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
        self.myMotor = rev.CANSparkMax(1, rev.MotorType.kBrushless)

        self.myMotor.setP(0.1)
        self.myMotor.setI(0.0)   
        self.myMotor.setD(0.0)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        self.myMotor.setReference(1.2, rev.CANSparkMax.ControlType.kVelocity)

        prev = self.prev
        self.prev = copy.deepcopy(buf)
