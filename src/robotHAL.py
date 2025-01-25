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
        self.shooterSpeeds: list[float] = [0, 0]
        self.otherSpeed: float = 0

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        table.putNumber("shooterSpeed1", self.shooterSpeeds[0])
        table.putNumber("shooterSpeed2", self.shooterSpeeds[1])

        table.putNumber("otherSpeed", self.otherSpeed)


class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

        self.shooterMotors = [
            rev.CANSparkMax(11, rev.CANSparkMax.MotorType.kBrushless),
            rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless),
        ]

        self.otherMotor = rev.CANSparkMax(15, rev.CANSparkMax.MotorType.kBrushless)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        for m, s in zip(self.shooterMotors, buf.shooterSpeeds):
            m.set(s)

        self.otherMotor.set(buf.otherSpeed)
