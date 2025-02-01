import copy
import math

import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
from timing import TimeData
from ntcore import NetworkTableInstance


class RobotHALBuffer:
    def __init__(self) -> None:
        self.elevatorVoltage = 0
        self.elevatorRotations = 0

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        self.elevatorVoltage = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        self.table.putNumber("Elevator Voltage", self.elevatorVoltage)
        self.table.putNumber("Elevator Rotations", self.elevatorRotations)


class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()
        self.elevatorMotor = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)
        self.elevatorMotor.setVoltage(buf.elevatorVoltage)
        buf.elevatorRotations = self.elevatorMotor.getPosition()