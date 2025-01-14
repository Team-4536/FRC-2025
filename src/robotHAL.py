import copy
import math
import robot


import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
from timing import TimeData
from ntcore import NetworkTableInstance

class RobotHALBuffer:
    def __init__(self) -> None:
        
        self.driveVolts = 0
    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


        



class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

        self.driveMotorFL = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorFR = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBL = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBR = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        self.driveMotorFL.set(buf.driveVolts*0.1)
        self.table.putNumber('hal drive volts', buf.driveVolts)


