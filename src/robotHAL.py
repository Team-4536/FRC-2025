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
        self.rollerVoltage = 0
        self.motorSensorValue = False

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
       self.rollerVoltage = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()
        self.rollerMotor = rev.SparkMax(2,rev.SparkLowLevel.MotorType.kBrushless)
        self.motor1 = rev.SparkMax(1, rev.SparkLowLevel.MotorType.kBrushless)
        self.motorSensor = wpilib.DigitalInput(0)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)
        bob = self.motor1.getReverseLimitSwitch()
        self.rollerMotor.setVoltage(buf.rollerVoltage)
        buf.motorSensorValue = self.motorSensor.get()
        self.table.putNumber("voltage", buf.rollerVoltage)
        self.table.putBoolean("reverselimitswitch", bob.get())
        #self.table.putBoolean("forwardlimitswitch", self.motor1.getForwardLimitSwitch())
        