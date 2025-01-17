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
        self.ringSensorValue = False
        self.limitSwitchValue = False
        self.rollerVoltage = 0
        self.infraredSensorValue = False
        
        self.intakeVoltage = 0
        self.intakeVoltageTwo = 0
        self.ejectVoltage = 0
        self.ejectVoltageTwo = 0

        

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        self.intakeVoltage = 0
        self.intakeVoltageTwo = 0
        self.rollerVoltage = 0
        self.ejectVoltage = 0
        self.ejectVoltageTwo = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        table.putBoolean("ringin", self.ringSensorValue)
        table.putBoolean("limit switch", self.limitSwitchValue)
        table.putBoolean("Infrared", self.infraredSensorValue)






class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()
        self.ringSensor = wpilib.DigitalInput(2)
        self.limitSwitch = wpilib.DigitalInput(0)
        self.rollerMotor = rev.SparkMax(10,rev.SparkLowLevel.MotorType.kBrushless)
        self.infraredSensor = wpilib.DigitalInput(1)

        self.intakeMotor = rev.SparkMax(12,rev.SparkLowLevel.MotorType.kBrushless)
        self.intakeMotorTwo = rev.SparkMax(11,rev.SparkLowLevel.MotorType.kBrushless)
        self.ejectMotor = rev.SparkMax(12,rev.SparkLowLevel.MotorType.kBrushless)
        self.ejectMotorTwo = rev.SparkMax(11,rev.SparkLowLevel.MotorType.kBrushless)


    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        buf.ringSensorValue = self.ringSensor.get()
        buf.limitSwitchValue = self.limitSwitch.get()
        self.rollerMotor.setVoltage(buf.rollerVoltage)
        buf.infraredSensorValue = self.infraredSensor.get()

        self.intakeMotor.setVoltage(buf.intakeVoltage)
        self.intakeMotorTwo.setVoltage(buf.intakeVoltageTwo)
        self.ejectMotor.setVoltage(buf.ejectVoltage)
        self.ejectMotorTwo.setVoltage(buf.ejectVoltageTwo)