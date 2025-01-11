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

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        table.putBoolean("ringin", self.ringSensorValue)
        table.putBoolean("limit switch", self.limitSwitchValue)




class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()
        self.ringSensor = wpilib.DigitalInput(2)
        self.limitSwitch = wpilib.DigitalInput(0)
        self.myMotorFun = rev.CANSparkMax(1)




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
