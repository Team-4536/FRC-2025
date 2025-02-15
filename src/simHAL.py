import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive
from timing import TimeData
from wpimath.geometry import Rotation2d, Translation2d
from manipulator import ManipulatorSubsystem

class RobotSimHAL:
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putBoolean("first manipulator sensor", False)
        self.table.putBoolean("second manipulator sensor", False)
        self.table.putNumber("arm voltage", 0)
        self.table.putNumber("manipulator voltage", 0)

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        buf.secondManipulatorSensor = self.table.getBoolean(
            "second manipulator sensor", False
        )
        buf.firstManipulatorSensor = self.table.getBoolean(
            "first manipulator sensor", False
        )
        buf.armVolts = self.table.getNumber(
            "arm voltage", 0
        )
        buf.manipulatorVolts = self.table.getNumber(
            "manipulator voltage", 0
        )

    def resetGyroToAngle(self, ang: float) -> None:
        pass
