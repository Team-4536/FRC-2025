import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer, RobotHAL
from swerveDrive import SwerveDrive
from timing import TimeData
from wpimath.geometry import Rotation2d, Translation2d
import wpilib
from wpilib import SmartDashboard
from manipulator import ManipulatorSubsystem
from elevator import ElevatorSubsystem
from elevator import ElevatorMode
from wpimath.units import meter


class RobotSimHAL:
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putBoolean("first manipulator sensor", False)
        self.table.putBoolean("second manipulator sensor", False)
        self.table.putNumber("arm voltage", 0)
        self.table.putNumber("manipulator voltage", 0)
        # self.table.getNumber("Elevator Mode", self.mode)
        self.prev.yaw = 1
        self.drivePositionsList = [0, 0, 0, 0]
        self.steerPositionList = [0, 0, 0, 0]

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        buf.secondManipulatorSensor = self.table.getBoolean(
            "second manipulator sensor", False
        )
        buf.firstManipulatorSensor = self.table.getBoolean(
            "first manipulator sensor", False
        )
        buf.yaw += 0.1
        self.drivePositionsList = [0, 0, 0, 0]
        self.steerPositionList = [0, 0, 0, 0]
        wpilib.SmartDashboard.putNumber("yaw", buf.yaw)

        self.table.putNumber("arm voltage", buf.armVolts)
        buf.manipulatorVolts = self.table.getNumber("manipulator voltage", 0)
        # self.table.putNumber("Elevator State", self)

        # elevator state
        # elevator position

        # limit switches

        self.table.putNumber("arm voltage", buf.armVolts)
        buf.manipulatorVolts = self.table.getNumber("manipulator voltage", 0)
        buf.drivePositionsList[0] += ((buf.driveFLSetpoint*0.002) / ((2*math.pi) * 0.05)) * 2 * math.pi
        buf.drivePositionsList[1] += ((buf.driveFRSetpoint*0.002) / ((2*math.pi) * 0.05)) * 2 * math.pi
        buf.drivePositionsList[2] += ((buf.driveBLSetpoint*0.002) / ((2*math.pi) * 0.05)) * 2 * math.pi
        buf.drivePositionsList[3] += ((buf.driveBRSetpoint*0.002) / ((2*math.pi) * 0.05)) * 2 * math.pi
        # self.table.putNumber("Elevator State", self)

    # elevator state
    # elevator position

    # limit switches
        buf.armVolts = self.table.getNumber("arm voltage", 0)
        buf.manipulatorVolts = self.table.getNumber("manipulator voltage", 0)

    def resetGyroToAngle(self, ang: float) -> None:
        pass
