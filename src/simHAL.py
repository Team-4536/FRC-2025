import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer, RobotHAL
from swerveDrive import SwerveDrive
from timing import TimeData
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.units import radians, meters_per_second
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkClosedLoopController,
    ClosedLoopConfig,
    ClosedLoopSlot,
    LimitSwitchConfig,
)
from wpimath.kinematics import SwerveModulePosition


class RobotSimHAL:
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.hal = RobotHAL()
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putBoolean("first manipulator sensor", False)
        self.table.putBoolean("second manipulator sensor", False)
        

        self.elevatorArbFF: float = 0
        self.elevatorSetpoint: float = 0
        # Rotations
        self.elevatorPos: float = 0
        # These Values are in CCW Radians, (-pi, pi]
        self.turnCCWFL: radians = 0
        self.turnCCWFR: radians = 0
        self.turnCCWBL: radians = 0
        self.turnCCWBR: radians = 0

        self.elevatorSlot: ClosedLoopSlot = ClosedLoopSlot.kSlot0
        self.elevatorControl: SparkMax.ControlType = SparkMax.ControlType.kPosition

        self.driveFLSetpoint: meters_per_second = 0
        self.driveFRSetpoint: meters_per_second = 0
        self.driveBLSetpoint: meters_per_second = 0
        self.driveBRSetpoint: meters_per_second = 0

        self.turnFLSetpoint: radians = 0
        self.turnFRSetpoint: radians = 0
        self.turnBLSetpoint: radians = 0
        self.turnBRSetpoint: radians = 0

        self.secondManipulatorSensor: bool = False
        self.firstManipulatorSensor: bool = False
        self.manipulatorVolts: float = 0

        self.drivePositionsList = [0, 0, 0, 0]
        self.steerPositionList = [0, 0, 0, 0]

    

        self.moduleFL = SwerveModulePosition(0, Rotation2d(radians(0)))
        self.moduleFR = SwerveModulePosition(0, Rotation2d(radians(0)))
        self.moduleBL = SwerveModulePosition(0, Rotation2d(radians(0)))
        self.moduleBR = SwerveModulePosition(0, Rotation2d(radians(0)))

        self.yaw: float = 0
    

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        buf.secondManipulatorSensor = self.table.getBoolean(
            "second manipulator sensor", False
        )
        buf.firstManipulatorSensor = self.table.getBoolean(
            "first manipulator sensor", False
        )

    def resetGyroToAngle(self, ang: float) -> None:
        pass
