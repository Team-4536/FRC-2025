import math
import json
import limelight  # type: ignore
import limelightresults  # type: ignore
from ntcore import NetworkTableInstance
from networktables import NetworkTables
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive


class LimeLightClass:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.llTable = NetworkTableInstance.getDefault().getTable("limelight")

        self.discoveredLimelight = limelight.discover_limelights()

        self.tx: float = 0.0
        self.allowedError: float = 5.0  # in degrees
        self.strafeDirection: float = 0.0
        self.validTarget: bool = False

        self.ll = None

        if self.discoveredLimelight:
            print("Discovered Limelight")
            limelightAddress = self.discoveredLimelight[0]
            self.table.putString("limelight address", limelightAddress)

            self.ll = limelight.Limelight(limelightAddress)
            self.ll.enable_websocket()
            self.status = self.ll.get_status()
        else:
            print("No Limelight discovered")

    def update(self, hal: RobotHALBuffer):
        if self.ll is not None:

            self.ll.get_latest_results()
            self.tx = self.llTable.getNumber("tx", -1)
            self.table.putNumber("tx", self.tx)
            # Assume target is valid if tx is within measurable range
            self.validTarget = True  # Optionally improve logic using parsed.validity

            if self.validTarget:
                if self.tx > self.allowedError:
                    self.strafeDirection = 1
                elif self.tx < -self.allowedError:
                    self.strafeDirection = -1
                else:
                    self.strafeDirection = 0

                # Uncomment below to activate swerve drive behavior
                # SwerveDrive.update(self, hal, self.strafeDirection, 0, 0, 0)

            self.table.putNumber("strafe direction", self.strafeDirection)
