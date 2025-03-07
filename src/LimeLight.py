import limelight  # type: ignore
import limelightresults  # type: ignore
from ntcore import NetworkTableInstance
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive
import math
import json


class LimeLight:

    def __init__(self):

        self.driveMotorP: float = 0
        self.turnMotorP: float = 0
        self.ll = None
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.llTable = NetworkTableInstance.getDefault().getTable("limelight")
        self.table.putNumber("Limelight X and Y P", self.driveMotorP)
        self.table.putNumber("Limelight Rotation P", self.turnMotorP)
        self.discoveredLimelight: bool = limelight.discover_limelights(debug=True)

        if self.discoveredLimelight:
            print("discovered limelight")
            limelightAdress = self.discoveredLimelight[0]
            self.ll = limelight.Limelight(limelightAdress)
            self.llResults = self.ll.get_results()
            self.parse = limelightresults.parse_results(self.llResults)
            self.status = self.ll.get_status()
            self.ll.enable_websocket()
        else:
            print("no limelight discovered")

    def update(self, hal: RobotHALBuffer):
        # pass
        if self.ll is not None:
            self.driveMotorP = self.table.getNumber(
                "Limelight X and Y P", self.driveMotorP
            )
            self.turnMotorP = self.table.getNumber(
                "Limelight Rotation P", self.turnMotorP
            )

            self.table.putNumber("Limelight X and Y P", self.driveMotorP)
            self.table.putNumber("Limelight Rotation P", self.turnMotorP)
            self.results = self.ll.get_latest_results()

        tx: float = self.llTable.getNumber("tx", 0.0)
        # from -28.5 (left) to 28.5 (right) degrees
        validTarget: bool = 1 == self.llTable.getNumber("tv", 0)
        # 1 means a valid target is found

        alowedError: float = 5  # in degrees
        strafeDirection: float = 0

        if validTarget:

            if tx > alowedError:
                strafeDirection = 1

            elif tx < -alowedError:
                strafeDirection = -1

        # SwerveDrive.update(self, hal, strafeDirection, 0, 0, 0)

        self.table.putNumber("strafe direction", strafeDirection)
