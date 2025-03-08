import limelight  # type: ignore
import limelightresults  # type: ignore
from ntcore import NetworkTableInstance
import ntcore
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive
import math


class LimeLightClass:

    def __init__(self):

        self.driveMotorP: float = 0
        self.turnMotorP: float = 0
        self.ll = None
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.llTable = NetworkTableInstance.getDefault().getTable("limelight")
        bobsUncle = False
        self.llTable.setBoolean("X ad Y pos", bobsUncle)
        anotherCopy = self.llTable.getBoolean("X and Y pos")
        # ntcore.NetworkTableInstance.startServer()
        self.discoveredLimelight = limelight.discover_limelights(debug=True)

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
        pass
        if self.ll is not None:

            self.results = self.ll.get_latest_results()
            self.parse = limelightresults.parse_results(self.llResults)

        for tag in self.parse.fiducialResults:
            tx: float = self.results.get("tx", None)
        self.table.putNumber("tx", tx)
        # from -28.5 (left) to 28.5 (right) degrees
        validTarget: bool = 1 == self.parse.validity
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
