import limelight
import limelightresults
from ntcore import NetworkTableInstance
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians


class LimeLight:

    def __init__(self):

        self.driveMotorP = 0
        self.turnMotorP = 0
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("Limelight X and Y P", self.driveMotorP)
        self.table.putNumber("Limelight Rotation P", self.turnMotorP)
        discoveredLimelight = limelight.discover_limelights(debug=True)

        if discoveredLimelight:
            limelightAdress = discoveredLimelight[0]
            ll = limelight.Limelight(limelightAdress)
            self.results = ll.get_results()
            self.status = ll.get_status()
            ll.enable_websocket()

        self.holonmicCtrlr = HolonomicDriveController(
            PIDController(self.driveMotorP, 0, 0),
            PIDController(self.driveMotorP, 0, 0),
            PIDController(self.turnMotorP, 0, 0),
        )

    def update(self):

        self.driveMotorP = self.table.getNumber("Limelight X and Y P", self.driveMotorP)
        self.turnMotorP = self.table.getNumber("Limelight Rotation P", self.turnMotorP)
        self.table.putNumber("Limelight X and Y P", self.driveMotorP)
        self.table.putNumber("Limelight Rotation P", self.turnMotorP)
