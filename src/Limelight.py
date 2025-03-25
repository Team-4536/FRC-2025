from ntcore import NetworkTableInstance
from wpimath.controller import (
    PIDController,
)
from wpimath.kinematics import ChassisSpeeds
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive


class Limelight:

    DESIRED_TX: float = 0

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.llTable = NetworkTableInstance.getDefault().getTable("limelight")

        self.tx: float = 0.0
        self.allowedError: float = 1  # in degrees
        self.strafeDirection: float = 0.0
        self.validTarget: bool = False
        self.targetRequested: bool = False
        self.reefPIDController = PIDController(0.1, 0, 0)

    def update(self, hal: RobotHALBuffer, swerve: SwerveDrive):

        self.tx = self.llTable.getNumber("tx", 0.0)
        # Assume targetRequested is valid if tx is within measurable range
        self.validTarget = self.llTable.getNumber("tv", 0) == 1

        if not self.validTarget:
            swerve.updateWithoutSticks(hal, ChassisSpeeds(0, 0, 0))
            return

        hal.AutoTargetPipeDirection = self.strafeDirection
        chassisY = self.reefPIDController.calculate(self.tx, self.DESIRED_TX)
        chassisSpeeds = ChassisSpeeds(0, chassisY, 0)
        swerve.updateWithoutSticks(hal, chassisSpeeds)
        self.table.putNumber("Strafe Speed", chassisY)
