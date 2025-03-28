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
        self.txForMultipleTargets = []

    def update(self, hal: RobotHALBuffer, swerve: SwerveDrive):

        if not self.validTarget:
            swerve.updateWithoutSticks(hal, ChassisSpeeds(0, 0, 0))
            return
        rawTargets = self.llTable.getNumberArray("raw_targets", [])
        self.txForMultipleTargets = []
        for i in range(0, len(rawTargets), 3):
            self.txForMultipleTargets.append(rawTargets[i])

        self.validTarget = self.llTable.getNumber("tv", 0) == 1
        if self.txForMultipleTargets[1]:
            smallest = self.txForMultipleTargets[0]
            for i in (1, len(self.txForMultipleTargets)):
                if smallest > self.txForMultipleTargets[i]:
                    smallest = self.txForMultipleTargets[i]
            self.tx = smallest
        else:
            self.tx = self.llTable.getNumber("tx", 0.0)

        chassisY = self.reefPIDController.calculate(self.tx, self.DESIRED_TX)
        chassisSpeeds = ChassisSpeeds(0, chassisY, 0)
        swerve.updateWithoutSticks(hal, chassisSpeeds)
        self.table.putNumber("Strafe Speed", chassisY)
