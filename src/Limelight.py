from ntcore import NetworkTableInstance

# from networktables import NetworkTable
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive


class Limelight:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.llTable = NetworkTableInstance.getDefault().getTable("limelight")

        self.tx: float = 0.0
        self.allowedError: float = 1  # in degrees
        self.strafeDirection: float = 0.0
        self.validTarget: bool = False
        self.targetRequested: bool = False

    def update(self, hal: RobotHALBuffer, autoTarget: bool):

        if autoTarget:
            self.targetRequested = True

        if self.targetRequested:
            self.tx = self.llTable.getNumber("tx", 0.0)
            # Assume targetRequested is valid if tx is within measurable range
            self.validTarget = self.llTable.getNumber("tv", 0) == 1

            if self.validTarget:
                if self.tx > self.allowedError:
                    self.strafeDirection = 1
                elif self.tx < -self.allowedError:
                    self.strafeDirection = -1
                else:
                    self.targetRequested = False
                    self.strafeDirection = 0

                # Uncomment below to activate swerve drive behavior
                hal.AutoTargetPipeDirection = self.strafeDirection
                SwerveDrive.update(self, hal, self.strafeDirection, 0, 0, 0)

        self.table.putNumber("strafe direction", self.strafeDirection)
