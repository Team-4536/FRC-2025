from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Rotation2d
from wpimath.geometry import Pose2d


class setpoints:
    def __init__(self):

        self.controller = HolonomicDriveController(
            PIDController(0.1, 0, 0),
            PIDController(0.1, 0, 0),
            ProfiledPIDControllerRadians(
                1, 0, 0, TrapezoidProfileRadians.Constraints(6.28, 3.14)
            ),
        )
        self.currentPose = Pose2d(0, 0, 0)
        self.desiredPose = Pose2d(1, 0, 0)

    def update(self):
        adjustedSpeeds = self.controller.calculate(
            self.currentPose, self.desiredPose, 0.25, Rotation2d.fromDegrees(70.0)
        )
