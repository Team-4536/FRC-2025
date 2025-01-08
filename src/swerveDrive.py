import math

import robotHAL
from ntcore import NetworkTableInstance
from real import angleWrap
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive():
    def __init__(self, angle: Rotation2d, pose: Pose2d, wheelStates: list[SwerveModulePosition]) -> None:
        pass

    def resetOdometry(self, pose: Pose2d, hal: robotHAL.RobotHALBuffer):
        pass

    def update(self, dt: float, hal: robotHAL.RobotHALBuffer, speed: ChassisSpeeds):
        pass

    def updateOdometry(self, hal: robotHAL.RobotHALBuffer):
        pass
