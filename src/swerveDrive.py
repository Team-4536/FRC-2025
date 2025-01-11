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
class SwerveDrive:

    #oneftInMeters = 0.3048
    #modPos = list[Translation2d] = [
     #   Translation2d(oneftInMeters, oneftInMeters),
      #  Translation2d(oneftInMeters, -oneftInMeters),
       # Translation2d(-oneftInMeters, oneftInMeters),
        #Translation2d(-oneftInMeters,-oneftInMeters)]
        

    def __init__(
        self, angle: Rotation2d, pose: Pose2d, wheelStates: list[SwerveModulePosition]
    ) -> None:
        
        #self.kinematics = SwerveDrive4Kinematics(self.modPos)
        #self.odometry = SwerveDrive4Odometry(self.kinematics, angle, pose)
        #self.wheelStates = wheelStates
        pass

    def resetOdometry(self, pose: Pose2d, hal: robotHAL.RobotHALBuffer):
        pass

    def update(self, dt: float, hal: robotHAL.RobotHALBuffer, speed: ChassisSpeeds):
        
        pass

    def updateOdometry(self, hal: robotHAL.RobotHALBuffer):
        pass
