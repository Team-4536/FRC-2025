import math
import robotHAL
import robot

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

    
        

    def __init__(
        self, angle: Rotation2d, pose: Pose2d, wheelStates: list[SwerveModulePosition]
    ) -> None:
        
        oneftInMeters = 0.3048

        self.modulePositions: list[Translation2d] = [
            Translation2d(oneftInMeters, oneftInMeters),
            Translation2d(oneftInMeters, -oneftInMeters),
            Translation2d(-oneftInMeters, oneftInMeters),
            Translation2d(-oneftInMeters,-oneftInMeters)]
        self.kinematics = SwerveDrive4Kinematics(*self.modulePositions)
        
        

    

    def resetOdometry(self, pose: Pose2d, hal: robotHAL.RobotHALBuffer):
        pass

    def update(self, dt: float, hal: robotHAL.RobotHALBuffer, speed: ChassisSpeeds):
        
        wheelPositions = 
        self.unleashedModules = self.kinematics.toSwerveModuleStates(robot.chassisSpeeds)
        self.swerveModule = self.kinematics.desaturateWheelSpeeds(self.unleashedModules)

        for i in range(4):
            state = self.optimizeTarget(self.swerveModule[i], wheelPositions[i].angle)

    def updateOdometry(self, hal: robotHAL.RobotHALBuffer):
        pass


    def optimizeTarget(self, target: SwerveModuleState, moduleAngle: Rotation2d) -> SwerveModuleState:

        error = angleWrap(target.angle.radians() - moduleAngle.radians())

        outputSpeed = target.speed
        outputAngle = target.angle.radians()

        # optimize
        if abs(error) > math.pi / 2:
            outputAngle = outputAngle + math.pi
            outputSpeed = -outputSpeed

        # return
        outputAngleRot2d = Rotation2d(angleWrap(outputAngle))
        output = SwerveModuleState(outputSpeed, outputAngleRot2d)

        return output