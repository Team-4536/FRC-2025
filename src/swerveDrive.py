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
    MAX_METERS_PER_SEC = 4.0  # stolen from lastyears code

    def __init__(self) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        oneftInMeters = 0.3048

        self.modulePositions: list[Translation2d] = [
            Translation2d(oneftInMeters, oneftInMeters),
            Translation2d(oneftInMeters, -oneftInMeters),
            Translation2d(-oneftInMeters, oneftInMeters),
            Translation2d(-oneftInMeters, -oneftInMeters),
        ]
        self.kinematics = SwerveDrive4Kinematics(*self.modulePositions)

        self.table.putNumber("SD Joystick X offset", 0)
        self.table.putNumber("SD Joystick Y offset", 0)
        self.table.putNumber("SD Joystick Omega offset", 0)

    def resetOdometry(self, pose: Pose2d, hal: robotHAL.RobotHALBuffer):
        pass

    def update(
        self,
        hal: robotHAL.RobotHALBuffer,
        joystickX: float,
        joystickY: float,
        joystickRotation: float,
    ):
        self.table.putNumber("mech Ctrl X", joystickX)
        self.table.putNumber("mech Ctrl Y", joystickY)
        self.table.putNumber("mech Ctrl Rotation", joystickRotation)

        if abs(joystickX) < 0.05:
            joystickX = 0
        if abs(joystickY) < 0.05:
            joystickY = 0
        if abs(joystickRotation) < 0.05:
            joystickRotation = 0

        self.driveX = joystickX * 1.0 + self.table.getNumber(
            "SD Joystick X offset", 0
        )  # * 0.2
        self.driveY = joystickY * 1.0 + self.table.getNumber(
            "SD Joystick Y offset", 0
        )  # * 0.2
        self.driveRotation = joystickRotation * 0.2 + self.table.getNumber(
            "SD Joystick Omega offset", 0
        )  # 0.0625

        self.chassisSpeeds = ChassisSpeeds(self.driveX, self.driveY, self.driveRotation)

        self.table.putNumber("ChassisSpeeds vx", self.chassisSpeeds.vx)
        self.table.putNumber("ChassisSpeeds vy", self.chassisSpeeds.vy)
        self.table.putNumber("ChassisSpeeds omega", self.chassisSpeeds.omega)

        self.unleashedModules = self.kinematics.toSwerveModuleStates(self.chassisSpeeds)
        swerveModuleStates = self.kinematics.desaturateWheelSpeeds(
            self.unleashedModules,
            self.MAX_METERS_PER_SEC,
        )

        self.table.putNumber(
            "SD Original Turn Setpoint", swerveModuleStates[0].angle.radians()
        )

        FLModuleState = self.optimizeTarget(
            swerveModuleStates[0], Rotation2d(hal.turnPosFL)
        )
        hal.driveFLSetpoint = FLModuleState.speed
        hal.turnFLSetpoint = FLModuleState.angle.radians()
        self.table.putNumber("SD Opimized Turn Setpoint", FLModuleState.angle.radians())

        FRModuleState = self.optimizeTarget(
            swerveModuleStates[1], Rotation2d(hal.turnPosFR)
        )
        hal.driveFRSetpoint = FRModuleState.speed
        hal.turnFRSetpoint = FRModuleState.angle.radians()

        BLModuleState = self.optimizeTarget(
            swerveModuleStates[2], Rotation2d(hal.turnPosBL)
        )
        hal.driveBLSetpoint = BLModuleState.speed
        hal.turnBLSetpoint = BLModuleState.angle.radians()

        BRModuleState = self.optimizeTarget(
            swerveModuleStates[3], Rotation2d(hal.turnPosBR)
        )
        hal.driveBRSetpoint = BRModuleState.speed
        hal.turnBRSetpoint = BRModuleState.angle.radians()

    def updateOdometry(self, hal: robotHAL.RobotHALBuffer):
        pass

    # def optimizeTarget(
    #     self, target: SwerveModuleState, moduleAngle: Rotation2d
    # ) -> SwerveModuleState:

    #     error = angleWrap(target.angle.radians() - moduleAngle.radians())

    #     outputSpeed = target.speed
    #     outputAngle = target.angle.radians()

    #     # optimize
    #     if abs(error) > math.pi / 2:
    #         outputAngle = outputAngle + math.pi
    #         outputSpeed = -outputSpeed

    #     # return
    #     outputAngleRot2d = Rotation2d(angleWrap(outputAngle))
    #     output = SwerveModuleState(outputSpeed, outputAngleRot2d)

    #     return output

    def optimizeTarget(
        self, target: SwerveModuleState, moduleAngle: Rotation2d
    ) -> SwerveModuleState:
        return target
