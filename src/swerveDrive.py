import math
import robotHAL
import robot
import numpy as np
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

        # self.modulePositions: list[Translation2d] = [
        #     Translation2d(oneftInMeters, oneftInMeters),
        #     Translation2d(oneftInMeters, -oneftInMeters),
        #     Translation2d(-oneftInMeters, oneftInMeters),
        #     Translation2d(-oneftInMeters, -oneftInMeters),
        # ]
        self.modulePositions: list[Translation2d] = [
            Translation2d(-oneftInMeters, oneftInMeters),
            Translation2d(oneftInMeters, oneftInMeters),
            Translation2d(-oneftInMeters, -oneftInMeters),
            Translation2d(oneftInMeters, -oneftInMeters),
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
        LBumperSpeedToggle: float
    ):
        self.table.putNumber("Drive Ctrl X", joystickX)
        self.table.putNumber("Drive Ctrl Y", joystickY)
        self.table.putNumber("Drive Ctrl Rotation", joystickRotation)

        if abs(joystickX) < 0.05:
            joystickX = 0
        if abs(joystickY) < 0.05:
            joystickY = 0
        if abs(joystickRotation) < 0.05:
            joystickRotation = 0

        self.number = 1
        
        self.offsetX = (0.05*np.sign(joystickX))
        self.offsetY = (0.05*np.sign(joystickY))
        self.offsetR = (0.05*np.sign(joystickRotation))

        self.proxyDeadZoneX = (joystickX - self.offsetX) * 3.5
        self.proxyDeadZoneY = (joystickY - self.offsetY) * 3.5
        self.proxyDeadZoneR = (joystickRotation - self.offsetR) * 3.5

        self.driveX = self.proxyDeadZoneX*((LBumperSpeedToggle)+1)#self.table.getNumber("SD Joystick X offset", 0)
        self.driveY = self.proxyDeadZoneY*((LBumperSpeedToggle)+1)#self.table.getNumber("SD Joystick Y offset", 0)
        self.driveRotation = self.proxyDeadZoneR*((LBumperSpeedToggle)+1)#self.table.getNumber(
            #"SD Joystick Omega offset", 0
        #)

        driveVector = Translation2d(joystickX, joystickY)
        driveVector = driveVector.rotateBy(Rotation2d(-hal.yaw))

        # self.chassisSpeeds = ChassisSpeeds(self.driveX, self.driveY, self.driveRotation)
        self.chassisSpeeds = ChassisSpeeds(
            driveVector.X() * 5, driveVector.Y() * 5, self.driveRotation
        )

        self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
        self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
        self.table.putNumber("SD ChassisSpeeds omega", self.chassisSpeeds.omega)

        self.unleashedModules = self.kinematics.toSwerveModuleStates(self.chassisSpeeds)
        swerveModuleStates = self.kinematics.desaturateWheelSpeeds(
            self.unleashedModules,
            self.MAX_METERS_PER_SEC,
        )

        self.table.putNumber(
            "SD Original Turn Setpoint", swerveModuleStates[0].angle.radians()
        )

        self.table.putNumber("SD Original Drive Setpoint", swerveModuleStates[0].speed)

        FLModuleState = self.optimizeTarget(
            swerveModuleStates[0], Rotation2d(hal.turnCCWFL)
        )
        hal.driveFLSetpoint = FLModuleState.speed
        hal.turnFLSetpoint = FLModuleState.angle.radians()
        self.table.putNumber("SD Opimized Turn Setpoint", FLModuleState.angle.radians())

        FRModuleState = self.optimizeTarget(
            swerveModuleStates[1], Rotation2d(hal.turnCCWFR)
        )
        hal.driveFRSetpoint = FRModuleState.speed
        hal.turnFRSetpoint = FRModuleState.angle.radians()

        BLModuleState = self.optimizeTarget(
            swerveModuleStates[2], Rotation2d(hal.turnCCWBL)
        )
        hal.driveBLSetpoint = BLModuleState.speed
        hal.turnBLSetpoint = BLModuleState.angle.radians()

        BRModuleState = self.optimizeTarget(
            swerveModuleStates[3], Rotation2d(hal.turnCCWBR)
        )
        hal.driveBRSetpoint = BRModuleState.speed
        hal.turnBRSetpoint = BRModuleState.angle.radians()

    def updateOdometry(self, hal: robotHAL.RobotHALBuffer):
        pass

    def optimizeTarget(
        self, target: SwerveModuleState, moduleAngle: Rotation2d
    ) -> SwerveModuleState:

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
