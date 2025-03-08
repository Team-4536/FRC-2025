import math
import robotHAL
import robot
import numpy as np
import wpilib
from wpilib import SmartDashboard, Field2d
from ntcore import NetworkTableInstance
from real import angleWrap
import wpimath.units
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.units import meters, radians
import wpimath.unitsfrom wpimath.units import radians


# adapted from here https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive:
    MAX_METERS_PER_SEC = 8.0  # stolen from lastyears code

    def __init__(self) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        oneftInMeters = 0.3048

        self.modulePositions: list[Translation2d] = [
            Translation2d(-oneftInMeters, oneftInMeters),
            Translation2d(oneftInMeters, oneftInMeters),
            Translation2d(-oneftInMeters, -oneftInMeters),
            Translation2d(oneftInMeters, -oneftInMeters),
        ]

        ModulePos = SwerveModulePosition(0, Rotation2d(0))
        modulePosList: list[SwerveModulePosition * 4] = [  # type: ignore
            ModulePos,
            ModulePos,
            ModulePos,
            ModulePos,
        ]

        self.angle = Rotation2d(0)
        self.pose = Pose2d(
            wpimath.units.meters(0), wpimath.units.meters(0), wpimath.units.radians(0)
        )
        self.kinematics = SwerveDrive4Kinematics(*self.modulePositions)
        self.odometry = SwerveDrive4Odometry(
            self.kinematics, self.angle, tuple(modulePosList), self.pose
        )

        self.table.putNumber("SD Joystick X offset", 0)
        self.table.putNumber("SD Joystick Y offset", 0)
        self.table.putNumber("SD Joystick Omega offset", 0)

        ModulePos = SwerveModulePosition(0, Rotation2d(0))
        modulePosList: list[SwerveModulePosition * 4] = [  # type: ignore
            ModulePos,
            ModulePos,
            ModulePos,
            ModulePos,
        ]

        self.angle = Rotation2d(0)
        self.pose = Pose2d(
            wpimath.units.meters(0), wpimath.units.meters(0), wpimath.units.radians(0)
        )
        self.odometry = SwerveDrive4Odometry(
            self.kinematics, self.angle, tuple(modulePosList), self.pose
        )

    def resetOdometry(self, pose: Pose2d, hal: robotHAL.RobotHALBuffer):

        modulePosList = (
            SwerveModulePosition(
                hal.drivePositionsList[0], Rotation2d(radians(hal.steerPositionList[0]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[1], Rotation2d(radians(hal.steerPositionList[1]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[2], Rotation2d(radians(hal.steerPositionList[2]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[3], Rotation2d(radians(hal.steerPositionList[3]))
            ),
        )
        # modulePosList = (hal.moduleFL, hal.moduleFR, hal.moduleBL, hal.moduleBR)
        self.odometry.resetPosition(Rotation2d(hal.yaw), modulePosList, pose)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(
        self,
        hal: robotHAL.RobotHALBuffer,
        joystickX: float,
        joystickY: float,
        joystickRotation: float,
        RTriggerScalar: float,
    ):
        self.table.putNumber("Drive Ctrl X", joystickX)
        self.table.putNumber("Drive Ctrl Y", joystickY)
        self.table.putNumber("Drive Ctrl Rotation", joystickRotation)

        if math.sqrt(joystickX**2 + joystickY**2) < 0.08:
            joystickX = 0
            joystickY = 0
        if abs(joystickRotation) < 0.05:
            joystickRotation = 0

        self.number = 1

        self.offsetX = 0.05 * np.sign(joystickX)
        self.offsetY = 0.05 * np.sign(joystickY)
        self.offsetR = 0.05 * np.sign(joystickRotation)

        self.proxyDeadZoneX = (joystickX - self.offsetX) * 3.5
        self.proxyDeadZoneY = (joystickY - self.offsetY) * 3.5
        self.proxyDeadZoneR = (joystickRotation - self.offsetR) * 3.5

        self.driveX = self.proxyDeadZoneX
        self.driveY = self.proxyDeadZoneY
        self.driveRotation = self.proxyDeadZoneR

        driveVector = Translation2d(self.driveX, self.driveY)
        driveVector = driveVector.rotateBy(Rotation2d(-hal.yaw))

        self.chassisSpeeds = ChassisSpeeds(
            driveVector.X() * 0.5 * 4**RTriggerScalar,
            driveVector.Y() * 0.5 * 4**RTriggerScalar,
            -self.driveRotation * 3,
        )

        self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
        self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
        self.table.putNumber("SD ChassisSpeeds omega", self.chassisSpeeds.omega)

        self.unleashedModules = self.kinematics.toSwerveModuleStates(self.chassisSpeeds)
        swerveModuleStates = self.kinematics.desaturateWheelSpeeds(
            self.unleashedModules,
            self.MAX_METERS_PER_SEC,
        )

        FLModuleState = self.optimizeTarget(
            swerveModuleStates[0], Rotation2d(hal.turnCCWFL)
        )
        hal.driveFLSetpoint = FLModuleState.speed
        hal.turnFLSetpoint = FLModuleState.angle.radians()

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

        modulePosList = (
            SwerveModulePosition(
                hal.drivePositionsList[0], Rotation2d(radians(hal.steerPositionList[0]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[1], Rotation2d(radians(hal.steerPositionList[1]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[2], Rotation2d(radians(hal.steerPositionList[2]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[3], Rotation2d(radians(hal.steerPositionList[3]))
            ),
        )

        self.odometry.update(Rotation2d(hal.yaw), modulePosList)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        # self.OdomField.setRobotPose(self.odometry.getPose().X, self.odometry.getPose().Y, self.odometry.getPose().rotation)

        self.table.putNumber("odomX", self.odometry.getPose().x)
        self.table.putNumber("odomy", self.odometry.getPose().y)

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

    def updateWithoutSticks(
        self, hal: robotHAL.RobotHALBuffer, chassisSpeed: ChassisSpeeds
    ):

        self.chassisSpeeds = chassisSpeed

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
