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
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfileRadians


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

        self.holonomicController = HolonomicDriveController(
            PIDController(0.1, 0, 0),
            PIDController(0.1, 0, 0),
            ProfiledPIDController(0.1, 0, 0, TrapezoidProfileRadians.Constraints(6.28, 3/4 * math.pi))
            )

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
        self.table.putNumber("Drive Ctrl X", joystickX)
        self.table.putNumber("Drive Ctrl Y", joystickY)
        self.table.putNumber("Drive Ctrl Rotation", joystickRotation)

        if abs(joystickX) < 0.05:
            joystickX = 0
        if abs(joystickY) < 0.05:
            joystickY = 0
        if abs(joystickRotation) < 0.05:
            joystickRotation = 0

        self.driveX = joystickX * 5 + self.table.getNumber("SD Joystick X offset", 0)
        self.driveY = joystickY * 5 + self.table.getNumber("SD Joystick Y offset", 0)
        self.driveRotation = joystickRotation * -6.5 + self.table.getNumber(
            "SD Joystick Omega offset", 0
        )

        #-------------
        rotPos = Rotation2d(hal.yaw)
        fakeBotPos = Pose2d(0, 0, rotPos)

        rotTarget = Rotation2d(90) #TEMPORARY CONSTANT


        adjustedSpeeds = self.holonomicController.calculate(fakeBotPos, 0, 0, rotTarget.fromDegrees)
        
        rotPIDSpeed = adjustedSpeeds.omega


        driveVector = Translation2d(joystickX, joystickY)

        #toggle abs drive
        if hal.fieldOriented == True:
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
