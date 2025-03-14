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
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import feetToMeters


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive:
    MAX_METERS_PER_SEC = 8.0  # stolen from lastyears code

    def __init__(self) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        oneftInMeters = feetToMeters(1)

        frontLeftLocation = Translation2d(oneftInMeters, oneftInMeters)
        frontRightLocation = Translation2d(oneftInMeters, -oneftInMeters)
        backLeftLocation = Translation2d(-oneftInMeters, oneftInMeters)
        backRightLocation = Translation2d(-oneftInMeters, -oneftInMeters)
        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        )

        # =======NEW, NOT TUNED=======================================
        constraints = TrapezoidProfileRadians.Constraints(4 * math.pi, 20 * math.pi)
        xPID = PIDController(0.1, 0, 0)
        yPID = PIDController(0.1, 0, 0)
        rotPID = ProfiledPIDControllerRadians(1.4, 0, 0, constraints)

        self.holonomicController = HolonomicDriveController(xPID, yPID, rotPID)

        # ============================================================

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
        RTriggerScalar: float,
    ):
        self.table.putNumber("Drive Ctrl X", joystickX)
        self.table.putNumber("Drive Ctrl Y", joystickY)
        self.table.putNumber("Drive Ctrl Rotation", joystickRotation)

        if math.sqrt(joystickX**2 + joystickY**2) < 0.08:
            joystickX = 0
            joystickY = 0
        if abs(joystickRotation) < 0.06:
            joystickRotation = 0

        self.offsetX = 0.05 * np.sign(joystickX)
        self.offsetY = 0.05 * np.sign(joystickY)
        self.offsetR = 0.05 * np.sign(joystickRotation)

        self.proxyDeadZoneX = (joystickX - self.offsetX) * 3.5
        self.proxyDeadZoneY = (joystickY - self.offsetY) * 3.5
        self.proxyDeadZoneR = (joystickRotation - self.offsetR) * 10

        # the controller's x axis the the ChassisSpeeds' y axis and same for the other x and y axies
        # the signes are flipped for the differences too
        self.driveY = -self.proxyDeadZoneX
        self.driveX = -self.proxyDeadZoneY
        self.driveRotation = -self.proxyDeadZoneR

        driveVector = Translation2d(self.driveX, self.driveY)

        # abs drive toggle
        if hal.fieldOriented:
            driveVector = driveVector.rotateBy(Rotation2d(-hal.yaw))

        # disable rotatioanl PID if turn stick is moved
        if self.driveRotation != 0:
            hal.rotPIDToggle = False

        self.table.putNumber("z_PID Setpoint", hal.rotPIDsetpoint)
        self.table.putBoolean("z_Absolute Drive", hal.fieldOriented)

        # --------------EMMETT'S SCARY NEW STUFF-----------------------------------
        rotPos = Rotation2d(hal.yaw)
        fakeBotPos = Pose2d(0, 0, rotPos)
        rotTarget = Rotation2d.fromDegrees(hal.rotPIDsetpoint)

        # returns chassis speeds
        adjustedSpeeds = self.holonomicController.calculate(
            fakeBotPos, fakeBotPos, 0, rotTarget
        )
        # take only rotational speed
        rotPIDSpeed = adjustedSpeeds.omega

        # only use rotational PID if it's activated
        if hal.rotPIDToggle:
            rotFinal = rotPIDSpeed * 5
        else:
            rotFinal = -self.driveRotation  # copied from HCPA code

        # -------------------------------------------------------------------

        self.chassisSpeeds = ChassisSpeeds(
            driveVector.X() * 0.5 * 4**RTriggerScalar,
            driveVector.Y() * 0.5 * 4**RTriggerScalar,
            rotFinal,
        )

        self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
        self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
        self.table.putNumber(
            "SD ChassisSpeeds omega (rotFinal)", self.chassisSpeeds.omega
        )
        self.table.putNumber(
            "SD RotPIDSpeed omega (adjustedSpeedsOmega)",
            adjustedSpeeds.omega,  # * (180 / math.pi)
        )
        self.table.putBoolean("rotPIDToggle", hal.rotPIDToggle)
        self.table.putNumber("z_target rotDeg", rotTarget.degrees())
        self.table.putNumber("z_current rotDeg", fakeBotPos.rotation().degrees())

        self.unleashedModules = self.kinematics.toSwerveModuleStates(self.chassisSpeeds)
        swerveModuleStates = self.kinematics.desaturateWheelSpeeds(
            self.unleashedModules,
            self.MAX_METERS_PER_SEC,
        )

        self.table.putNumber(
            "SD Module Original Turn Setpoint", swerveModuleStates[0].angle.radians()
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
