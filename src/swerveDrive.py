import math
import math
import wpilib
from wpilib import SmartDashboard, Field2d
import numpy as np
from ntcore import NetworkTableInstance
from real import angleWrap
import wpimath.units
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import feetToMeters, radians
from ntcore import NetworkTableInstance
from wpimath.units import feetToMeters
from ntcore import NetworkTableInstance
from robot import Robot
import rev
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkClosedLoopController,
    ClosedLoopConfig,
    ClosedLoopSlot,
    LimitSwitchConfig,
)
import navx

# from math import radians


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive:
    MAX_METERS_PER_SEC = 8.0  # stolen from lastyears code

    def __init__(self) -> None:

        self.angle = Rotation2d(0)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.FMSData = NetworkTableInstance.getDefault().getTable("FMSInfo")

        self.driveMotorFL = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.driveMotorFR = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.driveMotorBL = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.driveMotorBR = SparkMax(2, SparkMax.MotorType.kBrushless) #change numbers they aren't correct

        self.turnMotorFL = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.turnMotorFR = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.turnMotorBL = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.turnMotorBR = SparkMax(2, SparkMax.MotorType.kBrushless)

        self.turnPosFL = self.turnMotorFL.getAbsoluteEncoder().getPosition() #in CCW radians
        self.turnPosFR = self.turnMotorFR.getAbsoluteEncoder().getPosition()
        self.turnPosBL = self.turnMotorBL.getAbsoluteEncoder().getPosition()
        self.turnPosBR = self.turnMotorBR.getAbsoluteEncoder().getPosition()

        oneftInMeters = feetToMeters(1)

        frontLeftLocation = Translation2d(oneftInMeters, oneftInMeters)
        frontRightLocation = Translation2d(oneftInMeters, -oneftInMeters)
        backLeftLocation = Translation2d(-oneftInMeters, oneftInMeters)
        backRightLocation = Translation2d(-oneftInMeters, -oneftInMeters)
        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        )

        self.yawOffset = 0.0

        self.fieldOriented = True

        # =======NEW, NOT TUNED=======================================
        constraints = TrapezoidProfileRadians.Constraints(4 * math.pi, 20 * math.pi)
        xPID = PIDController(0, 0, 0)
        yPID = PIDController(0, 0, 0)
        rotPID = ProfiledPIDControllerRadians(1.4, 0, 0, constraints)

        self.holonomicController = HolonomicDriveController(xPID, yPID, rotPID)
        

        # ============================================================

        driveMotorPIDConfig = SparkMaxConfig()
        driveMotorPIDConfig.smartCurrentLimit(40)
        driveMotorPIDConfig.closedLoop.pidf(0.00019, 0, 0, 0.00002).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)

        driveMotorPIDConfig.disableFollowerMode()

        driveMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            2000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(50000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(1)
        driveMotorPIDConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        


        self.FLSwerveModule = SwerveModuleController(
            "FL",
            self.driveMotorFL,
            driveMotorPIDConfig,
            self.turnMotorFL,
            turnMotorPIDConfig,
        )
        self.FRSwerveModule = SwerveModuleController(
            "FR",
            self.driveMotorFR,
            driveMotorPIDConfig,
            self.turnMotorFR,
            turnMotorPIDConfig,
        )
        self.BLSwerveModule = SwerveModuleController(
            "BL",
            self.driveMotorBL,
            driveMotorPIDConfig,
            self.turnMotorBL,
            turnMotorPIDConfig,
        )
        self.BRSwerveModule = SwerveModuleController(
            "BR",
            self.driveMotorBR,
            driveMotorPIDConfig,
            self.turnMotorBR,
            turnMotorPIDConfig,
        )

    def update(
        self,
        robot: Robot,
        joystickX: float,
        joystickY: float,
        joystickRotation: float,
        RTriggerScalar: float,
        resetOffset: bool
    ):
        
        yaw = robot.yaw

        self.table.putNumber("Drive Ctrl X", joystickX)
        self.table.putNumber("Drive Ctrl Y", joystickY)
        self.table.putNumber("Drive Ctrl Rotation", joystickRotation)

        if (
            math.sqrt(joystickX**2 + joystickY**2) < 0.08
        ):  # Distance formula = how far from center, stops tiny stick drift from affecting
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

        if resetOffset:
            self.yawOffset = yaw

        self.table.putNumber("absDriveOffset", self.yawOffset)

        # abs drive toggle
        if self.fieldOriented:
            driveVector = driveVector.rotateBy(Rotation2d(-yaw + self.yawOffset))

        # disable rotatioanl PID if turn stick is moved
        # if self.driveRotation != 0:
        #     hal.rotPIDToggle = False

        # self.table.putNumber("z_PID Setpoint", hal.rotPIDsetpoint)
        self.table.putBoolean("z_Absolute Drive", self.fieldOriented)

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
            rotFinal = self.driveRotation  # copied from HCPA code

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

        swerveModuleStates[0].optimize(Rotation2d(self.turnPosFL))

        FLModuleState = swerveModuleStates[0]

        hal.driveFLSetpoint = FLModuleState.speed
        self.turnMotorFL.getClosedLoopController().setReference(swerveModuleStates[0].angle.radians())

        swerveModuleStates[1].optimize(Rotation2d(self.turnMotorFR))

        FRModuleState = swerveModuleStates[1]

        hal.driveFRSetpoint = FRModuleState.speed
        hal.turnFRSetpoint = FRModuleState.angle.radians()

        swerveModuleStates[2].optimize(Rotation2d(self.turnMotorBL))
        BLModuleState = swerveModuleStates[2]

        hal.driveBLSetpoint = BLModuleState.speed
        hal.turnBLSetpoint = BLModuleState.angle.radians()

        swerveModuleStates[3].optimize(Rotation2d(self.turnPosBR))
        BRModuleState = swerveModuleStates[3]

        hal.driveBRSetpoint = BRModuleState.speed
        hal.turnBRSetpoint = BRModuleState.angle.radians()

class SwerveModuleController:
    def __init__(
            self,
            driveMotor: SparkMax,
            drivePID: PIDController,
            turnMotor: SparkMax,
            turnPID: PIDController
    ) -> None:
        
