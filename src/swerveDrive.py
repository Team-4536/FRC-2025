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
from wpimath.units import inchesToMeters, radians
from ntcore import NetworkTableInstance
from wpimath.units import feetToMeters
from ntcore import NetworkTableInstance
import rev
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkClosedLoopController,
    ClosedLoopConfig,
    ClosedLoopSlot,
    LimitSwitchConfig,
    MAXMotionConfig,
    SparkBaseConfig,
)
import navx
from enum import Enum
from phoenix6.hardware import CANcoder

# from math import radians


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
WHEEL_CIRCUMFRENCE = 0.1016 * math.pi
TURN_GEARING = 21.4
DRIVE_GEARING = 6.12


class SetPointType(Enum):
    DRIVE = 1
    ROTATION = 2


class SwerveDrive:
    MAX_METERS_PER_SEC = 8.0  # stolen from lastyears code

    def __init__(self) -> None:

        self.angle = Rotation2d(0)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.FMSData = NetworkTableInstance.getDefault().getTable("FMSInfo")

        self.driveMotorFL = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.driveMotorFR = SparkMax(8, SparkMax.MotorType.kBrushless)
        self.driveMotorBL = SparkMax(4, SparkMax.MotorType.kBrushless)
        self.driveMotorBR = SparkMax(6, SparkMax.MotorType.kBrushless)
        # change numbers they aren't correct

        self.turnMotorFL = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.turnMotorFR = SparkMax(7, SparkMax.MotorType.kBrushless)
        self.turnMotorBL = SparkMax(3, SparkMax.MotorType.kBrushless)
        self.turnMotorBR = SparkMax(5, SparkMax.MotorType.kBrushless)

        self.turnMotorFLEncoder = self.turnMotorFL.getEncoder()
        self.turnMotorFREncoder = self.turnMotorFR.getEncoder()
        self.turnMotorBLEncoder = self.turnMotorBL.getEncoder()
        self.turnMotorBREncoder = self.turnMotorBR.getEncoder()
        self.turnMotorFLEncoder.setPosition(
            CANcoder(21).get_absolute_position().value * TURN_GEARING
        )
        self.turnMotorFREncoder.setPosition(
            CANcoder(24).get_absolute_position().value * TURN_GEARING
        )
        self.turnMotorBLEncoder.setPosition(
            CANcoder(22).get_absolute_position().value * TURN_GEARING
        )
        self.turnMotorBREncoder.setPosition(
            CANcoder(23).get_absolute_position().value * TURN_GEARING
        )

        self.turnPosFL = self.turnMotorFLEncoder.getPosition() * math.tau
        self.turnPosFR = self.turnMotorFREncoder.getPosition() * math.tau
        self.turnPosBL = self.turnMotorBLEncoder.getPosition() * math.tau
        self.turnPosBR = self.turnMotorBREncoder.getPosition() * math.tau

        # returns rotations

        oneftInMeters = inchesToMeters(11)

        frontLeftLocation = Translation2d(oneftInMeters, oneftInMeters)
        frontRightLocation = Translation2d(oneftInMeters, -oneftInMeters)
        backLeftLocation = Translation2d(-oneftInMeters, oneftInMeters)
        backRightLocation = Translation2d(-oneftInMeters, -oneftInMeters)
        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        )

        self.yawOffset = 0.0

        self.fieldOriented = False

        # =======NEW, NOT TUNED=======================================
        # constraints = TrapezoidProfileRadians.Constraints(4 * math.pi, 20 * math.pi)
        # xPID = PIDController(0, 0, 0)
        # yPID = PIDController(0, 0, 0)
        # rotPID = ProfiledPIDControllerRadians(1.4, 0, 0, constraints)

        # self.holonomicController = HolonomicDriveController(xPID, yPID, rotPID)

        # ============================================================

        self.driveConfig = (
            SparkMaxConfig()
            .smartCurrentLimit(40)
            .disableFollowerMode()
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            .apply(
                ClosedLoopConfig()
                .pidf(0.00019, 0, 0, 0.00002)
                .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
                .apply(
                    MAXMotionConfig()
                    .maxVelocity(2000, ClosedLoopSlot.kSlot0)
                    .maxAcceleration(50000, ClosedLoopSlot.kSlot0)
                    .allowedClosedLoopError(1)
                )
            )
        )

        self.turnConfig = (
            SparkMaxConfig()
            .smartCurrentLimit(40)
            .inverted(True)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            .apply(
                ClosedLoopConfig()
                .pidf(0.15, 0, 0, 0)
                .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
                .positionWrappingEnabled(False)
                .apply(
                    MAXMotionConfig()
                    .maxVelocity(5000, ClosedLoopSlot.kSlot0)
                    .maxAcceleration(10000, ClosedLoopSlot.kSlot0)
                    .allowedClosedLoopError(0.2)
                )
            )
        )

        configurePID(self.driveMotorFL, self.driveConfig)
        configurePID(self.driveMotorFR, self.driveConfig)
        configurePID(self.driveMotorBL, self.driveConfig)
        configurePID(self.driveMotorBR, self.driveConfig)

        configurePID(self.turnMotorFL, self.turnConfig)
        configurePID(self.turnMotorFR, self.turnConfig)
        configurePID(self.turnMotorBL, self.turnConfig)
        configurePID(self.turnMotorBR, self.turnConfig)

        table = NetworkTableInstance.getDefault()
        topic = table.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.pub = topic.publish()

    def update(
        self,
        robot,
        joystickX: float,
        joystickY: float,
        joystickRotation: float,
        RTriggerScalar: float,
        resetOffset: bool,
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

        self.table.putBoolean("feildOriented", self.fieldOriented)

        self.chassisSpeeds = ChassisSpeeds(
            driveVector.X() * 0.5 * 4**RTriggerScalar,
            driveVector.Y() * 0.5 * 4**RTriggerScalar,
            self.driveRotation,
        )

        self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
        self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
        self.table.putNumber(
            "SD ChassisSpeeds omega (rotFinal)", self.chassisSpeeds.omega
        )

        self.turnPosFL = self.turnMotorFLEncoder.getPosition() * math.tau / TURN_GEARING
        self.turnPosFR = self.turnMotorFREncoder.getPosition() * math.tau / TURN_GEARING
        self.turnPosBL = self.turnMotorBLEncoder.getPosition() * math.tau / TURN_GEARING
        self.turnPosBR = self.turnMotorBREncoder.getPosition() * math.tau / TURN_GEARING

        self.table.putNumber("Fl CANcoder", self.turnPosFL)

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

        setPoint(self.driveMotorFL, FLModuleState, SetPointType.DRIVE)
        setPoint(self.turnMotorFL, FLModuleState, SetPointType.ROTATION)

        swerveModuleStates[1].optimize(Rotation2d(self.turnPosFR))

        FRModuleState = swerveModuleStates[1]

        setPoint(self.driveMotorFR, FRModuleState, SetPointType.DRIVE)
        setPoint(self.turnMotorFR, FRModuleState, SetPointType.ROTATION)

        swerveModuleStates[2].optimize(Rotation2d(self.turnPosBL))
        BLModuleState = swerveModuleStates[2]

        setPoint(self.driveMotorBL, BLModuleState, SetPointType.DRIVE)
        setPoint(self.turnMotorBL, BLModuleState, SetPointType.ROTATION)

        swerveModuleStates[3].optimize(Rotation2d(self.turnPosBR))
        BRModuleState = swerveModuleStates[3]

        setPoint(self.driveMotorBR, BRModuleState, SetPointType.DRIVE)
        setPoint(self.turnMotorBR, BRModuleState, SetPointType.ROTATION)

        self.table.putNumber("expected FL Drive setpoint", FLModuleState.speed * 60)
        self.table.putNumber(
            "Actual turn setpoint FL",
            FLModuleState.angle.radians() / (math.tau) * TURN_GEARING,
        )

        self.pub.set(list(swerveModuleStates))


def configurePID(motor: SparkMax, config: SparkBaseConfig):

    motor.configure(
        config,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters,
    )


def setPoint(
    motor: SparkMax,
    swerveModuleState: SwerveModuleState,
    setPointType: SetPointType = SetPointType.DRIVE,
):

    controlType = SparkMax.ControlType.kMAXMotionVelocityControl
    setPoint = swerveModuleState.speed / WHEEL_CIRCUMFRENCE * DRIVE_GEARING * 60

    if setPointType == SetPointType.ROTATION:
        controlType = SparkMax.ControlType.kMAXMotionPositionControl
        setPoint = swerveModuleState.angle.radians() / (math.tau) * TURN_GEARING

    motor.getClosedLoopController().setReference(
        setPoint,
        controlType,
        ClosedLoopSlot.kSlot0,
    )
