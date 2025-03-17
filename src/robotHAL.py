import copy
import math
from wpimath import units
from real import angleWrap
import navx
from navx import AHRS
import ntcore
import rev
import wpilib
from wpilib import SerialPort
from phoenix6.hardware import CANcoder
from timing import TimeData
from ntcore import NetworkTableInstance
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkClosedLoopController,
    ClosedLoopConfig,
    ClosedLoopSlot,
    LimitSwitchConfig,
)
from wpimath.kinematics import SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.units import (
    meters_per_second,
    radians,
    rotationsToRadians,
    degreesToRadians,
)


class RobotHALBuffer:
    def __init__(self) -> None:
        self.elevatorArbFF: float = 0
        self.elevatorSetpoint: float = 0
        # Rotations
        self.elevatorPos: float = 0
        # These Values are in CCW Radians, (-pi, pi]
        self.turnCCWFL: radians = 0
        self.turnCCWFR: radians = 0
        self.turnCCWBL: radians = 0
        self.turnCCWBR: radians = 0

        self.elevatorSlot: ClosedLoopSlot = ClosedLoopSlot.kSlot0
        self.elevatorControl: SparkMax.ControlType = SparkMax.ControlType.kPosition

        self.driveFLSetpoint: meters_per_second = 0
        self.driveFRSetpoint: meters_per_second = 0
        self.driveBLSetpoint: meters_per_second = 0
        self.driveBRSetpoint: meters_per_second = 0

        self.turnFLSetpoint: radians = 0
        self.turnFRSetpoint: radians = 0
        self.turnBLSetpoint: radians = 0
        self.turnBRSetpoint: radians = 0

        self.secondManipulatorSensor: bool = False
        self.firstManipulatorSensor: bool = False
        self.manipulatorVolts: float = 0

        self.drivePositionsList: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.steerPositionList: list[float] = [0.0, 0.0, 0.0, 0.0]

        self.moduleFL = SwerveModulePosition(0, Rotation2d(radians(0)))
        self.moduleFR = SwerveModulePosition(0, Rotation2d(radians(0)))
        self.moduleBL = SwerveModulePosition(0, Rotation2d(radians(0)))
        self.moduleBR = SwerveModulePosition(0, Rotation2d(radians(0)))
        self.frontArmLimitSwitch: bool = False
        self.backArmLimitSwitch: bool = False
        self.armPos: float = 0
        self.armSetpoint: float = 0
        self.armTopLimitSwitch: bool = False
        self.armBottomLimitSwitch: bool = False

        self.elevServoAngle: float = 0.0

        self.yaw: float = 0

        self.fieldOriented: bool = True
        self.rotPIDsetpoint: int = 0
        self.rotPIDToggle: bool = False

        self.setChuteVoltage = 0
        self.chuteLimitSwitch = 0
        self.chuteMotorVoltage = 0.0

        self.chutePosition: float = 0.0
        self.resetChuteEncoder: bool = False

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        self.manipulatorVolts = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        table.putNumber("Elevator Pos(rot)", self.elevatorPos)
        table.putNumber("Turn CCW FL", self.turnCCWFL)
        table.putNumber("Turn CCW FR", self.turnCCWFR)
        table.putNumber("Turn CCW BL", self.turnCCWBL)
        table.putNumber("Turn CCW BR", self.turnCCWBR)

        table.putBoolean("Manipulator sensor 2", self.secondManipulatorSensor)
        table.putBoolean("Manipulator sensor 1", self.firstManipulatorSensor)

        table.putBoolean("Front Arm Limit Switch", self.frontArmLimitSwitch)
        table.putBoolean("Reverse Arm Limit Switch", self.backArmLimitSwitch)

        table.putNumber("yaw", self.yaw)


debugMode = False


class RobotHAL:
    def __init__(self) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.prev = RobotHALBuffer()

        global debugMode
        self.table.putBoolean("Debug Mode", debugMode)

        self.wheelRadius = 0.05  # meters

        manipulatorConfig = SparkMaxConfig()
        manipulatorConfig.limitSwitch.forwardLimitSwitchEnabled(False)
        manipulatorConfig.limitSwitch.reverseLimitSwitchEnabled(False)
        self.manipulatorMotor = SparkMax(9, SparkMax.MotorType.kBrushless)
        self.manipulatorMotor.configure(
            manipulatorConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )
        self.secondManipulatorSensor = self.manipulatorMotor.getForwardLimitSwitch()
        self.firstManipulatorSensor = self.manipulatorMotor.getReverseLimitSwitch()

        self.elevServo = wpilib.Servo(0)

        self.armMotor = SparkMax(11, rev.SparkMax.MotorType.kBrushless)
        self.armMotorEncoder = self.armMotor.getEncoder()
        self.frontArmLimitSwitch = self.armMotor.getForwardLimitSwitch()
        self.backArmLimitSwitch = self.armMotor.getReverseLimitSwitch()
        armConfig = SparkMaxConfig()
        armConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        armConfig.limitSwitch.reverseLimitSwitchEnabled(True)
        armConfig.limitSwitch.forwardLimitSwitchType(
            armConfig.limitSwitch.Type.kNormallyOpen
        )
        armConfig.limitSwitch.reverseLimitSwitchType(
            armConfig.limitSwitch.Type.kNormallyOpen
        )
        armConfig.smartCurrentLimit(20, 20)
        armConfig.closedLoop.pidf(0.08, 0, 0, 0)
        armConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        self.armTopLimitSwitch = self.armMotor.getForwardLimitSwitch()
        self.armBottomLimitSwitch = self.armMotor.getReverseLimitSwitch()

        self.armMotor.configure(
            armConfig,
            SparkMax.ResetMode.kNoResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

        self.armController = RevMotorController(
            "Arm", self.armMotor, armConfig, SparkMax.ControlType.kPosition
        )

        self.frontArmLimitSwitch = self.armMotor.getForwardLimitSwitch()
        self.backArmLimitSwitch = self.armMotor.getReverseLimitSwitch()

        self.turnMotorFL = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorFR = rev.SparkMax(3, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorBL = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorBR = rev.SparkMax(7, rev.SparkMax.MotorType.kBrushless)

        self.driveMotorFL = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorFR = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBL = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBR = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)

        self.chuteMotor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)

        chuteMotorConfig = SparkMaxConfig()
        chuteMotorConfig.IdleMode(chuteMotorConfig.IdleMode.kBrake.value)
        chuteMotorConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        chuteMotorConfig.limitSwitch.forwardLimitSwitchType(
            chuteMotorConfig.limitSwitch.Type.kNormallyClosed
        )
        chuteMotorConfig.limitSwitch.reverseLimitSwitchEnabled(True)
        chuteMotorConfig.smartCurrentLimit(15)

        self.chuteMotor.configure(
            chuteMotorConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

        self.chuteMotorLimitswitch = self.chuteMotor.getForwardLimitSwitch()

        self.driveMotorFLEncoder = self.driveMotorFL.getEncoder()
        self.driveMotorFREncoder = self.driveMotorFR.getEncoder()
        self.driveMotorBLEncoder = self.driveMotorBL.getEncoder()
        self.driveMotorBREncoder = self.driveMotorBR.getEncoder()

        self.turnMotorFLEncoder = self.turnMotorFL.getEncoder()
        self.turnMotorFREncoder = self.turnMotorFR.getEncoder()
        self.turnMotorBLEncoder = self.turnMotorBL.getEncoder()
        self.turnMotorBREncoder = self.turnMotorBR.getEncoder()

        self.chuteMotorEncoder = self.chuteMotor.getEncoder()

        self.turnMotorFLCANcoder = CANcoder(21)
        self.turnMotorFRCANcoder = CANcoder(22)
        self.turnMotorBLCANcoder = CANcoder(23)
        self.turnMotorBRCANcoder = CANcoder(24)

        driveMotorPIDConfig = SparkMaxConfig()
        driveMotorPIDConfig.smartCurrentLimit(40)
        driveMotorPIDConfig.closedLoop.pidf(0.00019, 0, 0, 0.00002).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)

        driveMotorPIDConfig.disableFollowerMode()

        driveMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            2000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(10000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(1)
        driveMotorPIDConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        turnMotorPIDConfig = SparkMaxConfig()
        turnMotorPIDConfig.smartCurrentLimit(40)
        turnMotorPIDConfig.closedLoop.pidf(0.15, 0, 0, 0).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)
        turnMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            5000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(10000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(0.2)
        turnMotorPIDConfig.closedLoop.positionWrappingEnabled(
            False
        ).positionWrappingInputRange(-math.pi, math.pi)
        turnMotorPIDConfig.inverted(True)
        turnMotorPIDConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

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

        self.turnMotorFLEncoder.setPosition(
            (self.turnMotorFLCANcoder.get_absolute_position().value_as_double) * 21.4
        )
        self.turnMotorFREncoder.setPosition(
            (self.turnMotorFRCANcoder.get_absolute_position().value_as_double) * 21.4
        )
        self.turnMotorBLEncoder.setPosition(
            (self.turnMotorBLCANcoder.get_absolute_position().value_as_double) * 21.4
        )
        self.turnMotorBREncoder.setPosition(
            (self.turnMotorBRCANcoder.get_absolute_position().value_as_double) * 21.4
        )

        self.elevatorMotor = SparkMax(10, SparkMax.MotorType.kBrushless)
        self.elevatorMotorEncoder = self.elevatorMotor.getEncoder()
        elevatorMotorPIDConfig = SparkMaxConfig()
        elevatorMotorPIDConfig.smartCurrentLimit(35)  # 20 in comp
        elevatorMotorPIDConfig.closedLoop.pidf(0.1, 0, 0, 0).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-0.7, 0.7)

        elevatorMotorPIDConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        elevatorMotorPIDConfig.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        )
        elevatorMotorPIDConfig.limitSwitch.reverseLimitSwitchEnabled(True)
        elevatorMotorPIDConfig.limitSwitch.reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyClosed
        )
        elevatorMotorPIDConfig.closedLoop.pidf(
            0.0001, 0, 0.001, 0.00211, ClosedLoopSlot.kSlot1
        )
        elevatorMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            5000, ClosedLoopSlot.kSlot1
        ).maxAcceleration(10000, ClosedLoopSlot.kSlot1).allowedClosedLoopError(
            0.05, ClosedLoopSlot.kSlot1
        )

        elevatorMotorPIDConfig.closedLoop.pidf(
            0.0001, 0, 0.001, 0.00211, ClosedLoopSlot.kSlot2
        )
        elevatorMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            2500, ClosedLoopSlot.kSlot2
        ).maxAcceleration(5000, ClosedLoopSlot.kSlot2).allowedClosedLoopError(
            0.05, ClosedLoopSlot.kSlot2
        )

        self.elevatorController = RevMotorController(
            "Elevator",
            self.elevatorMotor,
            elevatorMotorPIDConfig,
            SparkMax.ControlType.kPosition,
        )

        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kUSB1)

        self.table.putBoolean("ResetYaw", False)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        self.gyro.reset()
        self.gyro.setAngleAdjustment(-math.degrees(ang))

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.copy(buf)

        global debugMode
        debugMode = self.table.getBoolean("Debug Mode", debugMode)

        TURN_GEARING = 21.4
        buf.turnCCWFL = angleWrap(
            self.turnMotorFLEncoder.getPosition() * 2 * math.pi / TURN_GEARING
        )
        buf.turnCCWFR = angleWrap(
            self.turnMotorFREncoder.getPosition() * 2 * math.pi / TURN_GEARING
        )
        buf.turnCCWBL = angleWrap(
            self.turnMotorBLEncoder.getPosition() * 2 * math.pi / TURN_GEARING
        )
        buf.turnCCWBR = angleWrap(
            self.turnMotorBREncoder.getPosition() * 2 * math.pi / TURN_GEARING
        )

        self.FLSwerveModule.update(buf.driveFLSetpoint, buf.turnFLSetpoint)
        self.FRSwerveModule.update(buf.driveFRSetpoint, buf.turnFRSetpoint)
        self.BLSwerveModule.update(buf.driveBLSetpoint, buf.turnBLSetpoint)
        self.BRSwerveModule.update(buf.driveBRSetpoint, buf.turnBRSetpoint)

        self.table.putNumber(
            "BL Turning Pos Can",
            self.turnMotorBLCANcoder.get_absolute_position().value_as_double,
        )
        self.table.putNumber(
            "BR Turning Pos Can",
            self.turnMotorBRCANcoder.get_absolute_position().value_as_double,
        )
        self.table.putNumber(
            "FL Turning Pos Can",
            self.turnMotorFLCANcoder.get_absolute_position().value_as_double,
        )
        self.table.putNumber(
            "FR Turning Pos Can",
            self.turnMotorFRCANcoder.get_absolute_position().value_as_double,
        )

        self.table.putNumber(
            "FL Turn Encoder Raw", self.turnMotorFLEncoder.getPosition()
        )
        self.table.putNumber(
            "FR Turn Encoder Raw", self.turnMotorFREncoder.getPosition()
        )
        self.table.putNumber(
            "BL Turn Encoder Raw", self.turnMotorBLEncoder.getPosition()
        )
        self.table.putNumber(
            "BR Turn Encoder Raw", self.turnMotorBREncoder.getPosition()
        )

        self.table.putNumber(
            "FL Drive Encoder Raw", self.driveMotorFLEncoder.getPosition()
        )
        self.table.putNumber(
            "FR Drive Encoder Raw", self.driveMotorFREncoder.getPosition()
        )
        self.table.putNumber(
            "BL Drive Encoder Raw", self.driveMotorBLEncoder.getPosition()
        )
        self.table.putNumber(
            "BR Drive Encoder Raw", self.driveMotorBREncoder.getPosition()
        )

        self.table.putNumber(
            "FR Drive Vel(RPM)", self.driveMotorFREncoder.getVelocity()
        )
        self.table.putNumber(
            "FL Drive Vel(RPM)", self.driveMotorFLEncoder.getVelocity()
        )
        self.table.putNumber(
            "BR Drive Vel(RPM)", self.driveMotorBREncoder.getVelocity()
        )
        self.table.putNumber(
            "BL Drive Vel(RPM)", self.driveMotorBLEncoder.getVelocity()
        )
        self.table.putNumber("elevator servo angle", self.elevServo.getAngle())

        buf.firstManipulatorSensor = self.firstManipulatorSensor.get()
        buf.secondManipulatorSensor = self.secondManipulatorSensor.get()

        buf.yaw = degreesToRadians(-self.gyro.getAngle())

        drivePosFL = (
            (2 * math.pi)
            * (
                self.driveMotorFLEncoder.getPosition()
                / SwerveModuleController.DRIVE_GEARING
            )
            * self.wheelRadius
        )
        drivePosFR = (
            (2 * math.pi)
            * (
                self.driveMotorFREncoder.getPosition()
                / SwerveModuleController.DRIVE_GEARING
            )
            * self.wheelRadius
        )
        drivePosBL = (
            (2 * math.pi)
            * (
                self.driveMotorBLEncoder.getPosition()
                / SwerveModuleController.DRIVE_GEARING
            )
            * self.wheelRadius
        )
        drivePosBR = (
            (2 * math.pi)
            * (
                self.driveMotorBREncoder.getPosition()
                / SwerveModuleController.DRIVE_GEARING
            )
            * self.wheelRadius
        )

        buf.drivePositionsList = [drivePosFL, drivePosFR, drivePosBL, drivePosBR]

        steerPosFL = (2 * math.pi) * (
            self.turnMotorFLEncoder.getPosition() / SwerveModuleController.TURN_GEARING
        )
        steerPosFR = (2 * math.pi) * (
            self.turnMotorFREncoder.getPosition() / SwerveModuleController.TURN_GEARING
        )
        steerPosBL = (2 * math.pi) * (
            self.turnMotorBLEncoder.getPosition() / SwerveModuleController.TURN_GEARING
        )
        steerPosBR = (2 * math.pi) * (
            self.turnMotorBREncoder.getPosition() / SwerveModuleController.TURN_GEARING
        )

        buf.steerPositionList = [steerPosFL, steerPosFR, steerPosBL, steerPosBR]

        buf.moduleFL = SwerveModulePosition(drivePosFL, Rotation2d(radians(steerPosFL)))
        buf.moduleFR = SwerveModulePosition(drivePosFR, Rotation2d(radians(steerPosFR)))
        buf.moduleBL = SwerveModulePosition(drivePosBL, Rotation2d(radians(steerPosBL)))
        buf.moduleBR = SwerveModulePosition(drivePosBR, Rotation2d(radians(steerPosBR)))
        self.elevServo.setAngle(buf.elevServoAngle)

        self.elevatorController.update(
            buf.elevatorSetpoint,
            buf.elevatorArbFF,
            buf.elevatorSlot,
            buf.elevatorControl,
        )
        self.manipulatorMotor.setVoltage(buf.manipulatorVolts)

        buf.elevatorPos = self.elevatorMotorEncoder.getPosition()

        buf.yaw = math.radians(-self.gyro.getAngle())

        self.armController.update(buf.armSetpoint, 0)

        buf.backArmLimitSwitch = self.backArmLimitSwitch.get()
        buf.frontArmLimitSwitch = self.frontArmLimitSwitch.get()
        buf.armPos = self.armMotorEncoder.getPosition()

        buf.armTopLimitSwitch = self.armTopLimitSwitch.get()
        buf.armBottomLimitSwitch = self.armBottomLimitSwitch.get()

        self.table.putBoolean("Arm Top Limit Switch", buf.armTopLimitSwitch)
        self.table.putBoolean("Arm Bottom Limit Switch", buf.armBottomLimitSwitch)
        self.table.putNumber(
            "Arm Voltage",
            self.armMotor.getAppliedOutput() * self.armMotor.getBusVoltage(),
        )
        self.table.putNumber("Arm Pos", self.armMotorEncoder.getPosition())
        self.table.putNumber("Arm Setpoint", buf.armSetpoint)

        buf.chuteMotorVoltage = (
            self.chuteMotor.getAppliedOutput() * self.chuteMotor.getBusVoltage()
        )
        buf.chuteLimitSwitch = self.chuteMotorLimitswitch.get()
        self.chuteMotor.setVoltage(buf.setChuteVoltage)
        buf.chutePosition = self.chuteMotorEncoder.getPosition()

        if buf.resetChuteEncoder:
            self.chuteMotorEncoder.setPosition(0)
            buf.resetChuteEncoder = False

        if self.table.getBoolean("ResetYaw", False):
            self.resetGyroToAngle(0)


class SwerveModuleController:
    WHEEL_RADIUS = 0.05  # in meters
    DRIVE_GEARING = 6.12
    TURN_GEARING = 21.4

    def __init__(
        self,
        name: str,
        driveMotor: SparkMax,
        driveConfig: SparkMaxConfig,
        turnMotor: SparkMax,
        turnConfig: SparkMaxConfig,
    ) -> None:
        self.name = name
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.driveMotor = RevMotorController(
            "Drive " + name,
            driveMotor,
            driveConfig,
            SparkMax.ControlType.kMAXMotionVelocityControl,
        )
        self.turnMotor = RevMotorController(
            "Turn " + name,
            turnMotor,
            turnConfig,
            SparkMax.ControlType.kMAXMotionPositionControl,
        )

    def update(self, driveSetpoint: meters_per_second, turnSetpoint: radians) -> None:
        self.table.putNumber(self.name + " Drive Setpoint (m/s)", driveSetpoint)
        self.table.putNumber(self.name + " Turn Setpoint (rads)", turnSetpoint)
        # converts to rotations
        self.driveMotor.update(
            driveSetpoint * self.DRIVE_GEARING * 60 / (2 * math.pi * self.WHEEL_RADIUS),
            0,
        )
        # converts to Rotations
        self.table.putNumber(self.name + "Before unwrap", turnSetpoint)
        newTurnSetpoint = self.unwrap(
            self.turnMotor.encoder.getPosition(), turnSetpoint
        )
        self.table.putNumber(self.name + "After unwrap", newTurnSetpoint)
        self.turnMotor.update(newTurnSetpoint, 0)

    def unwrap(self, motorPos: float, targetPos: radians) -> radians:
        motorPosRads = (motorPos / self.TURN_GEARING) * 2 * math.pi
        unwrappedPos = angleWrap(motorPosRads % (2 * math.pi))

        distance: radians = angleWrap(targetPos - unwrappedPos)

        return (motorPosRads + distance) * self.TURN_GEARING / (2 * math.pi)


class RevMotorController:
    def __init__(
        self,
        name: str,
        motor: SparkMax,
        config: SparkMaxConfig,
        controlType: SparkMax.ControlType,
    ) -> None:
        self.name = name
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.motor = motor
        self.encoder = motor.getEncoder()
        self.controller = motor.getClosedLoopController()
        self.config = SparkMaxConfig()
        self.config.apply(config)
        self.controlType: SparkMax.ControlType = controlType
        self.setpoint = 0.0

        self.motor.configure(
            self.config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

        self.PIDValues = {
            "kP": self.motor.configAccessor.closedLoop.getP(),
            "kI": self.motor.configAccessor.closedLoop.getI(),
            "kD": self.motor.configAccessor.closedLoop.getD(),
            "kFF": self.motor.configAccessor.closedLoop.getFF(),
        }

        for key, value in zip(self.PIDValues.keys(), self.PIDValues.values()):
            self.table.putNumber(name + key, value)

    def update(
        self,
        setpoint: float,
        arbFF: float,
        slot: ClosedLoopSlot = ClosedLoopSlot.kSlot0,
        controlType: SparkMax.ControlType | None = None,
    ) -> None:

        changeError = 1e-6
        reconfigureFlag = False
        global debugMode
        if debugMode:
            for key in self.PIDValues.keys():
                if (
                    abs(
                        self.PIDValues[key]
                        - self.table.getNumber(self.name + key, self.PIDValues[key])
                    )
                    > changeError
                ):
                    reconfigureFlag = True
                self.PIDValues[key] = self.table.getNumber(
                    self.name + key, self.PIDValues[key]
                )

            if reconfigureFlag:
                self.config.closedLoop.pidf(
                    self.PIDValues["kP"],
                    self.PIDValues["kI"],
                    self.PIDValues["kD"],
                    self.PIDValues["kFF"],
                )

                self.motor.configure(
                    self.config,
                    SparkMax.ResetMode.kResetSafeParameters,
                    SparkMax.PersistMode.kNoPersistParameters,
                )

        measuredPercentVoltage = self.motor.getAppliedOutput()
        measuredSpeed = self.encoder.getVelocity()
        measuredPosition = -self.encoder.getPosition()
        measuredVoltage = self.motor.getAppliedOutput() * self.motor.getBusVoltage()
        measuredAmps = self.motor.getOutputCurrent()
        self.table.putNumber(self.name + " Voltage", measuredVoltage)
        self.table.putNumber(self.name + " Velocity (RPM)", measuredSpeed)
        self.table.putNumber(self.name + " Position (rot)", measuredPosition)
        self.table.putNumber(self.name + " percent voltage", measuredPercentVoltage)
        self.table.putNumber(self.name + " current", measuredAmps)
        self.setpoint = setpoint
        self.table.putNumber(self.name + " setpoint", self.setpoint)

        if controlType == None:
            controlType = self.controlType

        self.controller.setReference(
            setpoint,
            controlType,
            slot,
            arbFF,
            SparkClosedLoopController.ArbFFUnits.kVoltage,
        )
