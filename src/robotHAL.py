import copy
import math
from real import angleWrap
import navx
import ntcore
import rev
import wpilib
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
from wpimath.units import meters_per_second, radians


class RobotHALBuffer:
    def __init__(self) -> None:
        self.elevatorArbFF: float = 0
        self.elevatorSetpoint: float = 0
        # Rotations
        self.elevatorPos: float = 0
        # These Values are in CCW Radians, (-pi, pi]
        self.turnPosFL: radians = 0
        self.turnPosFR: radians = 0
        self.turnPosBL: radians = 0
        self.turnPosBR: radians = 0

        self.driveFLSetpoint: meters_per_second = 0
        self.driveFRSetpoint: meters_per_second = 0
        self.driveBLSetpoint: meters_per_second = 0
        self.driveBRSetpoint: meters_per_second = 0

        self.turnFLSetpoint: radians = 0
        self.turnFRSetpoint: radians = 0
        self.turnBLSetpoint: radians = 0
        self.turnBRSetpoint: radians = 0

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        self.driveVolts = 0
        self.driveDesired = 0
        self.turnDesired = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        table.putNumber("Elevator Rotations", self.elevatorPos)

        table.putNumber("Turn Pos FL", self.turnPosFL)
        table.putNumber("Turn Pos FR", self.turnPosFR)
        table.putNumber("Turn Pos BL", self.turnPosBL)
        table.putNumber("Turn Pos BR", self.turnPosBR)


debugMode = True


class RobotHAL:
    def __init__(self) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.prev = RobotHALBuffer()

        self.elevatorMotor = SparkMax(10, SparkMax.MotorType.kBrushless)
        elevatorMotorPIDConfig = SparkMaxConfig()
        elevatorMotorPIDConfig.smartCurrentLimit(25)  # 20 in comp
        elevatorMotorPIDConfig.closedLoop.pidf(0.1, 0, 0, 0).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1, 1)
        elevatorMotorPIDConfig.closedLoop.maxMotion.maxVelocity(5000).maxAcceleration(
            10000
        ).allowedClosedLoopError(1)

        elevatorMotorPIDConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        elevatorMotorPIDConfig.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        )
        elevatorMotorPIDConfig.limitSwitch.reverseLimitSwitchEnabled(True)
        elevatorMotorPIDConfig.limitSwitch.reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyClosed
        )
        self.elevatorController = RevMotorController(
            "Elevator",
            self.elevatorMotor,
            elevatorMotorPIDConfig,
            SparkMax.ControlType.kMAXMotionPositionControl,
        )

        global debugMode
        self.table.putBoolean("Debug Mode", debugMode)

        self.turnMotorFL = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorFR = rev.SparkMax(3, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorBL = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorBR = rev.SparkMax(7, rev.SparkMax.MotorType.kBrushless)

        self.driveMotorFL = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorFR = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBL = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBR = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)

        self.driveMotorFLEncoder = self.driveMotorFL.getEncoder()
        self.driveMotorFREncoder = self.driveMotorFR.getEncoder()
        self.driveMotorBLEncoder = self.driveMotorBL.getEncoder()
        self.driveMotorBREncoder = self.driveMotorBR.getEncoder()

        self.turnMotorFLEncoder = self.turnMotorFL.getEncoder()
        self.turnMotorFREncoder = self.turnMotorFR.getEncoder()
        self.turnMotorBLEncoder = self.turnMotorBL.getEncoder()
        self.turnMotorBREncoder = self.turnMotorBR.getEncoder()

        self.turnMotorFLCANcoder = CANcoder(21)
        self.turnMotorFRCANcoder = CANcoder(22)
        self.turnMotorBLCANcoder = CANcoder(23)
        self.turnMotorBRCANcoder = CANcoder(24)

        self.turnMotorFLEncoder.setPosition(
            self.turnMotorFLCANcoder.get_absolute_position().value_as_double
        )
        self.turnMotorFREncoder.setPosition(
            self.turnMotorFRCANcoder.get_absolute_position().value_as_double
        )
        self.turnMotorBLEncoder.setPosition(
            self.turnMotorBLCANcoder.get_absolute_position().value_as_double
        )
        self.turnMotorBREncoder.setPosition(
            self.turnMotorBRCANcoder.get_absolute_position().value_as_double
        )

        driveMotorPIDConfig = SparkMaxConfig()
        driveMotorPIDConfig.smartCurrentLimit(40)
        driveMotorPIDConfig.closedLoop.pidf(0.00019, 0, 0, 0.00002).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)

        driveMotorPIDConfig.disableFollowerMode()

        driveMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            2000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(4000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(1)

        turnMotorPIDConfig = SparkMaxConfig()
        turnMotorPIDConfig.smartCurrentLimit(40)
        turnMotorPIDConfig.closedLoop.pidf(0.25, 0, 0.01, 0).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)
        turnMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            5000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(10000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(0.1)
        turnMotorPIDConfig.closedLoop.positionWrappingEnabled(
            True
        ).positionWrappingInputRange(-math.pi, math.pi)

        # FL 135
        self.FLSwerveModule = SwerveModuleController(
            "FL",
            self.driveMotorFL,
            driveMotorPIDConfig,
            self.turnMotorFL,
            turnMotorPIDConfig,
        )
        # FR 45
        self.FRSwerveModule = SwerveModuleController(
            "FR",
            self.driveMotorFR,
            driveMotorPIDConfig,
            self.turnMotorFR,
            turnMotorPIDConfig,
        )
        # BL -45
        self.BLSwerveModule = SwerveModuleController(
            "BL",
            self.driveMotorBL,
            driveMotorPIDConfig,
            self.turnMotorBL,
            turnMotorPIDConfig,
        )
        # BR -135
        self.BRSwerveModule = SwerveModuleController(
            "BR",
            self.driveMotorBR,
            driveMotorPIDConfig,
            self.turnMotorBR,
            turnMotorPIDConfig,
        )

        self.table.putNumber("BL Turn Offset", 0)
        self.table.putNumber("BR Turn Offset", 0)
        self.table.putNumber("FL Turn Offset", 0)
        self.table.putNumber("FR Turn Offset", 0)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        global debugMode
        debugMode = self.table.getBoolean("Debug Mode", debugMode)

        self.elevatorController.update(buf.elevatorSetpoint, buf.elevatorArbFF)

        TURN_GEARING = 21.4
        buf.turnPosFL = angleWrap(
            self.turnMotorFLEncoder.getPosition() * 2 * math.pi / TURN_GEARING
        )
        buf.turnPosFR = angleWrap(
            self.turnMotorFREncoder.getPosition() * 2 * math.pi / TURN_GEARING
        )
        buf.turnPosBL = angleWrap(
            self.turnMotorBLEncoder.getPosition() * 2 * math.pi / TURN_GEARING
        )
        buf.turnPosBR = angleWrap(
            self.turnMotorBREncoder.getPosition() * 2 * math.pi / TURN_GEARING
        )

        self.FLSwerveModule.update(
            buf.driveFLSetpoint,
            buf.turnFLSetpoint + self.table.getNumber("FL Turn Offset", 0),
        )
        self.FRSwerveModule.update(
            buf.driveFRSetpoint,
            buf.turnFRSetpoint + self.table.getNumber("FR Turn Offset", 0),
        )
        self.BLSwerveModule.update(
            buf.driveBLSetpoint,
            buf.turnBLSetpoint + self.table.getNumber("BL Turn Offset", 0),
        )
        self.BRSwerveModule.update(
            buf.driveBRSetpoint,
            buf.turnBRSetpoint + self.table.getNumber("BR Turn Offset", 0),
        )


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
            driveSetpoint
            * self.DRIVE_GEARING
            * 60
            / (2 * math.pi * self.WHEEL_RADIUS, 0)
        )
        # converts to Rotations
        self.turnMotor.update(-turnSetpoint * self.TURN_GEARING / (2 * math.pi), 0)


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
        self.setpoint = 0

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

    def update(self, setpoint: float, arbFF: float) -> None:

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
                    SparkMax.ResetMode.kNoResetSafeParameters,
                    SparkMax.PersistMode.kNoPersistParameters,
                )

        measuredPercentVoltage = self.motor.getAppliedOutput()
        measuredSpeed = self.encoder.getVelocity()
        measuredPosition = self.encoder.getPosition()
        measuredVoltage = self.motor.getAppliedOutput() * self.motor.getAppliedOutput()
        measuredAmps = self.motor.getOutputCurrent()
        self.table.putNumber(self.name + " Voltage", measuredVoltage)
        self.table.putNumber(self.name + " Velocity (RPM)", measuredSpeed)
        self.table.putNumber(self.name + " Position (rot)", measuredPosition)
        self.table.putNumber(self.name + " percent voltage", measuredPercentVoltage)
        self.table.putNumber(self.name + " current", measuredAmps)
        self.setpoint = setpoint
        self.table.putNumber(self.name + " setpoint", self.setpoint)

        self.controller.setReference(
            setpoint,
            self.controlType,
            ClosedLoopSlot.kSlot0,
            arbFF,
            SparkClosedLoopController.ArbFFUnits.kVoltage,
        )
