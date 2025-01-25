import copy
import math
import robot


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
)


class RobotHALBuffer:
    def __init__(self) -> None:
        self.driveFLSetpoint = 0
        self.driveFRSetpoint = 0
        self.driveBLSetpoint = 0
        self.driveBRSetpoint = 0

        self.turnFLSetpoint = 0
        self.turnFRSetpoint = 0
        self.turnBLSetpoint = 0
        self.turnBRSetpoint = 0

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        self.driveVolts = 0
        self.driveDesired = 0
        self.turnDesired = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


defaultSparkMaxConfig = SparkMaxConfig()
defaultSparkMaxConfig.closedLoop.pidf(0, 0, 0, 0).IMaxAccum(1000).IZone(10).outputRange(
    -1.0, 1.0
).positionWrappingEnabled(False)
defaultSparkMaxConfig.closedLoop.maxMotion.allowedClosedLoopError(0).maxVelocity(
    2000
).maxAcceleration(4000)


class RobotHAL:
    def __init__(self) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.prev = RobotHALBuffer()

        self.turnMotorFL = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorFR = rev.SparkMax(3, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorBL = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushless)
        self.turnMotorBR = rev.SparkMax(7, rev.SparkMax.MotorType.kBrushless)

        self.driveMotorFL = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorFR = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBL = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBR = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)

        driveMotorPIDConfig = SparkMaxConfig()
        driveMotorPIDConfig.closedLoop.pidf(0.00019, 0, 0, 0.00002).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)

        driveMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            2000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(4000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(1)

        turnMotorPIDConfig = SparkMaxConfig()

        turnMotorPIDConfig.closedLoop.pidf(0.3, 0.001, 0.01, 0).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)

        turnMotorPIDConfig.closedLoop.maxMotion.maxVelocity(
            10000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(20000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(0.1)

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

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        self.FLSwerveModule.update(buf.driveFLSetpoint, buf.turnFLSetpoint)
        self.FLSwerveModule.update(buf.driveFRSetpoint, buf.turnFRSetpoint)
        self.FLSwerveModule.update(buf.driveBLSetpoint, buf.turnBLSetpoint)
        self.FLSwerveModule.update(buf.driveBRSetpoint, buf.turnBRSetpoint)


class SwerveModuleController:
    def __init__(
        self,
        name: str,
        driveMotor: SparkMax,
        driveConfig: SparkMaxConfig,
        turnMotor: SparkMax,
        turnConfig: SparkMaxConfig,
    ) -> None:
        self.name = name
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

    def update(self, driveSetpoint: float, turnSetpoint: float) -> None:
        self.driveMotor.update(driveSetpoint)
        self.turnMotor.update(turnSetpoint)


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

        self.PIDValues = {
            "kP": self.motor.configAccessor.closedLoop.getP(),
            "kI": self.motor.configAccessor.closedLoop.getI(),
            "kD": self.motor.configAccessor.closedLoop.getD(),
            "kFF": self.motor.configAccessor.closedLoop.getFF(),
        }

        self.motor.configure(
            self.config,
            SparkMax.ResetMode.kNoResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

        for key, value in zip(self.PIDValues.keys(), self.PIDValues.values()):
            self.table.putNumber(name + key, value)

    def update(self, setpoint: float) -> None:

        changeError = 1e-6
        reconfigureFlag = False
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
        self.table.putNumber(self.name + " Voltage", measuredVoltage)
        self.table.putNumber(self.name + " Velocity (RPM)", measuredSpeed)
        self.table.putNumber(self.name + " DriveMotorFL Position", measuredPosition)
        self.table.putNumber(
            self.name + " DriveMotorFL percent voltage", measuredPercentVoltage
        )
        self.setpoint = setpoint
        self.table.putNumber(self.name + " setpoint", self.setpoint)

        self.controller.setReference(setpoint, self.controlType, ClosedLoopSlot.kSlot0)
