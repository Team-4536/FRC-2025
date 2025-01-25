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

        self.driveVolts = 0
        self.driveDesired = 0
        self.turnDesired = 0

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

        self.FRDrivePID = RevPID(
            "FR Drive",
            self.driveMotorFR,
            driveMotorPIDConfig,
            SparkMax.ControlType.kMAXMotionVelocityControl,
        )
        self.FLDrivePID = RevPID(
            "FL Drive",
            self.driveMotorFL,
            driveMotorPIDConfig,
            SparkMax.ControlType.kMAXMotionVelocityControl,
        )
        self.BRDrivePID = RevPID(
            "BR Drive",
            self.driveMotorBR,
            driveMotorPIDConfig,
            SparkMax.ControlType.kMAXMotionVelocityControl,
        )
        self.BLDrivePID = RevPID(
            "BL Drive",
            self.driveMotorBL,
            driveMotorPIDConfig,
            SparkMax.ControlType.kMAXMotionVelocityControl,
        )

        turnMotorConfig = SparkMaxConfig()

        turnMotorConfig.closedLoop.pidf(0.3, 0.001, 0.01, 0).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)

        turnMotorConfig.closedLoop.maxMotion.maxVelocity(
            10000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(20000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(0.1)

        self.FLTurnPID = RevPID(
            "FL Turn",
            self.turnMotorFL,
            turnMotorConfig,
            SparkMax.ControlType.kMAXMotionPositionControl,
        )
        self.FRTurnPID = RevPID(
            "FR Turn",
            self.turnMotorFR,
            turnMotorConfig,
            SparkMax.ControlType.kMAXMotionPositionControl,
        )
        self.BLTurnPID = RevPID(
            "BL Turn",
            self.turnMotorBL,
            turnMotorConfig,
            SparkMax.ControlType.kMAXMotionPositionControl,
        )
        self.BRTurnPID = RevPID(
            "BR Turn",
            self.turnMotorBR,
            turnMotorConfig,
            SparkMax.ControlType.kMAXMotionPositionControl,
        )

        self.desiredDriveSpeed = 0
        self.table.putNumber("desired Drive speed", self.desiredDriveSpeed)

        self.desirePosition = 0
        self.table.putNumber("desired Turn position", self.desiredDriveSpeed)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        self.desiredDriveSpeed = self.table.getNumber(
            "desired Drive speed", self.desiredDriveSpeed
        )

        self.desirePosition = self.table.getNumber(
            "desired Turn position", self.desirePosition
        )

        self.FLDrivePID.update(self.desiredDriveSpeed)
        self.FRDrivePID.update(self.desiredDriveSpeed)
        self.BLDrivePID.update(self.desiredDriveSpeed)
        self.BRDrivePID.update(self.desiredDriveSpeed)

        self.FLTurnPID.update(self.desirePosition)
        self.FRTurnPID.update(self.desirePosition)
        self.BLTurnPID.update(self.desirePosition)
        self.BRTurnPID.update(self.desirePosition)


class RevPID:
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
