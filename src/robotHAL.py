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

        self.driveMotorFL = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.turnMotorFL = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.driveMotorFR = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)

        FRDriveConfig = SparkMaxConfig()
        FRDriveConfig.closedLoop.pidf(0.00019, 0, 0, 0.00002).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(-1.0, 1.0, rev.ClosedLoopSlot.kSlot0)

        FRDriveConfig.closedLoop.maxMotion.maxVelocity(
            2000, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(4000, rev.ClosedLoopSlot.kSlot0).allowedClosedLoopError(1)
        self.FRDrivePID = RevPID(
            "FR Drive",
            self.driveMotorFR,
            FRDriveConfig,
            SparkMax.ControlType.kMAXMotionVelocityControl,
        )

        self.desiredFRDriveSpeed = 0
        self.table.putNumber("desiredFRDrive", self.desiredFRDriveSpeed)

        # self.driveMotorBL = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        # self.driveMotorBR = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)

        # max velocity (RPM), max acceleration (RPM / s), allowed closed loop error, minimum output velocity,
        self.desiredPosition = 0
        self.table.putNumber("turn setpoint", self.desiredPosition)
        self.turnMotorP = 0.3
        self.turnMotorI = 0.001
        self.turnMotorD = 0.01
        self.table.putNumber("turn P", self.turnMotorP)
        self.table.putNumber("turn I", self.turnMotorI)
        self.table.putNumber("turn D", self.turnMotorD)
        self.turnMaxVelocity = 10000
        self.turnMaxAcceleration = 20000
        self.turnClosedLoopError = 0.1

        self.desiredSpeed = 0
        self.table.putNumber("speed setpoint", self.desiredSpeed)
        self.driveMotorP = 0.00019
        self.table.putNumber("drive P", self.driveMotorP)
        self.driveMotorFF = 0.00002
        self.table.putNumber("drive FF", self.driveMotorFF)

        self.driveMaxVelocity = 2000
        self.driveMaxAcceleration = 4000
        self.driveClosedLoopError = 1

        driveMotorConfig = rev.SparkBaseConfig()
        driveMotorConfig.closedLoop.pidf(
            self.driveMotorP, 0, 0, self.driveMotorFF, rev.ClosedLoopSlot.kSlot0
        ).setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(
            -1.0, 1.0, rev.ClosedLoopSlot.kSlot0
        )

        driveMotorConfig.closedLoop.maxMotion.maxVelocity(
            self.driveMaxVelocity, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(
            self.driveMaxAcceleration, rev.ClosedLoopSlot.kSlot0
        ).allowedClosedLoopError(
            1
        )

        error = self.driveMotorFL.configure(
            driveMotorConfig,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters,
        )

        turnMotorConfig = rev.SparkMaxConfig()

        turnMotorConfig.closedLoop.pid(
            self.turnMotorP, self.turnMotorI, self.turnMotorD, rev.ClosedLoopSlot.kSlot0
        ).setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(
            -1.0, 1.0
        )

        turnMotorConfig.closedLoop.maxMotion.maxVelocity(
            self.turnMaxVelocity
        ).maxAcceleration(self.turnMaxAcceleration).allowedClosedLoopError(0.1)

        self.turnMotorFL.configure(
            turnMotorConfig,
            SparkMax.ResetMode.kNoResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

        self.driveFLClosedLoopController = self.driveMotorFL.getClosedLoopController()
        self.driveFLEncoder = self.driveMotorFL.getEncoder()

        self.turnMotorController = self.turnMotorFL.getClosedLoopController()
        self.turnFLEncoder = self.turnMotorFL.getEncoder()

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        driveFLPercentVoltage = self.driveMotorFL.getAppliedOutput()
        driveFLSpeedFeedback = self.driveFLEncoder.getVelocity()
        driveFLPosition = self.driveFLEncoder.getPosition()
        driveFLVoltage = (
            self.driveMotorFL.getAppliedOutput() * self.driveMotorFL.getBusVoltage()
        )
        self.table.putNumber("DriveMotorFL voltage", driveFLVoltage)
        self.table.putNumber("DriveMotorFL Speed feedback", driveFLSpeedFeedback)
        self.table.putNumber("DriveMotorFL Position", driveFLPosition)
        self.table.putNumber("DriveMotorFL percent voltage", driveFLPercentVoltage)

        turnFLPercentVoltage = self.turnMotorFL.getAppliedOutput()
        turnFLSpeedFeedback = self.turnFLEncoder.getVelocity()
        turnFLPosition = self.turnFLEncoder.getPosition()
        turnFLVoltage = (
            self.turnMotorFL.getAppliedOutput() * self.turnMotorFL.getBusVoltage()
        )
        self.table.putNumber("TurnMotorFL voltage", turnFLVoltage)
        self.table.putNumber("TurnMotorFL Speed feedback", turnFLSpeedFeedback)
        self.table.putNumber("TurnMotorFL Position", turnFLPosition)
        self.table.putNumber("TurnMotorFL percent voltage", turnFLPercentVoltage)

        # self.driveMotorP = self.table.getNumber("drive P", self.MotorP)
        # self.driveMotorFF = self.table.getNumber("drive FF", self.MotorFF)

        # if (
        #     abs(self.table.getNumber("drive P", self.driveMotorP) - self.MotorP) < 1e-6
        #     or abs(self.table.getNumber("drive FF", self.driveMotorFF) - self.MotorFF) < 1e-6
        # ):
        #     self.driveMotorP = self.table.getNumber("drive P", self.MotorP)
        #     self.driveMotorFF = self.table.getNumber("drive FF", self.MotorFF)

        #     driveUniversalConfig.closedLoop.pidf(
        #         self.driveMotorP, 0, 0, self.driveMotorFF, rev.ClosedLoopSlot.kSlot0
        #     ).setFeedbackSensor(
        #         rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        #     ).outputRange(
        #         -1.0, 1.0, rev.ClosedLoopSlot.kSlot0
        #     )

        #     driveUniversalConfig.encoder.positionConversionFactor(
        #         1
        #     ).velocityConversionFactor(1)

        #     error = self.driveMotorFL.configure(
        #         driveUniversalConfig,
        #         rev.SparkBase.ResetMode.kNoResetSafeParameters,
        #         rev.SparkBase.PersistMode.kNoPersistParameters,
        #     )

        #     self.table.putNumber("Config err", error.value)

        if (
            abs(self.table.getNumber("turn P", self.turnMotorP) - self.turnMotorP)
            > 1e-6
        ):
            driveUniversalConfig = rev.SparkBaseConfig()
            self.turnMotorP = self.table.getNumber("turn P", self.turnMotorP)
            self.turnMotorI = self.table.getNumber("turn I", self.turnMotorI)
            self.turnMotorD = self.table.getNumber("turn D", self.turnMotorD)

            driveUniversalConfig.closedLoop.pid(
                self.turnMotorP, 0, 0, rev.ClosedLoopSlot.kSlot0
            ).setFeedbackSensor(
                rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
            ).outputRange(
                -1.0, 1.0, rev.ClosedLoopSlot.kSlot0
            )

            driveUniversalConfig.encoder.positionConversionFactor(
                1
            ).velocityConversionFactor(1)

            error = self.turnMotorFL.configure(
                driveUniversalConfig,
                rev.SparkBase.ResetMode.kNoResetSafeParameters,
                rev.SparkBase.PersistMode.kNoPersistParameters,
            )

            self.table.putNumber("Config err", error.value)

        self.desiredSpeed = self.table.getNumber("speed setpoint", self.desiredSpeed)
        self.driveFLClosedLoopController.setReference(
            self.desiredSpeed,
            rev.SparkBase.ControlType.kMAXMotionVelocityControl,
            rev.ClosedLoopSlot.kSlot0,
        )

        self.desiredPosition = self.table.getNumber(
            "turn setpoint", self.desiredPosition
        )
        self.turnMotorController.setReference(
            self.desiredPosition,
            rev.SparkBase.ControlType.kMAXMotionPositionControl,
            rev.ClosedLoopSlot.kSlot0,
        )

        self.desiredFRDriveSpeed = self.table.getNumber(
            "desiredFRDrive", self.desiredFRDriveSpeed
        )
        self.FRDrivePID.update(self.desiredFRDriveSpeed)


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
