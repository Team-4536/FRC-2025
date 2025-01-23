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
from rev import SparkMax, SparkClosedLoopController


class RobotHALBuffer:
    def __init__(self) -> None:
        pass

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

        self.driveMotorFL = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorFR = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBL = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBR = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        # max velocity (RPM), max acceleration (RPM / s), allowed closed loop error, minimum output velocity,
        self.desiredSpeed = 0
        self.table.putNumber("desired speed", self.desiredSpeed)
        self.MotorP = 0.00019
        self.table.putNumber("P", self.MotorP)
        self.MotorFF = 0.00002
        self.table.putNumber("FF", self.MotorFF)

        self.maxVelocity = 2000
        self.maxAcceleration = 4000
        self.closedLoopError = 1

        customConfig = rev.SparkBaseConfig()
        customConfig.closedLoop.pidf(
            self.MotorP, 0, 0, self.MotorFF, rev.ClosedLoopSlot.kSlot0
        ).setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(
            -1.0, 1.0, rev.ClosedLoopSlot.kSlot0
        )

        customConfig.closedLoop.maxMotion.maxVelocity(
            self.maxVelocity, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(
            self.maxAcceleration, rev.ClosedLoopSlot.kSlot0
        ).allowedClosedLoopError(
            1
        )

        error = self.driveMotorFL.configure(
            customConfig,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters,
        )

        self.table.putNumber("Config err", error)

        self.driveFLClosedLoopController = self.driveMotorFL.getClosedLoopController()
        self.driveFLEncoder = self.driveMotorFL.getEncoder()

        self.profilingStart = 0

    def startProfile(self):
        self.profilingStart = wpilib.getTime()

    def endProfile(self, name: str):
        self.table.putNumber(name + " time", wpilib.getTime() - self.profilingStart)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        self.startProfile()

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

        self.endProfile("Getting motor values")

        # self.MotorP = self.table.getNumber("P", self.MotorP)
        # self.MotorFF = self.table.getNumber("FF", self.MotorFF)

        driveUniversalConfig = rev.SparkBaseConfig()

        self.startProfile()
        # var1 = abs(self.table.getNumber("P", self.MotorP) - self.MotorP) < 1e-6
        # var2 = abs(self.table.getNumber("FF", self.MotorFF) - self.MotorFF) < 1e-6
        # self.table.putNumber()
        if (
            abs(self.table.getNumber("P", self.MotorP) - self.MotorP) > 1e-6
            or abs(self.table.getNumber("FF", self.MotorFF) - self.MotorFF) > 1e-6
        ):
            self.MotorP = self.table.getNumber("P", self.MotorP)
            self.MotorFF = self.table.getNumber("FF", self.MotorFF)

            driveUniversalConfig.closedLoop.pidf(
                self.MotorP, 0, 0, self.MotorFF, rev.ClosedLoopSlot.kSlot0
            ).setFeedbackSensor(
                rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
            ).outputRange(
                -1.0, 1.0, rev.ClosedLoopSlot.kSlot0
            )

            driveUniversalConfig.encoder.positionConversionFactor(
                1
            ).velocityConversionFactor(1)

            error = self.driveMotorFL.configure(
                driveUniversalConfig,
                rev.SparkBase.ResetMode.kNoResetSafeParameters,
                rev.SparkBase.PersistMode.kNoPersistParameters,
            )

            self.table.putNumber("Config err", error.value)

        self.endProfile("rev motor controller config")

        self.startProfile()

        self.desiredSpeed = self.table.getNumber("desired speed", self.desiredSpeed)
        self.driveFLClosedLoopController.setReference(
            self.desiredSpeed,
            rev.SparkBase.ControlType.kMAXMotionVelocityControl,
            rev.ClosedLoopSlot.kSlot0,
        )

        self.endProfile("Set reference")

        # self.table.putNumber("read p value", driveUniversalConfig.closedLoop.)


class PIDs:
    def __init__(
        self,
        name: str,
        motorRefrence: SparkMax,
        controlMode: SparkMax.ControlType,
        config: rev.SparkBaseConfig,
    ) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.motorReference: SparkMax = motorRefrence
        self.closedLoopController: SparkClosedLoopController = (
            self.motorReference.getClosedLoopController()
        )

        self.controlMode: SparkMax.ControlType = controlMode
        self.setpiont = 0

        self.motorReference.configure(
            config,
            rev.SparkMax.ResetMode.kNoResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )

        self.config = config

        # self.kP = self.motorReference.configAccessor.closedLoop.getP()
        # self.kI = self.motorReference.configAccessor.closedLoop.getI()
        # self.kD = self.motorReference.configAccessor.closedLoop.getD()
        # self.kFF = self.motorReference.configAccessor.closedLoop.getFF()

        # self.table.putNumber(self.name + "kP", self.kP)
        # self.table.putNumber(self.name + "kI", self.kI)
        # self.table.putNumber(self.name + "kD", self.kD)
        # self.table.putNumber(self.name + "kFF", self.kFF)

        # the key will be used for values on the network table
        # not that all network table values will be labeled self.name + key
        self.configurationVars: dict = {
            "kP": self.motorReference.configAccessor.closedLoop.getP(),
            "kI": self.motorReference.configAccessor.closedLoop.getI(),
            "kD": self.motorReference.configAccessor.closedLoop.getD(),
            "kFF": self.motorReference.configAccessor.closedLoop.getFF(),
        }

        for key, value in zip(
            self.configurationVars.keys(), self.configurationVars.values()
        ):
            self.table.putNumber(self.name + key, value)

    def liveConfig(self) -> None:
        tuneErr = 0.00001

        reconfigureFlag = False

        for key, value in zip(
            self.configurationVars.keys(), self.configurationVars.values()
        ):
            if abs(value - self.table.getNumber(self.name + key, value)) > tuneErr:
                reconfigureFlag = True
                break

        if reconfigureFlag:
            self.config.closedLoop.pidf(
                self.table.getNumber(self.name + "kP"),
                self.table.getNumber(self.name + "kI"),
                self.table.getNumber(self.name + "kD"),
                self.table.getNumber(self.name + "kFF"),
            )

            self.motorReference.configure(self.config)

        # if (
        #     abs(self.table.getNumber(self.name + "kP") - self.kP) > tuneErr
        #     or abs(self.table.getNumber(self.name + "kI") - self.kI) > tuneErr
        #     or abs(self.table.getNumber(self.name + "kD") - self.kD) > tuneErr
        #     or abs(self.table.getNumber(self.name + "kFF") - self.kFF) > tuneErr
        # ):
        #     self.config.closedLoop.pidf(
        #         self.table.getNumber(self.name + "kP"),
        #         self.table.getNumber(self.name + "kI"),
        #         self.table.getNumber(self.name + "kD"),
        #         self.table.getNumber(self.name + "kFF"),
        #     )

        #     self.motorReference.configure(self.config)

    def update(self, setpoint: float) -> None:
        self.closedLoopController.setReference(setpoint, self.controlMode)
        self.setpiont = setpoint

    def publish(self) -> None:
        self.table.putNumber(self.name + " setpoint", self.setpiont)
