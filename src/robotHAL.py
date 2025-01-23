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


class RobotHALBuffer:
    def __init__(self) -> None:

        self.driveVolts = 0

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

        # self.driveMotorFL.setVoltage(buf.driveVolts*1.6)
        # self.table.putNumber('hal drive volts', buf.driveVolts)

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
        if (
            abs(self.table.getNumber("P", self.MotorP) - self.MotorP) < 1e-6
            or abs(self.table.getNumber("FF", self.MotorFF) - self.MotorFF) < 1e-6
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

        def __init__(self) -> None:
            self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        def __periodic_(self) -> None:
            pass

        def update(self) -> None:
            pass
