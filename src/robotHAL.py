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

        self.desiredSpeed = 0
        self.table.putNumber("desired speed", self.desiredSpeed)
        self.MotorP = 0
        self.table.putNumber("P", self.MotorP)

        self.driveFLClosedLoopController = self.driveMotorFL.getClosedLoopController()

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

        driveFLEncoder = self.driveMotorFL.getEncoder()
        driveFLPercentVoltage = self.driveMotorFL.getAppliedOutput()
        driveFLSpeedFeedback = driveFLEncoder.getVelocity()
        driveFLPosition = driveFLEncoder.getPosition()
        driveFLVoltage = (
            self.driveMotorFL.getAppliedOutput() * self.driveMotorFL.getBusVoltage()
        )
        self.table.putNumber("DriveMotorFL voltage", driveFLVoltage)
        self.table.putNumber("DriveMotorFL Speed feedback", driveFLSpeedFeedback)
        self.table.putNumber("DriveMotorFL Position", driveFLPosition)
        self.table.putNumber("DriveMotorFL percent voltage", driveFLPercentVoltage)

        self.MotorP = self.table.getNumber("P", self.MotorP)

        driveUniversalConfig = rev.SparkBaseConfig()

        driveUniversalConfig.closedLoop.pidf(
            self.MotorP, 0, 0, 0, rev.ClosedLoopSlot.kSlot0
        )
        self.driveMotorFL.configure(
            driveUniversalConfig,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters,
        )

        self.desiredSpeed = self.table.getNumber("desired speed", self.desiredSpeed)
        self.driveFLClosedLoopController.setReference(
            self.desiredSpeed,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
        )

    class PIDs:

        def __init__(self) -> None:
            self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        def __periodic_(self) -> None:
            pass

        def update(self) -> None:
            pass
