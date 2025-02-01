import copy
import math

import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
from timing import TimeData
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkMaxConfigAccessor,
    BaseConfig,
    ClosedLoopSlot,
    SparkClosedLoopController,
    ClosedLoopConfig,
    LimitSwitchConfig,
)
from ntcore import NetworkTableInstance


class RobotHALBuffer:
    def __init__(self) -> None:
        self.elevatorArbFF: float = 0
        self.elevatorSetpoint: float = 0
        # Rotations
        self.elevatorPos: float = 0

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        table.putNumber("Elevator Rotations", self.elevatorPos)


debugMode = True


class RobotHAL:
    def __init__(self) -> None:
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

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        self.elevatorController.update(buf.elevatorSetpoint, buf.elevatorArbFF)


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
