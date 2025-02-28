# imports
from robotHAL import RobotHALBuffer
from robotHAL import RobotHAL
from ntcore import NetworkTableInstance
from robotHAL import RobotHAL
from rev import (
    SparkMax,
    ClosedLoopSlot,
)
import wpilib

from enum import Enum


class ElevatorMode(Enum):
    MANUAL_MODE = 0
    POSITION_MODE = 1


class ElevatorSubsystem:

    INTAKE_POS = 0
    L2_POS = 11.71
    L3_POS = 24.59
    L4_POS = 45

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("Elevator setpoint offset", 0)
        self.table.putNumber("Elevator arbFF offset", 0)

        self.velSetpoint = 0
        self.posSetpoint = 0

        self.mode = ElevatorMode.POSITION_MODE
        self.debugMode = True

    def update(
        self,
        hal: RobotHALBuffer,
        up: float,
        down: float,
        toggleMode: bool,
        POVSetpoint: float,
        armUp: bool,
        armDown: bool,
    ):
        # Dead-Zone
        if up < 0.1:
            up = 0
        if down < 0.1:
            down = 0
        if self.debugMode:
            self.table.putNumber("Elevator up", up)
            self.table.putNumber("Elevator down", down)
        if toggleMode:
            if self.mode == ElevatorMode.MANUAL_MODE:
                self.mode = ElevatorMode.POSITION_MODE
                self.posSetpoint = hal.elevatorPos
                self.velSetpoint = 0
            elif self.mode == ElevatorMode.POSITION_MODE:
                self.mode = ElevatorMode.MANUAL_MODE
                self.velSetpoint = 0
                self.posSetpoint = 0
        hal.elevatorSetpoint = self.velSetpoint + self.table.getNumber(
            "Elevator setpoint offset", 0
        )

        if self.debugMode:
            self.table.putNumber("Elevator State", self.mode.value)

        if self.mode == ElevatorMode.POSITION_MODE:
            hal.elevatorControl = SparkMax.ControlType.kPosition
            hal.elevatorSlot = ClosedLoopSlot.kSlot0

            if POVSetpoint == 180:
                self.posSetpoint = self.INTAKE_POS
            elif POVSetpoint == 90:
                self.posSetpoint = self.L2_POS
            elif POVSetpoint == 270:
                self.posSetpoint = self.L3_POS
            elif POVSetpoint == 0:
                self.posSetpoint = self.L4_POS
            hal.elevatorSetpoint = self.posSetpoint + self.table.getNumber(
                "Elevator setpoint offset", 0
            )

        elif self.mode == ElevatorMode.MANUAL_MODE:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionVelocityControl
            hal.elevatorSlot = ClosedLoopSlot.kSlot1
            # velocity logic on bottom and top
            self.velSetpoint = 90 * up + (-90 * down)  # moves the elevator

            if armUp:
                hal.armVolts = 1
            elif armDown:
                hal.armVolts = -1

        if hal.firstManipulatorSensor:
            self.posSetpoint = hal.elevatorPos

        if self.debugMode:
            self.table.putNumber("Elevator Setpoint(e)", hal.elevatorSetpoint)
            self.table.putNumber("Elevator Pos Setpoint", self.posSetpoint)
            self.table.putNumber("Elevator Vel Setpoint", self.velSetpoint)
            self.table.putString("Elevator State", self.mode.name)
        hal.elevatorArbFF = 0.5 + self.table.getNumber("Elevator arbFF offset", 0)
