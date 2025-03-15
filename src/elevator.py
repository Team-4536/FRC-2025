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
    ALGAE_L2_POS = 13.78
    ALGAE_L3_POS = 24.52

    ARM_UP_POS = 28
    ARM_DEALGAE_POS = 12
    ARM_BOTTOM_POS = 0

    # this is an elevator position where it is safe for the arm to move
    ELEVATOR_CLEARS_BUMPERS_FOR_ARM = 7

    def __init__(self) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("Elevator setpoint offset", 0)
        self.table.putNumber("Elevator arbFF offset", 0)

        self.velSetpoint = 0
        self.posSetpoint = 0

        self.mode = ElevatorMode.POSITION_MODE
        self.debugMode = False

        self.moveArmDown: bool = False
        self.armReachedTop: bool = False

    def update(
        self,
        hal: RobotHALBuffer,
        up: float,
        down: float,
        toggleManualMode: bool,
        POVSetpoint: float,
        armToggle: bool,
        algaePosMode: bool,
    ):
        if armToggle:
            self.moveArmDown = not self.moveArmDown

        # Dead-Zone
        if up < 0.1:
            up = 0
        if down < 0.1:
            down = 0
        if self.debugMode:
            self.table.putNumber("Elevator up", up)
            self.table.putNumber("Elevator down", down)
        if toggleManualMode:
            if self.mode == ElevatorMode.MANUAL_MODE:
                self.mode = ElevatorMode.POSITION_MODE
                self.posSetpoint = int(hal.elevatorPos)
                self.velSetpoint = 0
            elif self.mode == ElevatorMode.POSITION_MODE:
                self.mode = ElevatorMode.MANUAL_MODE
                self.velSetpoint = 0
                self.posSetpoint = 0

        if self.debugMode:
            self.table.putNumber("Elevator State", self.mode.value)

        if self.debugMode:
            self.table.putNumber("Elevator State", self.mode.value)

        if self.mode == ElevatorMode.POSITION_MODE:
            hal.elevatorControl = SparkMax.ControlType.kPosition
            hal.elevatorSlot = ClosedLoopSlot.kSlot0

            if not algaePosMode:
                if POVSetpoint == 180:
                    self.posSetpoint = self.INTAKE_POS
                elif POVSetpoint == 90:
                    self.posSetpoint = int(self.L2_POS)
                elif POVSetpoint == 270:
                    self.posSetpoint = int(self.L3_POS)
                elif POVSetpoint == 0:
                    self.posSetpoint = self.L4_POS

            else:  # algae pos mode
                if POVSetpoint == 180:
                    self.posSetpoint = int(self.ALGAE_L2_POS)
                elif POVSetpoint == 0:
                    self.posSetpoint = int(self.ALGAE_L3_POS)

            hal.elevatorSetpoint = self.posSetpoint + self.table.getNumber(
                "Elevator setpoint offset", 0
            )

            if (
                self.posSetpoint > self.ELEVATOR_CLEARS_BUMPERS_FOR_ARM
                and hal.elevatorPos > self.ELEVATOR_CLEARS_BUMPERS_FOR_ARM
            ):
                hal.armSetpoint = self.ARM_UP_POS
                if self.moveArmDown:
                    hal.armSetpoint = self.ARM_DEALGAE_POS

            if self.posSetpoint < self.ELEVATOR_CLEARS_BUMPERS_FOR_ARM:
                hal.armSetpoint = self.ARM_BOTTOM_POS
                self.moveArmDown = False
                # if not hal.armBottomLimitSwitch:
                #     hal.elevatorSetpoint = hal.elevatorPos
                if abs(hal.armPos - self.ARM_BOTTOM_POS) < 0.25:
                    hal.elevatorSetpoint = hal.elevatorPos

        elif self.mode == ElevatorMode.MANUAL_MODE:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionVelocityControl
            hal.elevatorSlot = ClosedLoopSlot.kSlot1
            # velocity logic on bottom and top
            self.velSetpoint = int(90 * up + (-90 * down))  # moves the elevator
            hal.elevatorSetpoint = self.velSetpoint + self.table.getNumber(
                "Elevator setpoint offset", 0
            )

        if not hal.elevatorPos <= 0.8 or hal.secondManipulatorSensor:
            hal.elevServoAngle = 60
        else:
            hal.elevServoAngle = 0

        if self.debugMode:
            self.table.putNumber("Elevator Setpoint(e)", hal.elevatorSetpoint)
            self.table.putNumber("Elevator Pos Setpoint", self.posSetpoint)
            self.table.putNumber("Elevator Vel Setpoint", self.velSetpoint)
        self.table.putBoolean("Algae Pos Mode", algaePosMode)
        self.table.putBoolean("Move Arm Down", self.moveArmDown)
        self.table.putString("Elevator State", self.mode.name)
        hal.elevatorArbFF = 0.5 + self.table.getNumber("Elevator arbFF offset", 0)
