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
    ALGAE_L2_POS = 0
    ALGAE_L3_POS = 0

    ARM_UP_POS = 5
    ARM_DEALGAE_POS = 2.5
    # the threshold at which the arm does not need to wory about hitting the bumpers
    ARM_CLEAR_BUMPER_POS = 3.5

    # this is an elevator position where it is safe for the arm to move
    ELEVATOR_CLEARS_BUMPERS_FOR_ARM = 7

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("Elevator setpoint offset", 0)
        self.table.putNumber("Elevator arbFF offset", 0)

        self.velSetpoint = 0
        self.posSetpoint = 0

        self.mode = ElevatorMode.POSITION_MODE
        self.debugMode = False

        self.moveArmDown: bool = False
        self.armReachedTop: bool = False
        self.algaePosMode: bool = False

    def update(
        self,
        hal: RobotHALBuffer,
        up: float,
        down: float,
        toggleManualMode: bool,
        POVSetpoint: float,
        armToggle: bool,
        algaePosToggle: bool,
    ):
        if armToggle:
            self.moveArmDown = not self.moveArmDown

        # dont let the toggle happen uless the arm is high enough or the elevator is above the threshold
        # this is to avoid the arm bumping into the bumper, this avoid more logic down the line
        if algaePosToggle and (
            hal.armPos > self.ARM_CLEAR_BUMPER_POS
            or hal.elevatorPos > self.ELEVATOR_CLEARS_BUMPERS_FOR_ARM
        ):
            self.algaePosMode = not self.algaePosMode

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
                self.posSetpoint = hal.elevatorPos
                self.velSetpoint = 0
            elif self.mode == ElevatorMode.POSITION_MODE:
                self.mode = ElevatorMode.MANUAL_MODE
                self.velSetpoint = 0
                self.posSetpoint = 0
        hal.elevatorSetpoint = self.velSetpoint + self.table.getNumber(
            "Elevator setpoint offset", 0
        )

        if self.mode == ElevatorMode.POSITION_MODE:
            hal.elevatorControl = SparkMax.ControlType.kPosition
            hal.elevatorSlot = ClosedLoopSlot.kSlot0

            if not self.algaePosMode:
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

                # always be moving arm up when not in dealgae mode and when bumpers are not in the way
                if (
                    hal.elevatorPos > self.ELEVATOR_CLEARS_BUMPERS_FOR_ARM
                    or hal.armPos > self.ARM_CLEAR_BUMPER_POS
                ):
                    hal.armSetpoint = self.ARM_UP_POS
                # Don't let the elevator move down below bumper threshold unless the arm is all the way up
                if (
                    self.posSetpoint < self.ELEVATOR_CLEARS_BUMPERS_FOR_ARM
                    and hal.armPos < self.ARM_CLEAR_BUMPER_POS
                ):
                    self.posSetpoint = hal.elevatorPos

                # when exiting normal scoring mode make sure the arm stays up
                self.moveArmDown = False

            else:  # algae pos mode
                if POVSetpoint == 180:
                    self.posSetpoint = self.ALGAE_L2_POS
                elif POVSetpoint == 0:
                    self.posSetpoint = self.ALGAE_L3_POS

                # it can be assumed that the arm is in an safe position because of the strict toggle logic
                if self.moveArmDown:
                    hal.armSetpoint = self.ARM_DEALGAE_POS
                else:
                    hal.armSetpoint = self.ARM_UP_POS

        elif self.mode == ElevatorMode.MANUAL_MODE:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionVelocityControl
            hal.elevatorSlot = ClosedLoopSlot.kSlot1
            # velocity logic on bottom and top
            self.velSetpoint = 90 * up + (-90 * down)  # moves the elevator

        if (
            not hal.elevatorPos <= 0.8
            or hal.secondManipulatorSensor
            or hal.firstManipulatorSensor
            and hal.secondManipulatorSensor
        ):
            hal.elevServoAngle = 60
        else:
            hal.elevServoAngle = 0

        if self.debugMode:
            self.table.putNumber("Elevator Setpoint(e)", hal.elevatorSetpoint)
            self.table.putNumber("Elevator Pos Setpoint", self.posSetpoint)
            self.table.putNumber("Elevator Vel Setpoint", self.velSetpoint)
        self.table.putBoolean("Algae Removal Mode", self.algaePosMode)
        self.table.putBoolean("Move Arm Down", self.moveArmDown)
        self.table.putString("Elevator State", self.mode.name)
        hal.elevatorArbFF = 0.5 + self.table.getNumber("Elevator arbFF offset", 0)
