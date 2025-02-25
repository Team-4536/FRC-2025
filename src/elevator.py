from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from rev import (
    SparkMax,
    ClosedLoopSlot,
)
from enum import Enum
from codeProfile import profile


class ElevatorMode(Enum):
    MANUAL_MODE = 0
    POSITION_MODE = 1


class ElevatorSubsystem:

    INTAKE_POS = 0
    L2_POS = 11.71
    L3_POS = 24.59
    L4_POS = 45

    def __init__(self):
        self.table = (
            NetworkTableInstance.getDefault()
            .getTable("telemetry")
            .getSubTable("Elevator Subsystem")
        )
        self.table.putBoolean("Elevator Debug Mode", False)
        self.table.putNumber("Elevator setpoint offset", 0)
        self.table.putNumber("Elevator arbFF offset", 0)
        self.velSetpoint = 0
        self.posSetpoint = 0
        self.mode = ElevatorMode.MANUAL_MODE
        self.debugMode = False

    @profile(__name__)
    def update(
        self,
        hal: RobotHALBuffer,
        up: float,
        down: float,
        toggleMode: bool,
        POVSetpoint: float,
    ):
        # Dead-Zone
        if up < 0.1:
            up = 0
        if down < 0.1:
            down = 0

        if toggleMode:
            if self.mode == ElevatorMode.MANUAL_MODE:
                self.mode = ElevatorMode.POSITION_MODE
                self.posSetpoint = hal.elevatorPos
                self.velSetpoint = 0
            elif self.mode == ElevatorMode.POSITION_MODE:
                self.mode = ElevatorMode.MANUAL_MODE
                self.velSetpoint = 0
                self.posSetpoint = 0

        if self.mode == ElevatorMode.POSITION_MODE:
            hal.elevatorControl = SparkMax.ControlType.kPosition
            hal.elevatorSlot = ClosedLoopSlot.kSlot0

            if POVSetpoint == 0:
                self.posSetpoint = self.INTAKE_POS
            elif POVSetpoint == 90:
                self.posSetpoint = self.L2_POS
            elif POVSetpoint == 180:
                self.posSetpoint = self.L3_POS
            elif POVSetpoint == 270:
                self.posSetpoint = self.L4_POS
            hal.elevatorSetpoint = self.posSetpoint

        elif self.mode == ElevatorMode.MANUAL_MODE:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionVelocityControl
            hal.elevatorSlot = ClosedLoopSlot.kSlot1
            # velocity logic on bottom and top
            self.velSetpoint = 75 * up + (-75 * down)  # moves the elevator
            hal.elevatorSetpoint = self.velSetpoint

        # all debug stuff is here
        self.debugMode = self.table.getBoolean("Debug Mode", False)
        if self.debugMode:
            self.table.putNumber("Elevator up", up)
            self.table.putNumber("Elevator down", down)
            self.table.putNumber("Elevator Setpoint", hal.elevatorSetpoint)
            self.table.putNumber("Elevator Pos Setpoint", self.posSetpoint)
            self.table.putNumber("Elevator Vel Setpoint", self.velSetpoint)
            self.table.putNumber("Elevator State", self.mode.value)

            hal.elevatorSetpoint += self.table.getNumber("Elevator setpoint offset", 0)

        hal.elevatorArbFF = 0.3 + self.table.getNumber("Elevator arbFF offset", 0)
