from robotHAL import RobotHALBuffer
from robotHAL import RobotHAL
from ntcore import NetworkTableInstance
from robotHAL import RobotHAL
from rev import (
    SparkMax,
    ClosedLoopSlot,
)

from enum import Enum


class ElevatorMode(Enum):
    MANUAL_MODE = 0
    POSITION_MODE = 1


class ElevatorSubsystem:

    INTAKE_POS = 0
    L2_POS = 11.71 + 1.5  # added 1.5 for north star
    L3_POS = 24.59 + 1.5  # added 1.5 for north star
    L4_POS = 45.5  # changed from 45
    ALGAE_L2_POS = 13.78
    ALGAE_L3_POS = 24.52

    ARM_UP_POS = 28
    ARM_DEALGAE_POS = 12
    ARM_BOTTOM_POS = -1  # changed from 0

    # this is an elevator position where it is safe for the arm to move
    ELEVATOR_CLEARS_BUMPERS_FOR_ARM = 7

    def __init__(self) -> None:
        self.table = (
            NetworkTableInstance.getDefault()
            .getTable("telemetry")
            .getSubTable("Elevator Subsystem")
        )

        self.elevatorDebugModeNT = self.table.getEntry("Elevator Debug Mode")
        self.elevatorDebugModeNT.setBoolean(False)
        self.elevatorSetpointOffsetNT = self.table.getEntry("Elevator Setpoint offset")
        self.elevatorSetpointOffsetNT.setFloat(0)
        self.elevatorArbFFOffsetNT = self.table.getEntry("Elevator arbFF offset")
        self.elevatorArbFFOffsetNT.setFloat(0)

        self.elevatorUpNT = self.table.getEntry("Elevator up")
        self.elevatorDownNT = self.table.getEntry("Elevator down")

        self.setpointNT = self.table.getEntry("Elevator Setpoint")
        self.posSetpointNT = self.table.getEntry("Elevator Pos Setpoint")
        self.velSetpointNT = self.table.getEntry("Elevator Vel Setpoint")
        self.algaePosModeNT = self.table.getEntry("Algae Pos Mode")
        self.moveArmDownNT = self.table.getEntry("Move Arm Down")
        self.stateNT = self.table.getEntry("Elevator State")

        self.velSetpoint: float = 0.0
        self.posSetpoint: float = 0.0

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
        algaePosMode: bool,
    ):
        if armToggle:
            self.moveArmDown = not self.moveArmDown

        # Dead-Zone
        if up < 0.1:
            up = 0
        if down < 0.1:
            down = 0

        if toggleManualMode:
            if self.mode == ElevatorMode.MANUAL_MODE:
                self.mode = ElevatorMode.POSITION_MODE
                self.posSetpoint = hal.elevatorPos
                self.velSetpoint = 0
            elif self.mode == ElevatorMode.POSITION_MODE:
                self.mode = ElevatorMode.MANUAL_MODE
                self.velSetpoint = 0
                self.posSetpoint = 0

        hal.elevatorSetpoint = (
            self.velSetpoint + self.elevatorSetpointOffsetNT.getFloat(0)
        )

        if self.mode == ElevatorMode.POSITION_MODE:
            hal.elevatorControl = SparkMax.ControlType.kPosition
            hal.elevatorSlot = ClosedLoopSlot.kSlot0

            if not algaePosMode:
                if POVSetpoint == 180:
                    self.posSetpoint = self.INTAKE_POS
                elif POVSetpoint == 90:
                    self.posSetpoint = self.L2_POS
                elif POVSetpoint == 270:
                    self.posSetpoint = self.L3_POS
                elif POVSetpoint == 0:
                    self.posSetpoint = self.L4_POS

            else:  # algae pos mode
                if POVSetpoint == 180:
                    self.posSetpoint = self.ALGAE_L2_POS
                elif POVSetpoint == 0:
                    self.posSetpoint = self.ALGAE_L3_POS

            hal.elevatorSetpoint = (
                self.posSetpoint + self.elevatorSetpointOffsetNT.getFloat(0)
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
                if not hal.armBottomLimitSwitch:
                    hal.elevatorSetpoint = hal.elevatorPos

        elif self.mode == ElevatorMode.MANUAL_MODE:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionVelocityControl
            hal.elevatorSlot = ClosedLoopSlot.kSlot1
            # velocity logic on bottom and top
            self.velSetpoint = 90 * up + (-90 * down)  # moves the elevator
            hal.elevatorSetpoint = (
                self.velSetpoint + self.elevatorSetpointOffsetNT.getFloat(0)
            )

        if not hal.elevatorPos <= 0.8 or hal.secondManipulatorSensor:
            hal.elevServoAngle = 60
        else:
            hal.elevServoAngle = 0

        self.debugMode = self.elevatorDebugModeNT.getBoolean(False)
        if self.debugMode:
            self.elevatorUpNT.setFloat(up)
            self.elevatorDownNT.setFloat(down)
            self.setpointNT.setFloat(hal.elevatorSetpoint)
            self.posSetpointNT.setFloat(self.posSetpoint)
            self.velSetpointNT.setFloat(self.velSetpoint)
        self.moveArmDownNT.setBoolean(self.moveArmDown)
        self.algaePosModeNT.setBoolean(self.algaePosMode)
        self.stateNT.setString(self.mode.name)

        hal.elevatorArbFF = 0.5 + self.elevatorArbFFOffsetNT.getFloat(0)

    def level4AutoUpdate(self, hal: RobotHALBuffer):

        # Dead-Zone

        hal.elevatorControl = SparkMax.ControlType.kPosition
        hal.elevatorSlot = ClosedLoopSlot.kSlot0

        self.posSetpoint = self.L4_POS
        hal.elevatorSetpoint = (
            self.posSetpoint + self.elevatorSetpointOffsetNT.getFloat(0)
        )

        if hal.elevatorSetpoint < 5 and not hal.backArmLimitSwitch:
            hal.elevatorSetpoint = hal.elevatorPos
            hal.armVolts = -1
        elif hal.elevatorSetpoint >= 5 and hal.elevatorPos >= 5:
            hal.armVolts = 1
        if hal.moveArmDown:
            hal.armVolts = -1

    def level0AutoUpdate(self, hal: RobotHALBuffer):

        # Dead-Zone
        hal.elevatorControl = SparkMax.ControlType.kPosition
        hal.elevatorSlot = ClosedLoopSlot.kSlot0

        self.posSetpoint = self.INTAKE_POS
        hal.elevatorSetpoint = (
            self.posSetpoint + self.elevatorSetpointOffsetNT.getFloat(0)
        )

        if hal.elevatorSetpoint < 5 and not hal.backArmLimitSwitch:
            hal.elevatorSetpoint = hal.elevatorPos
            hal.armVolts = -1
        elif hal.elevatorSetpoint >= 5 and hal.elevatorPos >= 5:
            hal.armVolts = 1
        if hal.moveArmDown:
            hal.armVolts = -1
