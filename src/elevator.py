# imports
from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from rev import (
    SparkMax,
    ClosedLoopSlot,
)


class ElevatorSubsystem:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("Elevator setpoint offset", 0)
        self.table.putNumber("Elevator arbFF offset", 0)
        self.velSetpoint = 0
        self.posSetpoint = 0
        self.mode = 0

    def update(
        self,
        hal: RobotHALBuffer,
        up: float,
        down: float,
        togglePositionMode: bool,
        toggleVelocityMode: bool,
    ):
        self.table.putNumber("Elevator up", up)
        self.table.putNumber("Elevator down", down)
        if togglePositionMode:
            self.mode = 0
        elif toggleVelocityMode:
            self.mode = 1

        # Dead-Zone
        if up < 0.1:
            up = 0
        if down < 0.1:
            down = 0

        if self.mode == 0:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionPositionControl
            hal.elevatorSlot = ClosedLoopSlot.kSlot0

        else:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionVelocityControl
            hal.elevatorSlot = ClosedLoopSlot.kSlot1
            # velocity logic on bottom and top
            if hal.elevatorPos < 10 and self.velSetpoint < 0:
                hal.elevatorSlot = ClosedLoopSlot.kSlot2
            elif hal.elevatorPos > 35 and self.velSetpoint > 0:
                hal.elevatorSlot = ClosedLoopSlot.kSlot2
            self.velSetpoint = up + (-1 * down)  # moves the elevator

        # prohibit setpoint locations
        if self.posSetpoint < 0:
            self.posSetpoint = 0
        if self.posSetpoint > 45:
            self.posSetpoint = 45

        hal.elevatorSetpoint = self.posSetpoint + self.table.getNumber(
            "Elevator setpoint offset", 0
        )
        hal.elevatorArbFF = 0.3 + self.table.getNumber("Elevator arbFF offset", 0)
