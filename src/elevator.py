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
        # mode 0 is position control, 1 is velocity

        self.setPointmode = True

    def update(
        self,
        hal: RobotHALBuffer,
        up: float,
        down: float,
        togglePositionMode: bool,
        toggleVelocityMode: bool,
        POVSetpoint: float,
    ):
        self.table.putNumber("Elevator up", up)
        self.table.putNumber("Elevator down", down)
        if togglePositionMode:
            self.mode = 0
            self.posSetpoint = hal.elevatorPos
        elif toggleVelocityMode:
            self.mode = 1
            self.velSetpoint = 0

        self.table.putNumber("Elevator State", self.mode)

        # Dead-Zone
        if up < 0.1:
            up = 0
        if down < 0.1:
            down = 0

        if self.setPointmode:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionPositionControl
            if (
                hal.elevatorPos > self.posSetpoint - 10
                and hal.elevatorPos < self.posSetpoint
            ):
                hal.elevatorSlot = ClosedLoopSlot.kSlot3
            elif (
                hal.elevatorPos < self.posSetpoint + 10
                and hal.elevatorPos > self.posSetpoint
            ):
                hal.elevatorSlot = ClosedLoopSlot.kSlot3
            else:
                hal.elevatorSlot = ClosedLoopSlot.kSlot0
            if POVSetpoint == 0:
                self.posSetpoint = 0
            elif POVSetpoint == 90:
                self.posSetpoint = 20
            elif POVSetpoint == 180:
                self.posSetpoint = 30
            self.velSetpoint = 0
        elif not self.setPointmode:
            hal.elevatorControl = SparkMax.ControlType.kMAXMotionVelocityControl
            hal.elevatorSlot = ClosedLoopSlot.kSlot1

            # velocity logic on bottom and top
            if hal.elevatorPos < 10 and self.velSetpoint < 0:
                hal.elevatorSlot = ClosedLoopSlot.kSlot2
            elif hal.elevatorPos > 35 and self.velSetpoint > 0:
                hal.elevatorSlot = ClosedLoopSlot.kSlot2
            self.velSetpoint = up + (-1 * down)  # moves the elevator
            self.posSetpoint = 0

        # # prohibit setpoint locations
        # if self.posSetpoint < 0:
        #     self.posSetpoint = 0
        # if self.posSetpoint > 45:
        #     self.posSetpoint = 45

        hal.elevatorSetpoint = (
            self.velSetpoint
            + self.posSetpoint
            + self.table.getNumber("Elevator setpoint offset", 0)
        )
        hal.elevatorArbFF = 0.3 + self.table.getNumber("Elevator arbFF offset", 0)
