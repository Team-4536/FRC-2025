from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from enum import Enum


class ChuteStates(Enum):
    STOP = 1
    UP = 3
    DOWN = 2


class IntakeChute:

    def __init__(self):

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.chuteSpeed = 1
        self.state = ChuteStates.DOWN
        self.table.putString("Chute Control Mode", self.state.name)
        self.chuteDirection = 0
        self.hasReset = False
        self.table.putBoolean("Chute HasReset", self.hasReset)
        self.setPoint = -115.1674728
        self.toggle = False
        self.table.putBoolean("Toggle Chute Mode", self.toggle)

    def update(
        self,
        hal: RobotHALBuffer,
    ):
        self.table.putNumber("Intake Chute Voltage", hal.chuteMotorVoltage)
        self.table.putBoolean("Chute Limit Switch Pressed", hal.chuteLimitSwitch)
        self.table.putNumber("Chute Motor Encoder Position", hal.chutePosition)
        self.table.putString("Chute Control Mode", self.state.name)

        if self.table.getBoolean("Toggle Chute Mode", self.toggle) is not self.toggle:

            self.toggle = self.table.getBoolean("Toggle Chute Mode", self.toggle)

            if self.state == ChuteStates.UP:
                self.state = ChuteStates.STOP
            elif self.state == ChuteStates.STOP:
                self.state = ChuteStates.DOWN
            else:
                self.state = ChuteStates.UP

        if self.state == ChuteStates.STOP:
            hal.setChuteVoltage = 0

        elif self.state == ChuteStates.DOWN:
            if (
                not hal.chuteLimitSwitch
            ):  # returns true if the hal.chuteLimitSwitch is pressed
                hal.setChuteVoltage = (
                    self.chuteSpeed
                )  # POSITIVE Pulls down Negative lets up

            elif not self.hasReset:
                self.state = ChuteStates.UP
                self.hasReset = True
                self.table.putBoolean("Chute HasReset", self.hasReset)
                hal.resetChuteEncoder = True

            else:
                hal.setChuteVoltage = 0

        elif self.state == ChuteStates.UP:

            if hal.chutePosition > self.setPoint:
                hal.setChuteVoltage = -self.chuteSpeed

            else:
                hal.setChuteVoltage = 0
                self.state = ChuteStates.STOP
