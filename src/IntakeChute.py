import wpilib
from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from enum import Enum


class ChuteStates(Enum):
    MANUAL = 1
    UP = 2
    DOWN = 3


class IntakeChute:

    def __init__(self):

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.chuteSpeed = 5
        self.state = ChuteStates.DOWN
        self.table.putNumber("Chute Control Mode", self.state.name)
        self.chuteDirection = 0
        self.resetStateUP = True
        self.manualState = False
        self.downState = True

    def update(
        self,
        hal: RobotHALBuffer,
        chuteDown: bool,
        chuteUp: bool,
        switchAutoState: bool,
        switchManualState: bool,
    ):

        self.table.putNumber("time", wpilib.getTime())
        self.table.putNumber("Intake Chute Voltage", hal.chuteMotorVoltage)
        self.table.putBoolean("limit switch", hal.chuteLimitSwitch)
        self.table.putBoolean("right trigger", chuteDown)
        self.table.putBoolean("left trigger", chuteUp)

        self.table.putBoolean("Chute In Manual", self.manualState)

        if switchManualState:
            self.manualState = not (self.manualState)

        if switchAutoState:
            self.downState = not (self.downState)

        if self.manualState:
            self.state = ChuteStates.MANUAL
        else:
            if self.downState:
                self.state = ChuteStates.DOWN
            else:
                self.state = ChuteStates.UP

        self.table.putNumber("Chute Control Mode", self.state.name)

        if chuteUp:
            self.chuteDirection = -1

        elif chuteDown:
            self.chuteDirection = 1

        else:
            self.chuteDirection = 0

        if self.state == ChuteStates.MANUAL:
            hal.setChuteVoltage = (
                self.chuteDirection * self.chuteSpeed
            )  # POSITIVE Pulls down Negative lets up

        elif self.state == ChuteStates.DOWN:
            if (
                not hal.chuteLimitSwitch
            ):  # returns true if the hal.chuteLimitSwitch is pressed
                hal.setChuteVoltage = self.chuteSpeed

            else:
                hal.setChuteVoltage = 0
                self.state = ChuteStates.MANUAL

        elif self.state == ChuteStates.UP:

            if self.resetStateUP:
                self.startTime = wpilib.getTime()
                self.resetStateUP = False

            self.currentTime = wpilib.getTime()

            if self.currentTime - self.startTime < 10:
                hal.setChuteVoltage = -self.chuteSpeed

            else:
                hal.setChuteVoltage = 0
                self.state = ChuteStates.MANUAL
