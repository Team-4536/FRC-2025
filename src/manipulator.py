from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from timing import TimeData
import wpilib
from enum import Enum
from rev import SparkMax


class ManipulatorState(Enum):
    IDLE = 0
    INTAKE = 1
    STORED = 2
    SHOOT = 3
    MANUAL = -1
    GOINGUP = 4
    UP = 5
    GOINGDOWN = 6


class ManipulatorSubsystem:
    debug = True

    def __init__(self):

        self.state = ManipulatorState.IDLE
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(
        self,
        buf: RobotHALBuffer,
        AButton: bool,
        LBumper: bool,
    ):
        if LBumper and self.state != ManipulatorState.MANUAL:
            self.state = ManipulatorState.MANUAL
            LBumper = False

        if self.state == ManipulatorState.IDLE:
            buf.manipulatorVolts = 0
            if buf.firstManipulatorSensor:
                self.state = ManipulatorState.INTAKE

        elif self.state == ManipulatorState.INTAKE:
            buf.manipulatorVolts = 4

            if not buf.secondManipulatorSensor and not buf.firstManipulatorSensor:
                self.state = ManipulatorState.IDLE

            if buf.secondManipulatorSensor and not buf.firstManipulatorSensor:
                self.state = ManipulatorState.STORED

        elif self.state == ManipulatorState.STORED:
            buf.manipulatorVolts = 0
            self.startTime = wpilib.getTime()

            if AButton:
                self.state = ManipulatorState.SHOOT

        elif self.state == ManipulatorState.SHOOT:
            buf.manipulatorVolts = 8

            if (
                wpilib.getTime() - self.startTime > 0.5
                and not buf.secondManipulatorSensor
            ):  # how long the shooting goes for in sec
                self.state = ManipulatorState.IDLE

        elif self.state == ManipulatorState.MANUAL:
            if AButton:
                buf.manipulatorVolts = 8
            else:
                buf.manipulatorVolts = 0

            if LBumper:
                self.state = ManipulatorState.IDLE

        if self.debug == True:
            self.table.putNumber("maniState", self.state.value)
            self.table.putNumber("manipulator voltage", buf.manipulatorVolts)

    def autoShootStored(self, buf: RobotHALBuffer, startTime):

        buf.manipulatorVolts = 8

        if (wpilib.getTime() - startTime > 1) and (
            buf.secondManipulatorSensor == False
        ):  # how long the shooting goes for in sec
            self.state = ManipulatorState.IDLE

    # def autoIntake(self, buf: RobotHALBuffer):
    #     self.state = ManipulatorState.IDLE
    #     buf.manipulatorVolts = 0
    #     buf.moveArmDown = True
    #     if buf.firstManipulatorSensor:
    #         self.state = ManipulatorState.INTAKE

    #     elif self.state == ManipulatorState.INTAKE:
    #         buf.manipulatorVolts = 4

    #         if not buf.secondManipulatorSensor and not buf.firstManipulatorSensor:
    #             self.state = ManipulatorState.IDLE

    #         if buf.secondManipulatorSensor and not buf.firstManipulatorSensor:
    #             self.state = ManipulatorState.STORED
