from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from timing import TimeData
import wpilib
from enum import Enum
from rev import SparkMax


class ManipulatorSubsystem:
    class ManipulatorState(Enum):
        IDLE = 0
        INTAKE = 1
        STORED = 2
        SHOOT = 3
        MANUAL = 99

    debug = False

    def __init__(self):

        self.state = self.ManipulatorState.IDLE
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(
        self,
        buf: RobotHALBuffer,
        AButton: bool,
        LBumper: bool,
    ):
        if LBumper and self.state != self.ManipulatorState.MANUAL:
            self.state = self.ManipulatorState.MANUAL
            LBumper = False

        if self.state == self.ManipulatorState.IDLE:
            buf.manipulatorVolts = 0
            if buf.firstManipulatorSensor:
                self.state = self.ManipulatorState.INTAKE

        elif self.state == self.ManipulatorState.INTAKE:
            buf.manipulatorVolts = 4

            if not buf.secondManipulatorSensor and not buf.firstManipulatorSensor:
                self.state = self.ManipulatorState.IDLE

            if buf.secondManipulatorSensor and not buf.firstManipulatorSensor:
                self.state = self.ManipulatorState.STORED

        elif self.state == self.ManipulatorState.STORED:
            buf.manipulatorVolts = 0
            self.startTime = wpilib.getTime()

            if AButton:
                self.state = self.ManipulatorState.SHOOT

        elif self.state == self.ManipulatorState.SHOOT:
            buf.manipulatorVolts = 8

            if (
                wpilib.getTime() - self.startTime > 0.5
                and not buf.secondManipulatorSensor
            ):  # how long the shooting goes for in sec
                self.state = self.ManipulatorState.IDLE

        elif self.state == self.ManipulatorState.MANUAL:
            if AButton:
                buf.manipulatorVolts = 8
            else:
                buf.manipulatorVolts = 0

            if LBumper:
                self.state = self.ManipulatorState.IDLE

        if self.debug == True:
            self.table.putNumber("manipulator voltage", buf.manipulatorVolts)
        self.table.putString("maniState", self.state.name)
