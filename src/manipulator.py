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
    MANUAL = 99


class ManipulatorSubsystem:

    debug = False

    def __init__(self):

        self.state: ManipulatorState = ManipulatorState.IDLE
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(
        self,
        buf: RobotHALBuffer,
        AButton: bool,
        LBumper: bool,
    ):
        #=========================================================
        if buf.controlMode == 0 or buf.controlMode == 1:
            if LBumper and self.state != ManipulatorState.MANUAL:
                self.state = ManipulatorState.MANUAL
                LBumper = False
        #=========================================================

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

            #=====================================================================
            if not buf.firstManipulatorSensor and not buf.secondManipulatorSensor:
                self.state = ManipulatorState.IDLE
            #=====================================================================

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
            self.table.putNumber("manipulator voltage", buf.manipulatorVolts)
        self.table.putString("maniState", self.state.name)
