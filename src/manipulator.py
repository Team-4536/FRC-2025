from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from timing import TimeData
import wpilib
from enum import Enum


class ManipulatorSubsystem:

    class ManipulatorState(Enum):
        IDLE = 0
        INTAKE = 1
        STORED = 2
        SHOOT = 3
        MANUAL = -1
        GOINGUP = 4
        UP = 5
        GOINGDOWN = 6

    debug = True

    def __init__(self):

        self.state = self.ManipulatorState.IDLE
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(
        self,
        buf: RobotHALBuffer,
        AButton: bool,
        LBumper: bool,
        BButton: bool,
        RBumper: bool,
    ):
        self.table.putBoolean("manRbumper", RBumper)

        if LBumper and not self.state == self.ManipulatorState.MANUAL:
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
                wpilib.getTime() - self.startTime > 1.5
                and not buf.secondManipulatorSensor
            ):  # <how long the shooting goes for in sec
                self.state = self.ManipulatorState.IDLE

        elif self.state == self.ManipulatorState.MANUAL:
            self.table.putNumber("maniphello world", 42)
            if AButton:
                buf.manipulatorVolts = 8
            else:
                buf.manipulatorVolts = 0
            if BButton:
                buf.armVolts = 1
            elif RBumper:
                buf.armVolts = -1
            else:
                buf.armVolts = 0

            # turn rbumper into a way to switch between arm modes
            if LBumper:
                self.state = self.ManipulatorState.IDLE

        # arm things
        # limit switches normaly open
        # false until you touch them

        # if self.state == self.ManipulatorState.IDLE:
        # buf.armVolts = 0
        # if BButton:
        # self.state = self.ManipulatorState.GOINGUP

        # elif self.state == self.ManipulatorState.GOINGUP:
        # buf.armVolts = 5
        # if buf.frontArmLimitSwitch:
        # self.state = self.ManipulatorState.UP

        # elif self.state == self.ManipulatorState.UP:
        # buf.armVolts = 0
        # if BButton:
        # self.state = self.ManipulatorState.GOINGDOWN

        # elif self.state == self.ManipulatorState.GOINGDOWN:
        # buf.armVolts = -5
        # if buf.frontArmLimitSwitch:
        # self.state = self.ManipulatorState.IDLE

        if self.debug == True:
            self.table.putString("maniState", self.state.name)
            self.table.putNumber("manipulator voltage", buf.manipulatorVolts)
            self.table.putNumber("maniArm voltage", buf.armVolts)
