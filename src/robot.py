import math

import robotHAL
import wpilib
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from timing import TimeData
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from manipulatorSubsystem import manipulatorSub

class RobotInputs:
    def __init__(self) -> None:
        self.driveCtrlr = wpilib.XboxController(0)
        self.mechCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

    def update(self) -> None:
        pass


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.time = TimeData(None)
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware: robotHAL.RobotHAL | RobotSimHAL
        if self.isSimulation():
            self.hardware = RobotSimHAL()
        else:
            self.hardware = robotHAL.RobotHAL()

        self.hardware.update(self.hal, self.time)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.input = RobotInputs()

        self.manipulatorSub: manipulatorSub = manipulatorSub()

        self.mechCtrlr = wpilib.XboxController(1)

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)

        self.hal.publish(self.table)

        self.hal.stopMotors()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        

        self.manipulatorSub.update(self.hal, self.mechCtrlr.getYButton(), self.mechCtrlr.getXButton())

        self.hal.publish(self.table)
        self.hardware.update(self.hal, self.time)
        
    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()

        self.hardware.update(self.hal, self.time)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal, self.time)


if __name__ == "__main__":
    wpilib.run(Robot)
