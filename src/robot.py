import math
import rev
import robotHAL
import wpilib
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from timing import TimeData
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from elevator import ElevatorSubsystem
from robotHAL import RobotHAL
from swerveDrive import SwerveDrive
from manipulator import ManipulatorSubsystem
from IntakeChute import IntakeChute
import profiler


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.time = TimeData(None)
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware: robotHAL.RobotHAL | RobotSimHAL
        if self.isSimulation():
            self.hardware = RobotSimHAL()
        else:
            self.hardware = robotHAL.RobotHAL()
        # self.hardware = robotHAL.RobotHAL()

        self.hardware.update(self.hal, self.time)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)
        self.mechCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.elevatorSubsystem = ElevatorSubsystem()
        self.manipulatorSubsystem = ManipulatorSubsystem()
        self.intakeChute = IntakeChute()

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish()
        self.hal.stopMotors()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        profiler.start()
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

        profiler.start()
        self.elevatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getRightTriggerAxis(),
            self.mechCtrlr.getLeftTriggerAxis(),
            self.mechCtrlr.getYButtonPressed(),
            self.mechCtrlr.getPOV(),
        )
        profiler.end("ElevatorSubsystem")

        profiler.start()
        self.intakeChute.update(
            self.hal,
            self.driveCtrlr.getRightTriggerAxis() >= 0.5,
            self.driveCtrlr.getLeftTriggerAxis() >= 0.5,
            self.driveCtrlr.getBButtonPressed(),
            self.driveCtrlr.getYButtonPressed(),
        )
        profiler.end("IntakeSubsystem")

        profiler.start()
        self.manipulatorSubsystem.update(
            self.hal, self.mechCtrlr.getAButton(), self.mechCtrlr.getLeftBumperPressed()
        )
        profiler.end("ManipulatorSubsystem")

        if self.driveCtrlr.getAButton():
            self.hardware.resetGyroToAngle(0)

        # Keep the lines below at the bottom of teleopPeriodic
        self.hal.publish()
        self.hardware.update(self.hal, self.time)

        profiler.end("TeleopPeriodic")

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of autonomousPeriodic

        self.hardware.update(
            self.hal, self.time
        )  # Keep this at the bottom of autonomousPeriodic

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal, self.time)


if __name__ == "__main__":
    wpilib.run(Robot)
