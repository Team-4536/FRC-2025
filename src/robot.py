import math
import rev
import robotHAL
import wpilib
import swerveDrive
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from timing import TimeData
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from elevator import ElevatorSubsystem
from robotHAL import RobotHAL, RobotHALBuffer
from swerveDrive import SwerveDrive
from wpimath.units import radians
from manipulator import ManipulatorSubsystem


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.mechCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        
        self.time = TimeData(None)
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware: robotHAL.RobotHAL | RobotSimHAL
        if self.isSimulation():
            self.hardware = RobotSimHAL()
        else:
            self.hardware = robotHAL.RobotHAL()

        self.hardware.update(self.hal, self.time)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)
        self.mechCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)

        self.elevatorSubsystem = ElevatorSubsystem()
        self.manipulatorSubsystem = ManipulatorSubsystem()

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.swerveDrive.updateOdometry(self.hal)
        self.hal.stopMotors()

    def teleopInit(self) -> None:
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

        self.swerveDrive.update(
            self.hal,
            self.driveCtrlr.getLeftX(),
            -self.driveCtrlr.getLeftY(),
            self.driveCtrlr.getRightX(),
        )

        self.elevatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getRightTriggerAxis(),
            self.mechCtrlr.getLeftTriggerAxis(),
            self.mechCtrlr.getYButtonPressed(),
            self.mechCtrlr.getPOV(),
        )

        self.manipulatorSubsystem.update(
            self.hal, self.mechCtrlr.getAButton(), self.mechCtrlr.getLeftBumperPressed()
        )

        if self.driveCtrlr.getAButton():
            self.hardware.resetGyroToAngle(0)

        self.swerveDrive.updateOdometry(self.hal)
        # Keep the lines below at the bottom of teleopPeriodic
        self.hal.publish(self.table)
        self.hardware.update(self.hal, self.time)

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of autonomousPeriodic

        # self.swerveDrive.resetOdometry(self, Pose2d(0, 0, Rotation2d(radians(0))), self.hal)
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)

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
