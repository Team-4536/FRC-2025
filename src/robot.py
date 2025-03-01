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
import robotAutos
from robotAutos import RobotAutos
from wpimath.units import radians
from manipulator import ManipulatorSubsystem
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController  # type: ignore
from IntakeChute import IntakeChute


class Robot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        AUTO_SIDE_RED = "red"
        AUTO_SIDE_BLUE = "blue"
        AUTO_SIDE_FMS = "FMS side"
        self.mechCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.autoSubsys = robotAutos.RobotAutos()

        self.OdomField = wpilib.Field2d()

        self.time = TimeData(None)
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware: robotHAL.RobotHAL | RobotSimHAL
        if self.isSimulation():
            self.hardware = RobotSimHAL()
        else:
            self.hardware = robotHAL.RobotHAL()

        self.autoSideChooser = wpilib.SendableChooser()
        self.autoSideChooser.setDefaultOption(AUTO_SIDE_FMS, AUTO_SIDE_FMS)
        self.autoSideChooser.addOption(AUTO_SIDE_RED, AUTO_SIDE_RED)
        self.autoSideChooser.addOption(AUTO_SIDE_BLUE, AUTO_SIDE_BLUE)
        wpilib.SmartDashboard.putData("auto side chooser", self.autoSideChooser)

        self.hardware.update(self.hal, self.time)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)
        self.mechCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)

        self.elevatorSubsystem = ElevatorSubsystem()
        self.manipulatorSubsystem = ManipulatorSubsystem()
        self.intakeChute = IntakeChute()

    def robotPeriodic(self) -> None:

        AUTO_SIDE_RED = "red"
        AUTO_SIDE_BLUE = "blue"
        AUTO_SIDE_FMS = "FMS side"

        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.swerveDrive.updateOdometry(self.hal)

        self.OdomField.setRobotPose(self.swerveDrive.odometry.getPose())
        wpilib.SmartDashboard.putData("Odom", self.OdomField)

        self.onRedSide: bool = self.autoSideChooser.getSelected() == AUTO_SIDE_RED
        if self.autoSideChooser.getSelected() == AUTO_SIDE_FMS:
            if (
                NetworkTableInstance.getDefault()
                .getTable("FMSInfo")
                .getBoolean("IsRedAlliance", False)
            ):
                self.onRedSide = True
            else:
                self.onRedSide = False

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
            self.driveCtrlr.getRightTriggerAxis(),
        )

        self.elevatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getRightTriggerAxis(),
            self.mechCtrlr.getLeftTriggerAxis(),
            self.mechCtrlr.getYButtonPressed(),
            self.mechCtrlr.getPOV(),
            self.mechCtrlr.getXButton(),
            self.mechCtrlr.getBButton(),
        )

        self.intakeChute.update(
            self.hal,
            self.driveCtrlr.getLeftBumper(),
            self.driveCtrlr.getRightBumper(),
            self.driveCtrlr.getBButtonPressed(),
            self.driveCtrlr.getYButtonPressed(),
        )

        self.manipulatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getAButton(),
            self.mechCtrlr.getLeftBumperPressed(),
        )

        if self.driveCtrlr.getStartButton():
            self.hardware.resetGyroToAngle(0)

        self.swerveDrive.updateOdometry(self.hal)
        # Keep the lines below at the bottom of teleopPeriodic
        self.hal.publish(self.table)
        self.hardware.update(self.hal, self.time)

    def autonomousInit(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of autonomousPeriodic

        self.auto, _ = self.autoSubsys.autoInit(self)

        self.holonomicDriveController = PPHolonomicDriveController(
            PIDConstants(0.00019, 0, 0, 0), PIDConstants(0.15, 0, 0, 0)
        )

        # self.swerveDrive.resetOdometry(self, Pose2d(0, 0, Rotation2d(radians(0))), self.hal)
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)

        self.hardware.update(
            self.hal, self.time
        )  # Keep this at the bottom of autonomousPeriodic

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of autonomousPeriodic
        self.auto.run(self)
        # self.swerveDrive.resetOdometry(self, Pose2d(0, 0, Rotation2d(radians(0))), self.hal)
        # self.swerveDrive.resetOdometry(Pose2d(), self.hal)

        self.intakeChute.update(
            self.hal,
            False,
            False,
            False,
            False,
        )

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
