import math
from photonOdometry import photonVision
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
        # self.driveCtrlr.getLeftBumperButtonPressed()
        self.photonCamera1 = photonVision("Camera1", 0, 0, 0, 0)
        self.photonCamera2 = photonVision("Camera2", 0, 0, 0, 0)
        # fileTest = open("pyTest.txt", "w")
        self.photonCamera1.photonTable.putBoolean("a button", False)
        self.a = True

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.elevatorSubsystem = ElevatorSubsystem()
        self.manipulatorSubsystem = ManipulatorSubsystem()

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.hal.stopMotors()
        self.photonCamera1.update()
        self.photonCamera2.update()
        self.PC1X = self.photonCamera1.robotX
        self.PC2X = self.photonCamera2.robotX
        self.PC1Y = self.photonCamera1.robotY
        self.PC2Y = self.photonCamera2.robotY
        self.PC1Angle = self.photonCamera1.robotAngle
        self.PC2Angle = self.photonCamera2.robotAngle
        self.photonCamera1.photonTable.putNumber("cam 1 x", self.PC1X)

        if self.driveCtrlr.getAButtonPressed():
            self.photonCamera1.savePos()
            self.photonCamera2.savePos()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

        self.swerveDrive.update(
            self.hal,
            self.driveCtrlr.getLeftX(),
            -self.driveCtrlr.getLeftY(),
            self.driveCtrlr.getRightX(),
        )
        # if self.driveCtrlr.getLeftBumperPressed:
        #     if self.driveCtrlr.get
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

        # Keep the lines below at the bottom of teleopPeriodic
        self.hal.publish(self.table)
        self.hardware.update(self.hal, self.time)

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
