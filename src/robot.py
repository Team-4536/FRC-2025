import math
from photonOdometry import photonVision
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
from IntakeChute import IntakeChute
from led import LEDSignals
import pathplannerlib  # type: ignore
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants  # type: ignore
import autoStages


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        AUTO_SIDE_RED = "red"
        AUTO_SIDE_BLUE = "blue"
        AUTO_SIDE_FMS = "FMS side"

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
        self.photonCamera1 = photonVision("Camera1", 30, 0.17938, 0.33337, 0.2889)
        self.photonCamera2 = photonVision("Camera2", -30, 0.11747, 0.33337, 0.2889)
        # 0.11747
        # 0.33337
        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)

        self.elevatorSubsystem = ElevatorSubsystem()
        self.manipulatorSubsystem = ManipulatorSubsystem()
        self.intakeChute = IntakeChute()
        self.tempFidId = -1
        self.ledSignals: LEDSignals = LEDSignals()

        self.povPrev = 0

        self.autoRoutineChooser = wpilib.SendableChooser()
        self.autoRoutineChooser.setDefaultOption(
            autoStages.RobotAutos.DO_NOTHING.value,
            autoStages.RobotAutos.DO_NOTHING.value,
        )
        for stage in autoStages.RobotAutos:
            self.autoRoutineChooser.addOption(stage.value, stage.value)

        wpilib.SmartDashboard.putData("auto routine chooser", self.autoRoutineChooser)

        self.autoSideChooser = wpilib.SendableChooser()
        # self.autoSideChooser.setDefaultOption(AUTO_SIDE_FMS, AUTO_SIDE_FMS)
        self.autoSideChooser.setDefaultOption(AUTO_SIDE_RED, AUTO_SIDE_RED)
        self.autoSideChooser.addOption(AUTO_SIDE_BLUE, AUTO_SIDE_BLUE)
        wpilib.SmartDashboard.putData("auto side chooser", self.autoSideChooser)

        # self.onRedSide: bool = self.autoSideChooser.getSelected() == AUTO_SIDE_RED

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.swerveDrive.updateOdometry(self.hal)
        self.hal.stopMotors()
        self.photonCamera1.update()
        self.photonCamera2.update()
        if self.photonCamera1.ambiguity == 0.0:
            self.photonPose2d = Pose2d(
                self.photonCamera1.robotX,
                self.photonCamera1.robotY,
                self.photonCamera1.robotAngle,
            )
            self.swerveDrive.odometry.resetPose(self.photonPose2d)
        if self.photonCamera2.ambiguity < 0.2:
            self.photonPose2d = Pose2d(
                self.photonCamera2.robotX,
                self.photonCamera2.robotY,
                self.photonCamera2.robotAngle,
            )
            self.swerveDrive.odometry.resetPose(self.photonPose2d)

        self.ledSignals.update(
            self.manipulatorSubsystem,
            self.elevatorSubsystem,
            self.intakeChute,
            self.hal.elevatorPos,
        )
        startCameraUpdate = wpilib.getTime()
        self.photonCamera1.update()
        self.photonCamera2.update()
        self.table.putNumber("Camera update Time", wpilib.getTime() - startCameraUpdate)
        if self.photonCamera1.ambiguity < 0.2:
            self.photonPose2d = Pose2d(
                self.photonCamera1.robotX,
                self.photonCamera1.robotY,
                self.photonCamera1.robotAngle,
            )
            self.swerveDrive.odometry.resetPose(self.photonPose2d)
        if self.photonCamera2.ambiguity < 0.2:
            self.photonPose2d = Pose2d(
                self.photonCamera2.robotX,
                self.photonCamera2.robotY,
                self.photonCamera2.robotAngle,
            )
            self.swerveDrive.odometry.resetPose(self.photonPose2d)

    def teleopInit(self) -> None:
        self.swerveDrive.resetOdometry(
            Pose2d(), self.hal
        )  # TODO: shouldnt reset pose to 0 0 0
        self.setpointActiveLeft = False
        self.setpointActiveRight = False

        self.hal.rotPIDToggle = False

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic
        if self.photonCamera1.TFID > -1:
            self.tempFidId = self.photonCamera1.TFID
        elif self.photonCamera2.TFID > -1:
            self.tempFidId = self.photonCamera2.TFID

        self.table.putNumber("tempFidID", self.tempFidId)

        if not self.setpointActiveLeft and not self.setpointActiveRight:
            startCameraUpdate = wpilib.getTime()
            self.swerveDrive.update(
                self.hal,
                self.driveCtrlr.getLeftX(),
                self.driveCtrlr.getLeftY(),
                self.driveCtrlr.getRightX(),
                self.driveCtrlr.getRightTriggerAxis(),
                self.driveCtrlr.getStartButtonPressed(),
            )
            self.table.putNumber(
                "Swerve drive update Time", wpilib.getTime() - startCameraUpdate
            )
        if self.driveCtrlr.getLeftBumperButton():
            self.setpointActiveLeft = True
            self.setpointActiveRight = False
            self.tempFidId = self.photonCamera1.fiducialId
            if self.photonCamera2.TFID > -1:
                self.tempFidId = self.photonCamera2.TFID
            elif self.photonCamera1.TFID > -1:
                self.tempFidId = self.photonCamera1.TFID
            else:
                self.setpointActiveLeft = False
        else:
            self.setpointActiveLeft = False
        if self.driveCtrlr.getRightBumperButton():
            self.setpointActiveRight = True
            self.setpointActiveLeft = False
            if self.photonCamera1.TFID > -1:
                self.tempFidId = self.photonCamera1.TFID
            elif self.photonCamera2.TFID > -1:
                self.tempFidId = self.photonCamera2.TFID
            else:
                self.setpointActiveRight = False
        else:
            self.setpointActiveRight = False
        if self.setpointActiveLeft:

            self.swerveDrive.setpointChooser(
                self.swerveDrive.odometry.getPose().rotation().radians(),
                self.tempFidId,
                "left",
            )
            self.swerveDrive.updateWithoutSticks(
                self.hal, self.swerveDrive.adjustedSpeeds
            )
        if self.setpointActiveRight:

            self.swerveDrive.setpointChooser(
                self.swerveDrive.odometry.getPose().rotation().radians(),
                self.tempFidId,
                "left",
            )
            self.swerveDrive.updateWithoutSticks(
                self.hal, self.swerveDrive.adjustedSpeeds
            )
        if self.setpointActiveRight:

            self.swerveDrive.setpointChooser(
                self.swerveDrive.odometry.getPose().rotation().radians(),
                self.tempFidId,
                "right",
            )
            self.swerveDrive.updateWithoutSticks(
                self.hal, self.swerveDrive.adjustedSpeeds
            )
        if (
            abs(self.driveCtrlr.getLeftX()) > 0.07
            or abs(self.driveCtrlr.getLeftY()) > 0.07
            or abs(self.driveCtrlr.getRightX()) > 0.07
            or abs(self.driveCtrlr.getRightY()) > 0.07
        ):
            self.setpointActiveLeft = False
            self.setpointActiveRight = False
            # self.tempFidId = -1
        startCameraUpdate = wpilib.getTime()
        self.elevatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getRightTriggerAxis(),
            self.mechCtrlr.getLeftTriggerAxis(),
            self.mechCtrlr.getYButtonPressed(),
            self.mechCtrlr.getPOV(),
            self.mechCtrlr.getXButtonPressed(),
            self.mechCtrlr.getBButton(),
        )
        self.table.putNumber(
            "Elavator update Time", wpilib.getTime() - startCameraUpdate
        )

        # convert POV buttons to bool values (sorry michael this code may be hard to look at)
        self.povLeftPressed = False
        self.povRightPressed = False
        if self.driveCtrlr.getPOV() == 270 and self.povPrev != 270:
            self.povLeftPressed = True
        if self.driveCtrlr.getPOV() == 90 and self.povPrev != 90:
            self.povRightPressed = True
        self.povPrev = self.driveCtrlr.getPOV()
        startCameraUpdate = wpilib.getTime()
        self.intakeChute.update(
            self.hal,
            self.driveCtrlr.getPOV() == 180,
            self.driveCtrlr.getPOV() == 0,
            self.povRightPressed,
            self.povLeftPressed,
        )
        self.table.putNumber(
            "Intake chute update Time", wpilib.getTime() - startCameraUpdate
        )
        startCameraUpdate = wpilib.getTime()
        self.manipulatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getAButton(),
            self.mechCtrlr.getLeftBumperPressed(),
        )
        self.table.putNumber(
            "manipulator update Time", wpilib.getTime() - startCameraUpdate
        )
        # if self.driveCtrlr.getStartButton():
        #     self.hardware.resetGyroToAngle(0)

        # abs drive toggle
        if self.driveCtrlr.getLeftStickButtonPressed():
            self.hal.fieldOriented = not self.hal.fieldOriented

        if self.driveCtrlr.getYButtonPressed():
            self.hal.rotPIDsetpoint = 240
            self.hal.rotPIDToggle = True
        elif self.driveCtrlr.getXButtonPressed():
            self.hal.rotPIDsetpoint = 300
            self.hal.rotPIDToggle = True
        elif self.driveCtrlr.getAButtonPressed():
            self.hal.rotPIDsetpoint = 60
            self.hal.rotPIDToggle = True
        elif self.driveCtrlr.getBButtonPressed():
            self.hal.rotPIDsetpoint = 120
            self.hal.rotPIDToggle = True
        startCameraUpdate = wpilib.getTime()
        self.swerveDrive.updateOdometry(self.hal)  ##TODO: Remove - in RobotPeriodic
        self.table.putNumber(
            "odometry update Time", wpilib.getTime() - startCameraUpdate
        )
        if self.mechCtrlr.getAButtonPressed():
            self.swerveDrive.savePos(
                self.tempFidId, self.swerveDrive.odometry.getPose().rotation().radians()
            )

        self.swerveDrive.updateOdometry(self.hal)  ##TODO: Remove
        if self.mechCtrlr.getAButtonPressed():
            if self.photonCamera1.fiducialId > 0:
                self.swerveDrive.savePos(
                    self.photonCamera1.fiducialId,
                    self.swerveDrive.odometry.getPose().rotation().radians(),
                )
            elif self.photonCamera2.fiducialId > 0:
                self.swerveDrive.savePos(
                    self.photonCamera2.fiducialId,
                    self.swerveDrive.odometry.getPose().rotation().radians(),
                )
            else:
                self.swerveDrive.savePos(
                    0, self.swerveDrive.odometry.getPose().rotation().radians()
                )

        self.hal.publish(self.table)
        startCameraUpdate = wpilib.getTime()
        self.hardware.update(self.hal, self.time)
        self.table.putNumber(
            "hardware update Time", wpilib.getTime() - startCameraUpdate
        )

    def autonomousInit(self) -> None:
        self.hal.stopMotors()
        AUTO_SIDE_RED = "red"
        AUTO_SIDE_BLUE = "blue"
        self.onRedSide: bool = self.autoSideChooser.getSelected() == AUTO_SIDE_RED
        self.autoStartTime = wpilib.getTime()
        self.holonomicDriveController = PPHolonomicDriveController(
            PIDConstants(5, 0, 0, 0), PIDConstants(0.15, 0, 0, 0)
        )

        self.auto: dict[str, autoStages.AutoStage] = autoStages.chooseAuto(
            self.autoRoutineChooser.getSelected(), self
        )

        self.autoKeys = list(self.auto.keys())
        self.currentAuto = 0
        self.autoFinished = False

        self.table.putString("Chosen Auto is", "temp")

        if not self.currentAuto == len(self.autoKeys):  ## TDOO Fix

            self.auto[self.autoKeys[self.currentAuto]].autoInit(self)

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of autonomousPeriodic

        # self.swerveDrive.resetOdometry(self, Pose2d(0, 0, Rotation2d(radians(0))), self.hal)
        # self.swerveDrive.resetOdometry(Pose2d(), self.hal)

        if self.currentAuto >= len(self.autoKeys):
            self.autoFinished = True
        else:
            self.table.putString("Current Stage", self.autoKeys[self.currentAuto])
            self.table.putNumber("Current Stage Number", self.currentAuto)

        self.table.putBoolean("Auto finished", self.autoFinished)

        if not self.autoFinished:
            self.auto[self.autoKeys[self.currentAuto]].run(self)
            if self.auto[self.autoKeys[self.currentAuto]].isDone(self):
                self.currentAuto += 1
                if not self.currentAuto >= len(self.autoKeys):
                    self.auto[self.autoKeys[self.currentAuto]].autoInit(self)

        self.intakeChute.update(
            self.hal,
            False,
            False,
            False,
            False,
        )

        # if (wpilib.getTime() - self.autoStartTime) < 5:
        #     self.swerveDrive.updateWithoutSticks(self.hal, ChassisSpeeds(-0.25, 0, 0))
        # else:
        #     self.swerveDrive.updateWithoutSticks(self.hal, ChassisSpeeds(0, 0, 0))

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
