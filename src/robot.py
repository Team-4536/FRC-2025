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
from profiler import Profiler


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
        # self.hardware = robotHAL.RobotHAL()

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

        self.swerveDrive: SwerveDrive = SwerveDrive(self.isSimulation())
        self.swerveDrive.resetOdometry(Pose2d(), self.hal, -1)

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

        self.driverOffset = 0
        self.cam1TFID = -1
        self.cam2TFID = -1

        self.hardwareProfiler = Profiler("HardwareUpdate")
        self.swerveDriveProfiler = Profiler("SwerveDriveUpdate")
        self.elevatorProfiler = Profiler("ElevatorUpdate")
        self.intakeChuteProfiler = Profiler("IntakeChuteUpdate")
        self.manipulatorProfiler = Profiler("ManipulatorUpdate")
        self.cameraProfiler = Profiler("CameraUpdate")

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish()
        self.hal.stopMotors()

        self.ledSignals.update(
            self.manipulatorSubsystem,
            self.elevatorSubsystem,
            self.intakeChute,
            self.hal.elevatorPos,
        )

        self.photonCamera1.update()
        self.photonCamera2.update()
        if self.photonCamera1.ambiguity < 0.02:
            self.photonPose2d = Pose2d(
                self.photonCamera1.robotX,
                self.photonCamera1.robotY,
                self.photonCamera1.robotAngle,
            )
            self.swerveDrive.resetOdometry(
                self.photonPose2d, self.hal, self.photonCamera1.ambiguity
            )
        if self.photonCamera2.ambiguity < 0.02:
            self.photonPose2d = Pose2d(
                self.photonCamera2.robotX,
                self.photonCamera2.robotY,
                self.photonCamera2.robotAngle,
            )
            self.swerveDrive.resetOdometry(
                self.photonPose2d, self.hal, self.photonCamera2.ambiguity
            )

        self.swerveDrive.updateOdometry(self.hal)

    def teleopInit(self) -> None:
        # self.swerveDrive.resetOdometry(Pose2d(), self.hal, -1)
        self.hardware.resetGyroToAngle(
            self.swerveDrive.odometry.getPose().rotation().radians()
        )
        self.setpointActiveLeft = False
        self.setpointActiveRight = False

        self.hal.rotPIDToggle = False
        AUTO_SIDE_RED = "red"
        self.onRedSide = self.autoSideChooser.getSelected() == AUTO_SIDE_RED
        if self.onRedSide:
            self.swerveDrive.yawOffset = math.pi
        else:
            self.swerveDrive.yawOffset = 0

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

        self.swerveDriveProfiler.start()
        self.swerveDrive.update(
            self.hal,
            self.driveCtrlr.getLeftX(),
            self.driveCtrlr.getLeftY(),
            self.driveCtrlr.getRightX(),
            self.driveCtrlr.getRightTriggerAxis(),
            self.driveCtrlr.getStartButtonPressed(),
            self.driveCtrlr.getLeftBumperButton(),
            self.driveCtrlr.getRightBumperButton(),
            self.mechCtrlr.getAButton(),
            self.photonCamera1,
            self.photonCamera2,
        )
        self.swerveDriveProfiler.end()

        self.elevatorProfiler.start()
        self.elevatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getRightTriggerAxis(),
            self.mechCtrlr.getLeftTriggerAxis(),
            self.mechCtrlr.getYButtonPressed(),
            self.mechCtrlr.getPOV(),
            self.mechCtrlr.getXButtonPressed(),
            self.mechCtrlr.getBButton(),
        )
        self.elevatorProfiler.end()

        # convert POV buttons to bool values (sorry michael this code may be hard to look at)
        self.cameraProfiler.start()
        self.povLeftPressed = False
        self.povRightPressed = False
        if self.driveCtrlr.getPOV() == 270 and self.povPrev != 270:
            self.povLeftPressed = True
        if self.driveCtrlr.getPOV() == 90 and self.povPrev != 90:
            self.povRightPressed = True
        self.povPrev = self.driveCtrlr.getPOV()
        self.cameraProfiler.end()

        self.intakeChuteProfiler.start()
        self.intakeChute.update(
            self.hal,
            self.driveCtrlr.getPOV() == 180,
            self.driveCtrlr.getPOV() == 0,
            self.povRightPressed,
            self.povLeftPressed,
        )
        self.intakeChuteProfiler.end()

        self.manipulatorProfiler.start()
        self.manipulatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getAButton(),
            self.mechCtrlr.getLeftBumperPressed(),
        )
        self.manipulatorProfiler.end()

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
        # self.swerveDrive.updateOdometry(self.hal)

        self.hal.publish()
        self.hardwareProfiler.start()
        self.hardware.update(self.hal, self.time)
        self.hardwareProfiler.end()

    def autonomousInit(self) -> None:
        self.hal.stopMotors()
        AUTO_SIDE_RED = "red"
        AUTO_SIDE_BLUE = "blue"
        self.onRedSide: bool = self.autoSideChooser.getSelected() == AUTO_SIDE_RED
        self.autoStartTime = wpilib.getTime()
        self.holonomicDriveController = PPHolonomicDriveController(
            PIDConstants(9.5, 0, 0, 0), PIDConstants(3, 0, 0, 0)
        )

        self.auto: dict[str, autoStages.AutoStage] = autoStages.chooseAuto(
            self.autoRoutineChooser.getSelected(), self
        )

        self.autoKeys = list(self.auto.keys())
        self.currentAuto = 0
        self.autoFinished = False

        if not self.currentAuto == len(self.autoKeys):  ## TDOO Fix

            self.auto[self.autoKeys[self.currentAuto]].autoInit(self)
        # Keep the lines below at the bottom of teleopPeriodic
        self.hal.publish()

        self.hardwareProfiler.start()
        self.hardware.update(self.hal, self.time)
        self.hardwareProfiler.end()

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of autonomousPeriodic

        # self.swerveDrive.resetOdometry(self, Pose2d(0, 0, Rotation2d(radians(0))), self.hal, -1)
        # self.swerveDrive.resetOdometry(Pose2d(), self.hal,-1)

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

        self.intakeChuteProfiler.start()
        self.intakeChute.update(
            self.hal,
            False,
            False,
            False,
            False,
        )
        self.intakeChuteProfiler.end()

        # if (wpilib.getTime() - self.autoStartTime) < 5:
        #     self.swerveDrive.updateWithoutSticks(self.hal, ChassisSpeeds(-0.25, 0, 0))
        # else:
        #     self.swerveDrive.updateWithoutSticks(self.hal, ChassisSpeeds(0, 0, 0))

        self.hardwareProfiler.start()
        self.hardware.update(
            self.hal, self.time
        )  # Keep this at the bottom of autonomousPeriodic
        self.hardwareProfiler.end()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardwareProfiler.start()
        self.hardware.update(self.hal, self.time)
        self.hardwareProfiler.end()


if __name__ == "__main__":
    wpilib.run(Robot)
