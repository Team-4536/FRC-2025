import robotHAL
import wpilib
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from timing import TimeData
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from robotHAL import RobotHAL, RobotHALBuffer
from swerveDrive import SwerveDrive
from wpimath.units import radians


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

        self.driveCtrlr = wpilib.XboxController(0)

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.povPrev = 0

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.swerveDrive.updateOdometry(self.hal)
        self.hal.stopMotors()

    def teleopInit(self) -> None:
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)
        self.setpointActiveLeft = False
        self.setpointActiveRight = False

        self.hal.rotPIDToggle = False

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

        if not self.setpointActiveLeft and not self.setpointActiveRight:
            self.swerveDrive.update(
                self.hal,
                self.driveCtrlr.getLeftX(),
                self.driveCtrlr.getLeftY(),
                self.driveCtrlr.getRightX(),
                self.driveCtrlr.getRightTriggerAxis(),
                self.driveCtrlr.getStartButtonPressed(),
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
        if self.driveCtrlr.getStartButton():
            self.hardware.resetGyroToAngle(0)

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
        self.swerveDrive.updateOdometry(self.hal)
        self.table.putNumber(
            "odometry update Time", wpilib.getTime() - startCameraUpdate
        )
        if self.mechCtrlr.getAButtonPressed():
            self.swerveDrive.savePos(
                self.tempFidId, self.swerveDrive.odometry.getPose().rotation().radians()
            )

        self.swerveDrive.updateOdometry(self.hal)
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
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal, self.time)


if __name__ == "__main__":
    wpilib.run(Robot)
