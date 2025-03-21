import math
import rev
import robotHAL
import wpilib
import swerveDrive
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
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
        self.hardware = robotHAL.RobotHAL()

        self.hardware.update(self.hal, self.time)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)
        self.mechCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)  ## Shouldn't this already be done in SwerveDrive.__init__?

        self.povPrev = 0  # probably don't need this.

    def robotPeriodic(self) -> None:
        self.swerveDrive.updateOdometry(self.hal)

    def teleopInit(self) -> None:
        self.swerveDrive.resetOdometry(Pose2d(), self.hal)  # not sure this is a good idea

    def teleopPeriodic(self) -> None:
        self.swerveDrive.update(
            self.hal,
            self.driveCtrlr.getLeftX(),
            self.driveCtrlr.getLeftY(),
            self.driveCtrlr.getRightX(),
            self.driveCtrlr.getRightTriggerAxis(),
            self.driveCtrlr.getStartButtonPressed(),
        )

        # if self.driveCtrlr.getStartButton():  # This resets the field centric drive
        #     self.hardware.resetGyroToAngle(0)

        # abs drive toggle
        if self.driveCtrlr.getLeftStickButtonPressed():
            self.hal.fieldOriented = not self.hal.fieldOriented

        self.swerveDrive.updateOdometry(self.hal)
        self.hal.publish(self.table)
        self.hardware.update(self.hal, self.time)

        # self.hal.publish(self.table)
        # self.hardware.update(self.hal, self.time)

    def autonomousPeriodic(self) -> None:
        # self.swerveDrive.update(
        #     self.hal,
        #     0,
        #     0.2, 
        #     0,
        #     0,
        #     False,
        # )
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

if __name__ == "__main__":
    wpilib.run(Robot)
