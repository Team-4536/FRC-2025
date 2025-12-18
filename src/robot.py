import robotHAL
import wpilib
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from robotHAL import RobotHAL, RobotHALBuffer
from swerveDrive import SwerveDrive
from wpimath.units import radians


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()

        self.hardware.update(self.hal)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.povPrev = 0

    def robotPeriodic(self) -> None:

        self.hal.publish(self.table)
        self.hal.stopMotors()

    def teleopInit(self) -> None:
        self.setpointActiveLeft = False
        self.setpointActiveRight = False

        self.hal.rotPIDToggle = False

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

       # if not self.setpointActiveLeft and not self.setpointActiveRight:
        self.swerveDrive.update(
            self.hal,
            self.driveCtrlr.getLeftX() * 0.1,
            self.driveCtrlr.getLeftY() * 0.1,
            self.driveCtrlr.getRightX()* 0.1,
            self.driveCtrlr.getRightTriggerAxis(),
            self.driveCtrlr.getStartButtonPressed(),
        )

        # if (
        #     abs(self.driveCtrlr.getLeftX()) > 0.07
        #     or abs(self.driveCtrlr.getLeftY()) > 0.07
        #     or abs(self.driveCtrlr.getRightX()) > 0.07
        #     or abs(self.driveCtrlr.getRightY()) > 0.07
        # ):
        #     self.setpointActiveLeft = False
        #     self.setpointActiveRight = False
            # self.tempFidId = -1

        # convert POV buttons to bool values (sorry michael this code may be hard to look at)

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

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)


if __name__ == "__main__":
    wpilib.run(Robot)
