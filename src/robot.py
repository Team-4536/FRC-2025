import wpilib
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from swerveDrive import SwerveDrive
from wpimath.units import radians
import rev
from rev import SparkMax
from time import sleep
import navx


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        sleep(1)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)

        self.swerveDrive: SwerveDrive = SwerveDrive()

        self.povPrev = 0

        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        self.yaw = self.gyro.getAngle()

    def robotPeriodic(self) -> None:

        # self.hal.publish(self.table)
        # self.hal.stopMotors()
        pass

    def teleopInit(self) -> None:
        self.setpointActiveLeft = False
        self.setpointActiveRight = False

        # self.hal.rotPIDToggle = False

    def teleopPeriodic(self) -> None:
        # self.hal.stopMotors()  # Keep this at the top of teleopPeriodic
        self.table.putNumber("Controller x", self.driveCtrlr.getLeftX())
        self.swerveDrive.update(
            self,
            self.driveCtrlr.getLeftX(),
            self.driveCtrlr.getLeftY(),
            self.driveCtrlr.getRightX(),
            self.driveCtrlr.getRightTriggerAxis(),
            self.driveCtrlr.getStartButtonPressed(),
        )

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        # self.hal.stopMotors()
        # self.hardware.update(self.hal)
        pass
