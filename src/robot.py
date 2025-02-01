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

        self.elevatorSubsystem = ElevatorSubsystem()
        self.swerveDrive: SwerveDrive = SwerveDrive()

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.table.putNumber("leftJoyY", self.hal.driveVolts)
        self.hal.publish(self.table)
        self.hal.stopMotors()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

        self.elevatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getRightTriggerAxis(),
            self.mechCtrlr.getLeftTriggerAxis(),
        )
        self.swerveDrive.update(
            self.hal,
            self.driveCtrlr.getLeftX(),
            self.driveCtrlr.getLeftY(),
            self.driveCtrlr.getRightX(),
        )

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
