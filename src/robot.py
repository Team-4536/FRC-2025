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
from manipulator import ManipulatorSubsystem
from IntakeChute import IntakeChute
from profiler import Profiler


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

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

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.elevatorSubsystem = ElevatorSubsystem()
        self.manipulatorSubsystem = ManipulatorSubsystem()
        self.intakeChute = IntakeChute()

        self.hardwareProfiler = Profiler("HardwareUpdate")
        self.swerveDriveProfiler = Profiler("SwerveDriveUpdate")
        self.elevatorProfiler = Profiler("ElevatorUpdate")
        self.intakeChuteProfiler = Profiler("IntakeChuteUpdate")
        self.manipulatorProfiler = Profiler("ManipulatorUpdate")

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish()
        self.hal.stopMotors()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

        self.swerveDriveProfiler.start()
        self.swerveDrive.update(
            self.hal,
            self.driveCtrlr.getLeftX(),
            -self.driveCtrlr.getLeftY(),
            self.driveCtrlr.getRightX(),
            self.driveCtrlr.getRightTriggerAxis(),
        )
        self.swerveDriveProfiler.end()

        self.elevatorProfiler.start()
        self.elevatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getRightTriggerAxis(),
            self.mechCtrlr.getLeftTriggerAxis(),
            self.mechCtrlr.getYButtonPressed(),
            self.mechCtrlr.getPOV(),
            self.mechCtrlr.getXButton(),
            self.mechCtrlr.getBButton(),
        )
        self.elevatorProfiler.end()

        self.intakeChuteProfiler.start()
        self.intakeChute.update(
            self.hal,
            self.driveCtrlr.getLeftBumper(),
            self.driveCtrlr.getRightBumper(),
            self.driveCtrlr.getBButtonPressed(),
            self.driveCtrlr.getYButtonPressed(),
        )
        self.intakeChuteProfiler.end()

        self.manipulatorProfiler.start()
        self.manipulatorSubsystem.update(
            self.hal,
            self.mechCtrlr.getAButton(),
            self.mechCtrlr.getLeftBumperPressed(),
        )
        self.manipulatorProfiler.end()

        if self.driveCtrlr.getStartButton():
            self.hardware.resetGyroToAngle(0)

        # Keep the lines below at the bottom of teleopPeriodic
        self.hal.publish()

        self.hardwareProfiler.start()
        self.hardware.update(self.hal, self.time)
        self.hardwareProfiler.end()

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of autonomousPeriodic

        self.intakeChuteProfiler.start()
        self.intakeChute.update(
            self.hal,
            False,
            False,
            False,
            False,
        )
        self.intakeChuteProfiler.end()

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
