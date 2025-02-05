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

        # self.elevatorSubsystem = ElevatorSubsystem()
        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.table.putNumber("FR Turn Setpoint", 0)
        self.table.putNumber("FL Turn Setpoint", 0)
        self.table.putNumber("BR Turn Setpoint", 0)
        self.table.putNumber("BL Turn Setpoint", 0)

        self.table.putNumber("FR Drive Setpoint", 0)
        self.table.putNumber("FL Drive Setpoint", 0)
        self.table.putNumber("BR Drive Setpoint", 0)
        self.table.putNumber("BL Drive Setpoint", 0)

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.table.putNumber("leftJoyY", self.hal.driveVolts)
        self.hal.publish(self.table)
        self.hal.stopMotors()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic

        # selfelevatorSubsystem.update(
        #     self.hal,
        #     self.mechCtrlr.getRightTriggerAxis(),
        #     self.mechCtrlr.getLeftTriggerAxis(),
        # )
        self.swerveDrive.update(
            self.hal,
            self.driveCtrlr.getLeftX(),
            -self.driveCtrlr.getLeftY(),
            self.driveCtrlr.getRightX(),
        )

        # self.hal.turnFRSetpoint = self.table.getNumber("FR Turn Setpoint", 0)
        # self.hal.turnFLSetpoint = self.table.getNumber("FL Turn Setpoint", 0)
        # self.hal.turnBRSetpoint = self.table.getNumber("BR Turn Setpoint", 0)
        # self.hal.turnBLSetpoint = self.table.getNumber("BL Turn Setpoint", 0)

        # self.hal.driveFRSetpoint = self.table.getNumber("FR Drive Setpoint", 0)
        # self.hal.driveFLSetpoint = self.table.getNumber("FL Drive Setpoint", 0)
        # self.hal.driveBRSetpoint = self.table.getNumber("BR Drive Setpoint", 0)
        # self.hal.driveBLSetpoint = self.table.getNumber("BL Drive Setpoint", 0)

        if self.mechCtrlr.getAButton():
            self.hal.manipulatorVolts = 4
        elif self.mechCtrlr.getBButton():
            self.hal.manipulatorVolts = -4
        else:
            self.hal.manipulatorVolts = 0

        # Keep the lines below at the bottom of teleopPeriodic
        self.hal.publish(self.table)
        self.hardware.update(self.hal, self.time)

    -0.298

    -5.798

    -0.284

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
