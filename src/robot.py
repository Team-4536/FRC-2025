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
from robotHAL import RobotHAL

class RobotInputs:
    def __init__(self) -> None:
        

        pass

    def update(self) -> None:
        pass


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.driveCtrlr = wpilib.XboxController(0)
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

        self.input = RobotInputs()

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.table.putNumber("leftJoyY", self.hal.driveVolts)
        self.hal.publish(self.table)
        self.input.update()
        self.hal.stopMotors()

    def teleopInit(self) -> None:
        
        pass

    def teleopPeriodic(self) -> None:
        self.hal.driveVolts = self.driveCtrlr.getLeftY()
        self.hal.stopMotors()
        self.hardware.update(self.hal, self.time)
        

        
    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal, self.time)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal, self.time)


if __name__ == "__main__":
    wpilib.run(Robot)
