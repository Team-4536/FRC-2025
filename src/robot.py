import math

import robotHAL
import wpilib
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from timing import TimeData
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from ringsubsystem import ringSubsystem



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

        self.ringSubsystem: ringSubsystem = ringSubsystem()

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)
        self.mechCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)
        

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.ringSensorValue = False
        self.limitSwitchValue = False
        self.rollerVoltage = 0
        self.infraredSensorValue = False
        
        
        self.hal.stopMotors()

        

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()
        self.mechCtrlr.getYButton()
        self.mechCtrlr.getXButton()
        
        if self.mechCtrlr.getBButton():
            self.hal.rollerVoltage = 0.75
        else:
            self.hal.rollerVoltage = 0

        self.ringSubsystem.update(self.hal, self.mechCtrlr.getYButton())
        self.ringSubsystem.update(self.hal, self.mechCtrlr.getXButton())

        self.table.putNumber("Roller Voltage", self.hal.rollerVoltage)

        self.hal.publish(self.table)
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