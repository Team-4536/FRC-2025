import math
from photonOdometry import photonVision
import rev
import robotHAL
import wpilib
import swerveDrive
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from simHAL import RobotSimHAL
import photonOdometry
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition

from robotHAL import RobotHAL, RobotHALBuffer
from swerveDrive import SwerveDrive
from wpimath.units import radians

import pathplannerlib  # type: ignore
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants  # type: ignore




class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()

        self.hardware.update(self.hal)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)

        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.povPrev = 0
        self.photonCamera1 = photonVision("Camera1", 30, 0.17938, 0.33337, 0.2889)
        self.photonCamera2 = photonVision("Camera2", -30, 0.11747, 0.33337, 0.2889)
        self.currentSetpointFidID = -1
        self.setpointActive = 0 #0 is no setpoints active, 1 is right setpoint active, 2 is left setpoint active

    def robotPeriodic(self) -> None:

        self.hal.publish(self.table)
        self.hal.stopMotors()
        self.photonCamera1.update()
        self.photonCamera2.update()
        if(self.photonCamera1.ambiguity < 0.15) and (self.photonCamera2.ambiguity < 0.15):
            self.comboCamX = (self.photonCamera1.robotX + self.photonCamera2.robotX)/2
            self.comboCamY = (self.photonCamera1.robotY + self.photonCamera2.robotY)/2
            self.comboCamTheta = (self.photonCamera1.robotAngle + self.photonCamera2.robotAngle)/2
            self.photonPose2d = Pose2d(self.comboCamX,self.comboCamY,self.comboCamTheta)
            self.swerveDrive.odometry.resetPose(self.photonPose2d)
        elif self.photonCamera1.ambiguity < 0.15:
            self.photonPose2d = Pose2d(
                self.photonCamera1.robotX,
                self.photonCamera1.robotY,
                self.photonCamera1.robotAngle,
            )
            self.swerveDrive.odometry.resetPose(self.photonPose2d)
        elif self.photonCamera2.ambiguity < 0.15:
            self.photonPose2d = Pose2d(
                self.photonCamera2.robotX,
                self.photonCamera2.robotY,
                self.photonCamera2.robotAngle,
            )
            self.swerveDrive.odometry.resetPose(self.photonPose2d)


    def teleopInit(self) -> None:
        self.setpointActiveLeft = False
        self.setpointActiveRight = False

        self.hal.rotPIDToggle = False

    def teleopPeriodic(self) -> None:
        self.hal.stopMotors()  # Keep this at the top of teleopPeriodic
        self.currYaw = self.swerveDrive.odometry.getPose().rotation().radians()
       # if not self.setpointActiveLeft and not self.setpointActiveRight:
        if(self.setpointActive == 0):
            self.swerveDrive.update(
                self.hal,
                self.driveCtrlr.getLeftX() * 0.5,
                self.driveCtrlr.getLeftY() * 0.5,
                self.driveCtrlr.getRightX()* 0.5,
                self.driveCtrlr.getRightTriggerAxis(),
                self.driveCtrlr.getStartButtonPressed(),
            )
        elif((self.setpointActive == 1) and (self.photonCamera2.TFID != -1)):
            self.swerveDrive.setpointChooser(self.currYaw, self.photonCamera2.TFID, "right")
            self.swerveDrive.updateWithoutSticks(self.hal, self.swerveDrive.adjustedSpeeds)
        elif((self.setpointActive == 2) and (self.photonCamera1.TFID != -1)):
            self.swerveDrive.setpointChooser(self.currYaw, self.photonCamera1.TFID, "left")
            self.swerveDrive.updateWithoutSticks(self.hal, self.swerveDrive.adjustedSpeeds)
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

        self.swerveDrive.updateOdometry(self.hal)
        

        self.hal.publish(self.table)

        self.hardware.update(self.hal)
       

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
