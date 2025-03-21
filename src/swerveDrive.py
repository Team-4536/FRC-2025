import math
import robotHAL
import robot
import math
import wpilib
from wpilib import SmartDashboard, Field2d
import numpy as np
from ntcore import NetworkTableInstance
from real import angleWrap
import wpimath.units
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import feetToMeters
from ntcore import NetworkTableInstance
import setpoints
from math import radians


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive:
    MAX_METERS_PER_SEC = 8.0  # stolen from lastyears code

    def __init__(self) -> None:
        self.setpointsTable = NetworkTableInstance.getDefault().getTable("setpoints")

        self.angle = Rotation2d(0)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.FMSData = NetworkTableInstance.getDefault().getTable("FMSInfo")

        oneftInMeters = feetToMeters(1)

        frontLeftLocation = Translation2d(oneftInMeters, oneftInMeters)
        frontRightLocation = Translation2d(oneftInMeters, -oneftInMeters)
        backLeftLocation = Translation2d(-oneftInMeters, oneftInMeters)
        backRightLocation = Translation2d(-oneftInMeters, -oneftInMeters)
        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        )

        # =======NEW, NOT TUNED=======================================
        constraints = TrapezoidProfileRadians.Constraints(4 * math.pi, 20 * math.pi)
        xPID = PIDController(0.1, 0, 0)
        yPID = PIDController(0.1, 0, 0)
        rotPID = ProfiledPIDControllerRadians(1.2, 0, 0, constraints)

        self.holonomicController = HolonomicDriveController(xPID, yPID, rotPID)
        self.yawOffset = 0.0

        # ============================================================
        self.OdomField = Field2d()
        self.table.putNumber("SD Joystick X offset", 0)
        self.table.putNumber("SD Joystick Y offset", 0)
        self.table.putNumber("SD Joystick Omega offset", 0)
        self.pose = Pose2d(
            wpimath.units.meters(0), wpimath.units.meters(0), wpimath.units.radians(0)
        )
        self.controller = HolonomicDriveController(
            PIDController(5, 0, 0),
            PIDController(5, 0, 0),
            ProfiledPIDControllerRadians(
                1.2, 0, 0, TrapezoidProfileRadians.Constraints(6.28, 3.14)
            ),
        )
        ModulePos = SwerveModulePosition(0, Rotation2d(0))
        modulePosList: list[SwerveModulePosition * 4] = [  # type: ignore
            ModulePos,
            ModulePos,
            ModulePos,
            ModulePos,
        ]
        self.odometry = SwerveDrive4Odometry(
            self.kinematics, self.angle, tuple(modulePosList), self.pose
        )

    def resetOdometry(self, pose: Pose2d, hal: robotHAL.RobotHALBuffer):

        modulePosList = (
            SwerveModulePosition(
                hal.drivePositionsList[0], Rotation2d(radians(hal.steerPositionList[0]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[1], Rotation2d(radians(hal.steerPositionList[1]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[2], Rotation2d(radians(hal.steerPositionList[2]))
            ),
            SwerveModulePosition(
                hal.drivePositionsList[3], Rotation2d(radians(hal.steerPositionList[3]))
            ),
        )
        # modulePosList = (hal.moduleFL, hal.moduleFR, hal.moduleBL, hal.moduleBR)
        self.odometry.resetPosition(Rotation2d(hal.yaw), modulePosList, pose)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(
        self,
        hal: robotHAL.RobotHALBuffer,
        joystickX: float,
        joystickY: float,
        joystickRotation: float,
        RTriggerScalar: float,
        resetOffset: bool,
    ):
        self.table.putNumber("Drive Ctrl X", joystickX)
        self.table.putNumber("Drive Ctrl Y", joystickY)
        self.table.putNumber("Drive Ctrl Rotation", joystickRotation)

        if math.sqrt(joystickX**2 + joystickY**2) < 0.08:
            joystickX = 0
            joystickY = 0
        if abs(joystickRotation) < 0.06:
            joystickRotation = 0

        self.offsetX = 0.05 * np.sign(joystickX)
        self.offsetY = 0.05 * np.sign(joystickY)
        self.offsetR = 0.05 * np.sign(joystickRotation)

        self.proxyDeadZoneX = (joystickX - self.offsetX) * 3.5
        self.proxyDeadZoneY = (joystickY - self.offsetY) * 3.5
        self.proxyDeadZoneR = (joystickRotation - self.offsetR) * 10

        # the controller's x axis the the ChassisSpeeds' y axis and same for the other x and y axies
        # the signes are flipped for the differences too
        self.driveY = -self.proxyDeadZoneX
        self.driveX = -self.proxyDeadZoneY
        self.driveRotation = -self.proxyDeadZoneR

        driveVector = Translation2d(self.driveX, self.driveY)

        if resetOffset:
            self.yawOffset = hal.yaw

        self.table.putNumber("absDriveOffset", self.yawOffset)

        # abs drive toggle
        if hal.fieldOriented:
            driveVector = driveVector.rotateBy(Rotation2d(-hal.yaw + self.yawOffset))

        # disable rotatioanl PID if turn stick is moved
        if self.driveRotation != 0:
            hal.rotPIDToggle = False

        self.table.putNumber("z_PID Setpoint", hal.rotPIDsetpoint)
        self.table.putBoolean("z_Absolute Drive", hal.fieldOriented)

        # --------------EMMETT'S SCARY NEW STUFF-----------------------------------
        rotPos = Rotation2d(hal.yaw)
        fakeBotPos = Pose2d(0, 0, rotPos)
        rotTarget = Rotation2d.fromDegrees(hal.rotPIDsetpoint)

        # returns chassis speeds
        adjustedSpeeds = self.holonomicController.calculate(
            fakeBotPos, fakeBotPos, 0, rotTarget
        )
        # take only rotational speed
        rotPIDSpeed = adjustedSpeeds.omega

        # only use rotational PID if it's activated
        if hal.rotPIDToggle:
            rotFinal = rotPIDSpeed * 5
        else:
            rotFinal = self.driveRotation  # copied from HCPA code

        # -------------------------------------------------------------------

        self.chassisSpeeds = ChassisSpeeds(
            driveVector.X() * 0.5 * 4**RTriggerScalar,
            driveVector.Y() * 0.5 * 4**RTriggerScalar,
            rotFinal,
        )

        self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
        self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
        self.table.putNumber(
            "SD ChassisSpeeds omega (rotFinal)", self.chassisSpeeds.omega
        )
        self.table.putNumber(
            "SD RotPIDSpeed omega (adjustedSpeedsOmega)",
            adjustedSpeeds.omega,  # * (180 / math.pi)
        )
        self.table.putBoolean("rotPIDToggle", hal.rotPIDToggle)
        self.table.putNumber("z_target rotDeg", rotTarget.degrees())
        self.table.putNumber("z_current rotDeg", fakeBotPos.rotation().degrees())

        self.unleashedModules = self.kinematics.toSwerveModuleStates(self.chassisSpeeds)
        swerveModuleStates = self.kinematics.desaturateWheelSpeeds(
            self.unleashedModules,
            self.MAX_METERS_PER_SEC,
        )

        self.table.putNumber(
            "SD Module Original Turn Setpoint", swerveModuleStates[0].angle.radians()
        )

        FLModuleState = self.optimizeTarget(
            swerveModuleStates[0], Rotation2d(hal.turnCCWFL)
        )
        hal.driveFLSetpoint = FLModuleState.speed
        hal.turnFLSetpoint = FLModuleState.angle.radians()

        FRModuleState = self.optimizeTarget(
            swerveModuleStates[1], Rotation2d(hal.turnCCWFR)
        )
        hal.driveFRSetpoint = FRModuleState.speed
        hal.turnFRSetpoint = FRModuleState.angle.radians()

        BLModuleState = self.optimizeTarget(
            swerveModuleStates[2], Rotation2d(hal.turnCCWBL)
        )
        hal.driveBLSetpoint = BLModuleState.speed
        hal.turnBLSetpoint = BLModuleState.angle.radians()

        BRModuleState = self.optimizeTarget(
            swerveModuleStates[3], Rotation2d(hal.turnCCWBR)
        )
        hal.driveBRSetpoint = BRModuleState.speed
        hal.turnBRSetpoint = BRModuleState.angle.radians()

    def updateOdometry(self, hal: robotHAL.RobotHALBuffer):

        modulePosList = (
            SwerveModulePosition(
                hal.drivePositionsList[0], Rotation2d(hal.steerPositionList[0])
            ),
            SwerveModulePosition(
                hal.drivePositionsList[1], Rotation2d(hal.steerPositionList[1])
            ),
            SwerveModulePosition(
                hal.drivePositionsList[2], Rotation2d(hal.steerPositionList[2])
            ),
            SwerveModulePosition(
                hal.drivePositionsList[3], Rotation2d(hal.steerPositionList[3])
            ),
        )
        self.odometry.update(Rotation2d(hal.yaw), modulePosList)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.odomPos = [self.odometry.getPose().X(), self.odometry.getPose().Y()]
        self.OdomField.setRobotPose(self.odometry.getPose())

        self.table.putNumber("odomX", self.odomPos[0])
        self.table.putNumber("odomy", self.odomPos[1])
        wpilib.SmartDashboard.putData("Odom", self.OdomField)

    def optimizeTarget(
        self, target: SwerveModuleState, moduleAngle: Rotation2d
    ) -> SwerveModuleState:

        error = angleWrap(target.angle.radians() - moduleAngle.radians())

        outputSpeed = target.speed
        outputAngle = target.angle.radians()

        # optimize
        if abs(error) > math.pi / 2:
            outputAngle = outputAngle + math.pi
            outputSpeed = -outputSpeed

        # return
        outputAngleRot2d = Rotation2d(angleWrap(outputAngle))
        output = SwerveModuleState(outputSpeed, outputAngleRot2d)

        return output

    def setpointChooser(self, yaw, fiducialID, side):

        self.currentPose = Pose2d(self.odomPos[0], self.odomPos[1], yaw)

        if side == "left":
            self.rot = Rotation2d(setpoints.tagLeft[fiducialID][2])
            self.desiredPose = Pose2d(
                setpoints.tagLeft[fiducialID][0],
                setpoints.tagLeft[fiducialID][1],
                self.rot,
            )

        elif side == "right":
            self.rot = Rotation2d(setpoints.tagRight[fiducialID][2])

            self.desiredPose = Pose2d(
                setpoints.tagRight[fiducialID][0],
                setpoints.tagRight[fiducialID][1],
                self.rot,
            )
        else:
            self.desiredPose = Pose2d(0, 0, 0)

        if not (setpoints.tagRight[fiducialID][0] == 0) and not (
            setpoints.tagRight[fiducialID][1] == 0
        ):
            self.adjustedSpeeds = self.controller.calculate(
                self.currentPose, self.desiredPose, 0, self.rot
            )

    def updateWithoutSticks(
        self, hal: robotHAL.RobotHALBuffer, chassisSpeed: ChassisSpeeds
    ):

        self.chassisSpeeds = chassisSpeed

        self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
        self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
        self.table.putNumber("SD ChassisSpeeds omega", self.chassisSpeeds.omega)

        self.unleashedModules = self.kinematics.toSwerveModuleStates(self.chassisSpeeds)
        swerveModuleStates = self.kinematics.desaturateWheelSpeeds(
            self.unleashedModules,
            self.MAX_METERS_PER_SEC,
        )

        self.table.putNumber(
            "SD Original Turn Setpoint", swerveModuleStates[0].angle.radians()
        )

        self.table.putNumber("SD Original Drive Setpoint", swerveModuleStates[0].speed)

        FLModuleState = self.optimizeTarget(
            swerveModuleStates[0], Rotation2d(hal.turnCCWFL)
        )
        hal.driveFLSetpoint = FLModuleState.speed
        hal.turnFLSetpoint = FLModuleState.angle.radians()
        self.table.putNumber("SD Opimized Turn Setpoint", FLModuleState.angle.radians())

        FRModuleState = self.optimizeTarget(
            swerveModuleStates[1], Rotation2d(hal.turnCCWFR)
        )
        hal.driveFRSetpoint = FRModuleState.speed
        hal.turnFRSetpoint = FRModuleState.angle.radians()

        BLModuleState = self.optimizeTarget(
            swerveModuleStates[2], Rotation2d(hal.turnCCWBL)
        )
        hal.driveBLSetpoint = BLModuleState.speed
        hal.turnBLSetpoint = BLModuleState.angle.radians()

        BRModuleState = self.optimizeTarget(
            swerveModuleStates[3], Rotation2d(hal.turnCCWBR)
        )
        hal.driveBRSetpoint = BRModuleState.speed
        hal.turnBRSetpoint = BRModuleState.angle.radians()

    def savePos(self, fiducialID: int, yaw: float):
        with open("/home/lvuser/photon.txt", "a") as f:
            f.write("match: " + str(self.FMSData.getNumber("MatchNumber", 0)) + " ")
            if self.FMSData.getBoolean("IsRedAlliance", True):
                f.write("Red")
            else:
                f.write("Blue")
            f.write(
                " "
                + str(self.FMSData.getNumber("StationNumber", 0))
                + " tag: "
                + str(fiducialID)
                + " X: "
                + f"{self.odomPos[0]}"
                + " Y: "
                + f"{self.odomPos[1]}"
                + " Rot: "
                + f"{yaw}"
                + "   -->   "
                + " Time: "
                + f"{wpilib.getTime()}"
                "\n"
            )
