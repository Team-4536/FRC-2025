import math
import robotHAL
import robot
import math
import wpilib
from wpilib import SmartDashboard, Field2d
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
from wpimath.units import radians
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Rotation2d
from wpimath.geometry import Pose2d


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive:

    MAX_METERS_PER_SEC = 4.0  # stolen from lastyears code

    def __init__(self) -> None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        oneftInMeters = 0.3048
        self.OdomField = Field2d()
        self.controller = HolonomicDriveController(
            PIDController(0.1, 0, 0),
            PIDController(0.1, 0, 0),
            ProfiledPIDControllerRadians(
                1, 0, 0, TrapezoidProfileRadians.Constraints(6.28, 3.14)
            ),
        )

        # self.modulePositions: list[Translation2d] = [
        #     Translation2d(oneftInMeters, oneftInMeters),
        #     Translation2d(oneftInMeters, -oneftInMeters),
        #     Translation2d(-oneftInMeters, oneftInMeters),
        #     Translation2d(-oneftInMeters, -oneftInMeters),
        # ]
        self.modulePositions: list[Translation2d] = [
            Translation2d(-oneftInMeters, oneftInMeters),
            Translation2d(oneftInMeters, oneftInMeters),
            Translation2d(-oneftInMeters, -oneftInMeters),
            Translation2d(oneftInMeters, -oneftInMeters),
        ]
        # ModuleState = SwerveModuleState(wpimath.units.meters(0), Rotation2d(0))
        ModulePos = SwerveModulePosition(0, Rotation2d(0))
        modulePosList = [ModulePos, ModulePos, ModulePos, ModulePos]

        self.angle = Rotation2d(0)
        self.pose = Pose2d(
            wpimath.units.meters(0), wpimath.units.meters(0), wpimath.units.radians(0)
        )
        self.kinematics = SwerveDrive4Kinematics(*self.modulePositions)
        self.odometry = SwerveDrive4Odometry(
            self.kinematics, self.angle, modulePosList, self.pose
        )

        self.table.putNumber("SD Joystick X offset", 0)
        self.table.putNumber("SD Joystick Y offset", 0)
        self.table.putNumber("SD Joystick Omega offset", 0)

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
    ):
        self.table.putNumber("Drive Ctrl X", joystickX)
        self.table.putNumber("Drive Ctrl Y", joystickY)
        self.table.putNumber("Drive Ctrl Rotation", joystickRotation)

        if abs(joystickX) < 0.05:
            joystickX = 0
        if abs(joystickY) < 0.05:
            joystickY = 0
        if abs(joystickRotation) < 0.05:
            joystickRotation = 0

        self.driveX = joystickX * 2.5 + self.table.getNumber("SD Joystick X offset", 0)
        self.driveY = joystickY * 2.5 + self.table.getNumber("SD Joystick Y offset", 0)
        self.driveRotation = joystickRotation * -2.5 + self.table.getNumber(
            "SD Joystick Omega offset", 0
        )

        self.chassisSpeeds = ChassisSpeeds(self.driveX, self.driveY, self.driveRotation)

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

    def updateOdometry(self, hal: robotHAL.RobotHALBuffer):

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

    def setpointChooser(self, yaw):
        self.currentPose = Pose2d(self.odomPos[0], self.odomPos[1], yaw)
        self.desiredPose = Pose2d(0, 1, yaw)
        self.adjustedSpeeds = self.controller.calculate(
            self.currentPose, self.desiredPose, 0.25, Rotation2d.fromDegrees(0.0)
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
