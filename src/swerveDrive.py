import math
from robotHAL import RobotHALBuffer
import robot
import math
import wpilib
from wpilib import SmartDashboard, Field2d
import numpy as np
from photonOdometry import photonVision
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
from wpimath.units import feetToMeters, radians
from ntcore import NetworkTableInstance
from wpimath.units import feetToMeters
from ntcore import NetworkTableInstance
import setpoints
from profiler import Profiler

# from math import radians


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive:
    MAX_METERS_PER_SEC = 8.0  # stolen from lastyears code
    DESIRED_TX: float = 160  # for limelights
    ALLOWED_ERROR_LL = 5

    def __init__(self) -> None:
        self.setpointsTable = NetworkTableInstance.getDefault().getTable("setpoints")

        self.angle = Rotation2d(0)
        self.FMSData = NetworkTableInstance.getDefault().getTable("FMSInfo")
        self.table = (
            NetworkTableInstance.getDefault()
            .getTable("telemetry")
            .getSubTable("Swerve Drive Subsystem")
        )
        self.debugMode = False

        oneftInMeters = feetToMeters(1)

        frontLeftLocation = Translation2d(oneftInMeters, oneftInMeters)
        frontRightLocation = Translation2d(oneftInMeters, -oneftInMeters)
        backLeftLocation = Translation2d(-oneftInMeters, oneftInMeters)
        backRightLocation = Translation2d(-oneftInMeters, -oneftInMeters)
        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        )
        self.OdomField = Field2d()

        # =======NEW, NOT TUNED=======================================
        constraints = TrapezoidProfileRadians.Constraints(4 * math.pi, 20 * math.pi)
        xPID = PIDController(0, 0, 0)
        yPID = PIDController(0, 0, 0)
        rotPID = ProfiledPIDControllerRadians(1.4, 0, 0, constraints)

        self.holonomicController = HolonomicDriveController(xPID, yPID, rotPID)
        self.yawOffset = 0.0

        # ============================================================
        self.OdomField = Field2d()
        # self.table.putNumber("SD Joystick X offset", 0)
        # self.table.putNumber("SD Joystick Y offset", 0)
        # self.table.putNumber("SD Joystick Omega offset", 0)
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
        # self.table.putBoolean("Swerve Drive Debug Mode", False)

        self.desaturateSpeedsProfiler = Profiler("Desaturate Wheel Speeds")
        self.optimizeTargetProfiler = Profiler("Optimize Target")
        self.swerveSticksMath = Profiler("Swerve Stick Math")
        # ==============================================================
        self.llTable = NetworkTableInstance.getDefault().getTable("limelight")
        self.reefPIDController = PIDController(0.01, 0, 0)
        self.adjustedSpeeds = self.controller.calculate(
            self.pose, self.pose, 0, self.pose.rotation()
        )

    def resetOdometry(self, pose: Pose2d, hal: RobotHALBuffer, ambiguity):

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
        # if ambiguity == 0:
        #     hal.newYaw = -pose.rotation().degrees()
        self.odometry.resetPosition(Rotation2d(hal.yaw), modulePosList, pose)

    def update(
        self,
        hal: RobotHALBuffer,
        joystickX: float,
        joystickY: float,
        joystickRotation: float,
        RTriggerScalar: float,
        resetOffset: bool,
        setPointLeft: bool,
        setPointRight: bool,
        savePos: bool,
        cam1: photonVision,
        cam2: photonVision,
    ):
        self.debugMode = self.table.getBoolean("Swerve Drive Debug Mode", False)

        camera1TFID = cam1.TFID
        camera2TFID = cam2.TFID

        if setPointLeft:

            setpointActiveLeft = True
            setpointActiveRight = False
            # self.tempFidId = camera1.fiducialId
            if camera2TFID > -1:
                tempFidId = camera2TFID
            elif camera1TFID > -1:
                tempFidId = camera1TFID
            else:
                setpointActiveLeft = False
        else:
            setpointActiveLeft = False
        if setPointRight:
            setpointActiveRight = True
            setpointActiveLeft = False
            if camera1TFID > -1:
                tempFidId = camera1TFID
            elif camera2TFID > -1:
                tempFidId = camera2TFID
            else:
                setpointActiveRight = False
        else:
            setpointActiveRight = False
        if (not cam1.hasTargets and not cam2.hasTargets) and (
            setPointLeft or setPointRight
        ):

            self.updateLimelight(hal, setPointLeft, setPointRight)

        elif setpointActiveLeft:

            self.setpointChooser(
                self.odometry.getPose().rotation().radians(),
                tempFidId,
                "left",
            )
            self.updateWithoutSticks(hal, self.adjustedSpeeds)
        elif setpointActiveRight:

            self.setpointChooser(
                self.odometry.getPose().rotation().radians(),
                tempFidId,
                "right",
            )
            self.updateWithoutSticks(hal, self.adjustedSpeeds)
        else:
            self.updateWithSticks(
                hal, joystickX, joystickY, joystickRotation, RTriggerScalar, resetOffset
            )

        if savePos:
            if tempFidId > 0:
                self.savePos(
                    tempFidId,
                    self.odometry.getPose().rotation().radians(),
                )
            else:
                self.savePos(0, self.odometry.getPose().rotation().radians())

    def updateWithSticks(
        self,
        hal: RobotHALBuffer,
        joystickX: float,
        joystickY: float,
        joystickRotation: float,
        RTriggerScalar: float,
        resetOffset: bool,
    ) -> None:
        if self.debugMode:
            # self.table.putNumber("Drive Ctrl X", joystickX)
            # self.table.putNumber("Drive Ctrl Y", joystickY)
            # self.table.putNumber("Drive Ctrl Rotation", joystickRotation)
            pass

        self.swerveSticksMath.start()

        if math.sqrt(joystickX**2 + joystickY**2) < 0.08:
            joystickX = 0
            joystickY = 0
        if abs(joystickRotation) < 0.06:
            joystickRotation = 0

        offsetX = 0.05 * np.sign(joystickX)
        offsetY = 0.05 * np.sign(joystickY)
        offsetR = 0.05 * np.sign(joystickRotation)

        proxyDeadZoneX = (joystickX - offsetX) * 3.5
        proxyDeadZoneY = (joystickY - offsetY) * 3.5
        proxyDeadZoneR = (joystickRotation - offsetR) * 10
        # # self.table.putNumber(
        #     "setting up dead zones stuff update Time",
        #     wpilib.getTime() - startCameraUpdate,
        # )
        # the controller's x axis the the ChassisSpeeds' y axis and same for the other x and y axies
        # the signes are flipped for the differences too

        driveY = -proxyDeadZoneX
        driveX = -proxyDeadZoneY
        driveRotation = -proxyDeadZoneR

        driveVector = Translation2d(driveX, driveY)

        if resetOffset:
            self.yawOffset = hal.yaw

        # self.table.putNumber("absDriveOffset", self.yawOffset)

        # abs drive toggle
        if hal.fieldOriented:
            driveVector = driveVector.rotateBy(Rotation2d(-hal.yaw + self.yawOffset))

        # disable rotatioanl PID if turn stick is moved
        if driveRotation != 0:
            hal.rotPIDToggle = False

        # self.table.putNumber("z_PID Setpoint", hal.rotPIDsetpoint)
        # self.table.putBoolean("z_Absolute Drive", hal.fieldOriented)
        # # self.table.putNumber(
        #     "unsure what this does update Time", wpilib.getTime() - startCameraUpdate
        # )
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
            rotFinal = driveRotation  # copied from HCPA code
        # # self.table.putNumber(
        #     " Emmetts scary stuff update Time", wpilib.getTime() - startCameraUpdate
        # )
        # -------------------------------------------------------------------

        chassisSpeeds = ChassisSpeeds(
            driveVector.X() * 0.5 * 4**RTriggerScalar,
            driveVector.Y() * 0.5 * 4**RTriggerScalar,
            rotFinal,
        )

        self.swerveSticksMath.end()

        if self.debugMode:
            # self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
            # self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
            # self.table.putNumber("SD ChassisSpeeds omega", self.chassisSpeeds.omega)
            # self.table.putNumber(
            #     "SD RotPIDSpeed omega (adjustedSpeedsOmega)",
            #     adjustedSpeeds.omega,  # * (180 / math.pi)
            # )
            # self.table.putBoolean("rotPIDToggle", hal.rotPIDToggle)
            # self.table.putNumber("z_target rotDeg", rotTarget.degrees())
            # self.table.putNumber("z_current rotDeg", fakeBotPos.rotation().degrees())
            pass

        self.updateWithoutSticks(hal, chassisSpeeds)

    def updateOdometry(self, hal: RobotHALBuffer):

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

        self.odomPos = [self.odometry.getPose().X(), self.odometry.getPose().Y()]
        self.OdomField.setRobotPose(self.odometry.getPose())

        # self.table.putNumber("odomX", self.odomPos[0])
        # self.table.putNumber("odomy", self.odomPos[1])
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

    def updateWithoutSticks(self, hal: RobotHALBuffer, chassisSpeeds: ChassisSpeeds):

        # self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
        # self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
        # self.table.putNumber("SD ChassisSpeeds omega", self.chassisSpeeds.omega)

        self.desaturateSpeedsProfiler.start()

        self.unleashedModules = self.kinematics.toSwerveModuleStates(chassisSpeeds)
        swerveModuleStates = self.kinematics.desaturateWheelSpeeds(
            self.unleashedModules,
            self.MAX_METERS_PER_SEC,
        )

        self.desaturateSpeedsProfiler.end()

        # self.table.putNumber(
        #     "SD Original Turn Setpoint", swerveModuleStates[0].angle.radians()
        # )

        # self.table.putNumber("SD Original Drive Setpoint", swerveModuleStates[0].speed)

        self.optimizeTargetProfiler.start()
        FLModuleState = self.optimizeTarget(
            swerveModuleStates[0], Rotation2d(hal.turnCCWFL)
        )
        hal.driveFLSetpoint = FLModuleState.speed
        hal.turnFLSetpoint = FLModuleState.angle.radians()
        # self.table.putNumber("SD Opimized Turn Setpoint", FLModuleState.angle.radians())

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
        self.optimizeTargetProfiler.end()

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

    def updateForAutos(self, hal: RobotHALBuffer, chassisSpeeds: ChassisSpeeds):

        # temp = chassisSpeed.vx
        # chassisSpeed.vx = chassisSpeed.vy
        # chassisSpeed.vy = temp

        # self.table.putNumber("SD ChassisSpeeds vx", self.chassisSpeeds.vx)
        # self.table.putNumber("SD ChassisSpeeds vy", self.chassisSpeeds.vy)
        # self.table.putNumber("SD ChassisSpeeds omega", self.chassisSpeeds.omega)

        self.unleashedModules = self.kinematics.toSwerveModuleStates(chassisSpeeds)
        swerveModuleStates = self.kinematics.desaturateWheelSpeeds(
            self.unleashedModules,
            self.MAX_METERS_PER_SEC,
        )

        # self.table.putNumber(
        # "SD Original Turn Setpoint", swerveModuleStates[0].angle.radians()
        # )

        # self.table.putNumber("SD Original Drive Setpoint", swerveModuleStates[0].speed)

        FLModuleState = self.optimizeTarget(
            swerveModuleStates[0], Rotation2d(hal.turnCCWFL)
        )
        hal.driveFLSetpoint = FLModuleState.speed
        hal.turnFLSetpoint = FLModuleState.angle.radians()
        # self.table.putNumber("SD Opimized Turn Setpoint", FLModuleState.angle.radians())

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

    def updateLimelight(self, hal: RobotHALBuffer, right, left):

        validTarget = self.llTable.getNumber("tv", 0) == 1
        if not validTarget:
            self.updateWithoutSticks(hal, ChassisSpeeds(0, 0, 0))
            return

        txValues = self.llTable.getNumberArray("llpython", [])
        if len(txValues) < 2:
            tx = txValues[0]
        else:
            if left:
                tx = min(txValues[0], txValues[1])
            elif right:
                tx = max(txValues[0], txValues[1])

        if abs(self.DESIRED_TX - tx) < self.ALLOWED_ERROR_LL:
            self.updateWithoutSticks(hal, ChassisSpeeds(0, 0, 0))
            return

        chassisY = self.reefPIDController.calculate(tx, self.DESIRED_TX)
        chassisSpeeds = ChassisSpeeds(0, chassisY, 0)
        self.updateWithoutSticks(hal, chassisSpeeds)
        self.table.putNumber("Strafe Speed", chassisY)
