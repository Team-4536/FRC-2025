import robot
import wpilib
import math
import swerveDrive
from swerveDrive import SwerveDrive
from wpimath.geometry import Translation2d
import rev
from rev import SparkMax
from pathplannerlib.path import PathPlannerPath, PathPlannerTrajectory  # type: ignore
from pathplannerlib.config import RobotConfig, ModuleConfig, DCMotor  # type: ignore
from robotHAL import RobotHAL as r
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath import units
from wpimath.units import seconds
from ntcore import NetworkTableInstance, NetworkTable
from typing import TYPE_CHECKING, Callable
from ntcore import Value
from enum import Enum
from manipulator import ManipulatorSubsystem
from wpimath.units import meters, radians
from manipulator import ManipulatorState

if TYPE_CHECKING:
    from robot import Robot

StageFunc = Callable[["Robot"], bool | None]
# Auto tools


class RobotAutos(Enum):
    NO_AUTO = "NO Auto"
    # MOVE_FORWARD_A = "Move Forward A"
    DO_NOTHING = "DO Nothing"
    HIGH4_CENTER = "place highL4"
    TEST = "test"
    REEF4_l4 = "reef:4 L_4"
    REEF1_2PIECE = "reef:1 2piece"
    L4_TEST = "l4 test"
    SHOOT_TEST = "shoot test"
    AUTO3_TEST = "3 auto test"
    INTAKE_TEST = "intake test"
    TEST_ROUTINE = "test routine"
    DRIVE_FORWARD = "drive forward"
    MIDDLE_PIECE = "middle 1 piece"
    PIECE2_AUTO = "2 piece auto"
    ROTATION_TEST = "rotation test"
    FROMLEFT_PLACE = "place from left"


def loadTrajectory(fileName: str, flipped: bool) -> PathPlannerTrajectory:
    oneftInMeters = units.feetToMeters(1)
    mass = units.lbsToKilograms(122)
    moi = (
        (1 / 12)
        * mass
        * (oneftInMeters * oneftInMeters + oneftInMeters * oneftInMeters)
    )
    # motor = SparkMax(1, rev.SparkMax.MotorType.kBrushless)
    motor = DCMotor(12, 2.6, 105, 1.8, 5676, 1)
    modConfig = ModuleConfig(0.05, 1.1, 9.5, motor, 42, 1)
    RConfig = RobotConfig(
        mass,
        moi,
        modConfig,
        [
            Translation2d(-oneftInMeters, oneftInMeters),
            Translation2d(oneftInMeters, oneftInMeters),
            Translation2d(-oneftInMeters, -oneftInMeters),
            Translation2d(oneftInMeters, -oneftInMeters),
        ],
    )
    p = PathPlannerPath.fromPathFile(fileName)
    if flipped:
        p = p.flipPath()

    t = p.generateTrajectory(
        ChassisSpeeds(),
        p.getStartingHolonomicPose().rotation(),
        RConfig,
    )
    return t


class AutoStage:
    def __init__(self):
        pass

    def run(self, r: "Robot"):
        pass

    def isDone(self, r: "Robot") -> bool:
        return True

    def autoInit(self, r: "Robot"):
        pass


class ASfollowPath(AutoStage):

    def __init__(self, trajName: str, flipped: bool, r: "Robot"):
        self.r = r
        self.done = False
        self.traj: PathPlannerTrajectory = loadTrajectory(trajName, flipped)

        # self.traj = PathPlannerTrajectory()

    def run(self, r: "Robot"):
        self.currentTime = wpilib.getTime()
        self.time = self.currentTime - self.startTime
        goal = self.traj.sample(self.time)

        table = NetworkTableInstance.getDefault().getTable("autos")
        # table.putNumber("pathGoalX", goal.getTargetHolonomicPose().X())
        # table.putNumber("pathGoalY", goal.getTargetHolonomicPose().Y())

        table.putNumber("pathGoalX", goal.pose.X())
        table.putNumber("pathGoalY", goal.pose.Y())
        table.putNumber("pathGoalR", goal.pose.rotation().radians())
        table.putNumber(
            "odomR", self.r.swerveDrive.odometry.getPose().rotation().radians()
        )
        adjustedSpeeds = self.r.holonomicDriveController.calculateRobotRelativeSpeeds(
            self.r.swerveDrive.odometry.getPose(), goal
        )

        b = self.r.holonomicDriveController.getPositionalError()

        table.putNumber("pathError", b)

        adjustedSpeeds.omega = adjustedSpeeds.omega
        table.putNumber("pathVelX", adjustedSpeeds.vx)
        table.putNumber("pathVelY", adjustedSpeeds.vy)
        table.putNumber("pathVelR", adjustedSpeeds.omega)

        table.putNumber("gyro", self.r.hal.yaw)

        self.r.swerveDrive.updateForAutos(self.r.hal, adjustedSpeeds)

    def autoInit(self, r):
        self.startTime = wpilib.getTime()
        self.r.swerveDrive.odometry.resetPose(self.traj.getInitialPose())
        # self.r.hardware.resetGyroToAngle(
        #     self.traj.getInitialPose().rotation().radians()
        # )

        table = NetworkTableInstance.getDefault().getTable("autos")
        table.putNumber("djoInitPoseX", self.traj.getInitialPose().x)
        table.putNumber("djoInitPoseY", self.traj.getInitialPose().y)
        table.putNumber("djoInitPoseR", self.traj.getInitialPose().rotation().radians())

    def isDone(self, r: "Robot"):
        x = self.r.swerveDrive.odometry.getPose().X()
        y = self.r.swerveDrive.odometry.getPose().Y()
        rot = self.r.swerveDrive.odometry.getPose().rotation().radians()

        end = self.traj.getEndState().pose

        error = meters(0.15)
        rotError = radians(0.2)

        table = NetworkTableInstance.getDefault().getTable("autos")
        table.putNumber("endstateX", self.traj.getEndState().pose.X())
        table.putNumber("endstateY", self.traj.getEndState().pose.Y())
        table.putNumber(
            "endstateRot", self.traj.getEndState().pose.rotation().radians()
        )
        table.putNumber("djoDonePoseX", r.swerveDrive.odometry.getPose().x)
        table.putNumber("djoDonePoseY", r.swerveDrive.odometry.getPose().y)
        table.putNumber(
            "djoDonePoseR", r.swerveDrive.odometry.getPose().rotation().radians()
        )

        if (
            (end.X() - error < x)
            and (x < end.X() + error)
            and (end.Y() - error < y)
            and (y < end.Y() + error)
            and (end.rotation().radians() - rotError < rot)
            and (rot < end.rotation().radians() + rotError)
        ):
            self.done = True
            return True


class ASelevator4(AutoStage):
    def __init__(self):
        self.done = False

    def run(self, r: "Robot"):

        r.elevatorSubsystem.level4AutoUpdate(r.hal)

    def isDone(self, r: "Robot"):

        if (r.hal.elevatorPos > r.hal.elevatorSetpoint - 1) and (
            r.hal.elevatorPos < r.hal.elevatorSetpoint + 1
        ):
            self.done = True

            return True


class ASelvator0(AutoStage):
    def __init__(self):
        self.done = False

    def run(self, r: "Robot"):
        r.elevatorSubsystem.level0AutoUpdate(r.hal)

    def isDone(self, r: "Robot"):
        if (r.hal.elevatorPos > r.hal.elevatorSetpoint - 1) and (
            r.hal.elevatorPos < r.hal.elevatorSetpoint + 1
        ):
            self.done = True
            return True


class ASShootStored(AutoStage):
    def __init__(self, r: "Robot", startTime):
        self.done = False
        # r.manipulatorSubsystem.state = ManipulatorSubsystem.ManipulatorState.STORED
        self.buf = r.hal

    def run(self, r: "Robot"):
        r.manipulatorSubsystem.update(r.hal, True, False)

    def isDone(self, r: "Robot"):
        if r.manipulatorSubsystem.state == ManipulatorState.IDLE:
            self.done = True
            return True

    def autoInit(self, r):
        self.startTime = wpilib.getTime
        r.manipulatorSubsystem.state = ManipulatorState.STORED


class ASintakeCoraL(AutoStage):
    def __init__(self):
        self.done = False

    def run(self, r: "Robot"):
        r.manipulatorSubsystem.update(r.hal, False, False)
        pass

    def isDone(self, r):
        if r.manipulatorSubsystem.state == ManipulatorState.STORED:
            self.done = True

        return self.done


# returns a dict with strings as key and AutoStage as value
def chooseAuto(stageChooser: str, r: "Robot") -> dict[str, AutoStage]:
    ret: dict[str, AutoStage] = dict()

    if stageChooser == RobotAutos.NO_AUTO.value:
        pass
    elif stageChooser == RobotAutos.DO_NOTHING.value:
        pass
    # elif stageChooser == RobotAutos.MOVE_FORWARD_A.value:
    #     ret["leftCorner-leftDiag"] = ASfollowPath("leftCorner-leftDiag", r.onRedSide, r)
    elif stageChooser == RobotAutos.TEST.value:
        ret["test"] = ASfollowPath("test", r.onRedSide, r)
    elif stageChooser == RobotAutos.REEF4_l4.value:
        ret["leftCorner-leftDiag"] = ASfollowPath("leftCorner-leftDiag", r.onRedSide, r)
        ret["leftDiag-reef4"] = ASfollowPath("leftDiag-reef4", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-piece"] = ASShootStored(r, wpilib.getTime())
    elif stageChooser == RobotAutos.REEF1_2PIECE.value:
        ret["middle-reef1"] = ASfollowPath("middle-reef1", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-piece"] = ASShootStored(r, wpilib.getTime())
        ret["elevator-level0"] = ASelvator0()
        ret["reef1-intake"] = ASfollowPath("reef1-intake", r.onRedSide, r)
        ret["intake-coral"] = ASintakeCoraL()
        ret["intake-reef6"] = ASfollowPath("intake-reef6", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-piece"] = ASShootStored(r, wpilib.getTime())
    elif stageChooser == RobotAutos.SHOOT_TEST.value:
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-stored"] = ASShootStored(r, wpilib.getTime())
    elif stageChooser == RobotAutos.L4_TEST.value:
        ret["elevator-level4"] = ASelevator4()
    elif stageChooser == RobotAutos.AUTO3_TEST.value:
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-stored"] = ASShootStored(r, wpilib.getTime())
        ret["elevator-level0"] = ASelvator0()
    elif stageChooser == RobotAutos.INTAKE_TEST.value:
        ret["intake-coral"] = ASintakeCoraL()
    elif stageChooser == RobotAutos.TEST_ROUTINE.value:
        ret["intake-coral"] = ASintakeCoraL()
        ret["middle-reef1"] = ASfollowPath("test", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-piece"] = ASShootStored(r, wpilib.getTime())
        ret["elevator-level0"] = ASelvator0()
    elif stageChooser == RobotAutos.DRIVE_FORWARD.value:
        ret["leftCorner-leftDiag"] = ASfollowPath("leftCorner-leftDiag", r.onRedSide, r)
    elif stageChooser == RobotAutos.MIDDLE_PIECE.value:
        ret["intake-coral"] = ASintakeCoraL()
        ret["middle-reef1"] = ASfollowPath("middle-reef1", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-stored"] = ASShootStored(r, wpilib.getTime())
        ret["elevator-level0"] = ASelvator0()
    elif stageChooser == RobotAutos.PIECE2_AUTO.value:
        ret["algae1"] = ASfollowPath("Algae1", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-stored"] = ASShootStored(r, wpilib.getTime())
        ret["elevator-level0"] = ASelvator0()
        ret["Algae1Dropoff"] = ASfollowPath("Algae1Dropoff", r.onRedSide, r)
        ret["intake"] = ASintakeCoraL()
        ret["Algae2"] = ASfollowPath("Algae2", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-stored"] = ASShootStored(r, wpilib.getTime())
        ret["elevator-level0"] = ASelvator0()
    elif stageChooser == RobotAutos.ROTATION_TEST.value:
        ret["rotation-test"] = ASfollowPath("rotation-test", r.onRedSide, r)
    elif stageChooser == RobotAutos.FROMLEFT_PLACE.value:
        ret["intake-coral"] = ASintakeCoraL()
        ret["leftCorner-reef6"] = ASfollowPath("leftCorner-reef6", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-stored"] = ASShootStored(r, wpilib.getTime())
        ret["elevator-level0"] = ASelvator0()

    # elif stageChooser == RobotAutos.HIGH4_CENTER:
    #     ret["elevator up"] = ASelevatorHigh(r)
    #     ret

    return ret
