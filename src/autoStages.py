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

if TYPE_CHECKING:
    from robot import Robot

StageFunc = Callable[["Robot"], bool | None]
# Auto tools


class RobotAutos(Enum):
    NO_AUTO = "NO Auto"
    MOVE_FORWARD_A = "Move Forward A"
    DO_NOTHING = "DO Nothing"
    HIGH4_CENTER = "place highL4"
    TEST = "test"
    REEF4_l4 = "reef:4 L_4"


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


class ASfollowPath(AutoStage):

    def __init__(self, trajName: str, flipped: bool, r: "Robot"):
        self.r = r
        self.done = False
        self.traj: PathPlannerTrajectory = loadTrajectory(trajName, flipped)
        self.startTime = wpilib.getTime()
        self.r.swerveDrive.odometry.resetPose(self.traj.getInitialPose())
        self.r.hardware.resetGyroToAngle(
            self.traj.getInitialPose().rotation().radians()
        )

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
        adjustedSpeeds.omega = adjustedSpeeds.omega
        table.putNumber("pathVelX", adjustedSpeeds.vx)
        table.putNumber("pathVelY", adjustedSpeeds.vy)
        table.putNumber("pathVelR", adjustedSpeeds.omega)

        table.putNumber("gyro", self.r.hal.yaw)

        self.r.swerveDrive.updateForAutos(self.r.hal, adjustedSpeeds)

    def isDone(self, r: "Robot"):
        x = self.r.swerveDrive.odometry.getPose().X()
        y = self.r.swerveDrive.odometry.getPose().Y()
        rot = self.r.swerveDrive.odometry.getPose().rotation()

        end = self.traj.getEndState().pose

        error = 0.2
        rotError = math.pi / 20

        if (
            (end.X() - error > x)
            and (x > end.X() + error)
            and (end.Y() - error > y)
            and (y > end.Y() + error)
            and (end.rotation().radians() - rotError > rot)
            and (rot > end.rotation().radians() + rotError)
        ):
            self.done = True


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


class ASShootStored(AutoStage):
    def __init__(self, r: "Robot"):
        self.done = False
        r.manipulatorSubsystem.state = 2
        self.buf = r.hal

    def run(self, r: "Robot"):
        r.manipulatorSubsystem.autoShootStored(r.hal)

    def isDone(self, r: "Robot"):
        if r.manipulatorSubsystem.state == r.manipulatorSubsystem.ManipulatorState.IDLE:
            self.done = True


# returns a dict with strings as key and AutoStage as value
def chooseAuto(stageChooser: str, r: "Robot") -> dict[str, AutoStage]:
    ret: dict[str, AutoStage] = dict()

    if stageChooser == RobotAutos.NO_AUTO.value:
        pass
    elif stageChooser == RobotAutos.DO_NOTHING.value:
        pass
    elif stageChooser == RobotAutos.MOVE_FORWARD_A.value:
        ret["leftCorner-leftDiag"] = ASfollowPath("leftCorner-leftDiag", r.onRedSide, r)
    elif stageChooser == RobotAutos.TEST.value:
        ret["test"] = ASfollowPath("test", r.onRedSide, r)
    elif stageChooser == RobotAutos.REEF4_l4.value:
        ret["leftCorner-leftDiag"] = ASfollowPath("leftCorner-leftDiag", r.onRedSide, r)
        ret["leftDiag-reef4"] = ASfollowPath("leftDiag-reef4", r.onRedSide, r)
        ret["elevator-level4"] = ASelevator4()
        ret["shoot-piece"] = ASShootStored(r)
    # elif stageChooser == RobotAutos.HIGH4_CENTER:
    #     ret["elevator up"] = ASelevatorHigh(r)
    #     ret

    return ret
