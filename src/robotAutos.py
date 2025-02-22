import robot
import wpilib
import swerveDrive
from swerveDrive import SwerveDrive
from autos import AutoBuilder
from wpimath.geometry import Translation2d
from rev import SparkMax
from pathplannerlib.path import PathPlannerPath, PathPlannerTrajectory
from pathplannerlib.config import RobotConfig, ModuleConfig

from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

AUTO_NONE = "do nothng"
DRIVE_FORWARD = "drive off the starting line"


class RobotAutos:
    def __init__(self) -> None:
        self.autoChooser = wpilib.SendableChooser()
        self.autoChooser.setDefaultOption(AUTO_NONE, AUTO_NONE)
        self.autoChooser.addOption(DRIVE_FORWARD, DRIVE_FORWARD)

    def loadTrajectory(self, fileName: str, flipped: bool) -> PathPlannerTrajectory:
        mass = 122 / 9.8
        moi = 1 / 2 * ((mass) * (mass))
        oneftInMeters = 0.3048
        motor = SparkMax()
        modConfig = ModuleConfig(
            0.5,
            1.1,
            9.5,
            motor,
        )
        RConfig = RobotConfig(
            mass,
            moi,
            modConfig[
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
            ChassisSpeeds(), p.getStartingHolonomicPose().rotation(), RConfig
        )
        return t

    def autoInit(self, r: "robot.Robot") -> tuple[AutoBuilder, Pose2d]:

        auto = AutoBuilder()
        initialPose: Pose2d = Pose2d()
        traj = self.loadTrajectory("blue_3_offline", r.onRedSide)

        if self.autoChooser.getSelected() == AUTO_NONE:
            pass

        elif self.autoChooser.getSelected == DRIVE_FORWARD:
            initialPose = traj.getInitialState().pose
            auto.addTelemetryStage(DRIVE_FORWARD)
            auto.addPathStage(traj)
