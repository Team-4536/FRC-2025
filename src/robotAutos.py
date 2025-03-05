import robot
import wpilib
import swerveDrive
from swerveDrive import SwerveDrive
from autos import AutoBuilder
from wpimath.geometry import Translation2d
import rev
from rev import SparkMax
from pathplannerlib.path import PathPlannerPath, PathPlannerTrajectory  # type: ignore
from pathplannerlib.config import RobotConfig, ModuleConfig, DCMotor  # type: ignore
from robotHAL import RobotHAL as r
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath import units


AUTO_NONE = "do nothng"
FORWARD_A = "Forward A"
REEF4_A = "reef4 A"


class RobotAutos:
    def __init__(self) -> None:
        self.autoChooser = wpilib.SendableChooser()
        self.autoChooser.setDefaultOption(AUTO_NONE, AUTO_NONE)
        self.autoChooser.addOption(FORWARD_A, FORWARD_A)
        self.autoChooser.addOption(REEF4_A, REEF4_A)
        wpilib.SmartDashboard.putData("auto path chooser", self.autoChooser)

    def loadTrajectory(self, fileName: str, flipped: bool) -> PathPlannerTrajectory:
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

    def autoInit(self, r: "robot.Robot") -> tuple[AutoBuilder, Pose2d]:

        auto = AutoBuilder()
        initialPose: Pose2d = Pose2d()
        traj = self.loadTrajectory("leftCorner-leftDiag", r.onRedSide)

        if self.autoChooser.getSelected() == AUTO_NONE:
            pass

        elif self.autoChooser.getSelected() == FORWARD_A:
            initialPose = traj.getInitialState().pose
            auto.addTelemetryStage(FORWARD_A)
            auto.addPathStage(traj)

        elif self.autoChooser.getSelected() == REEF4_A:
            initialPose = traj.getInitialState().pose
            auto.addTelemetryStage(FORWARD_A)
            auto.addPathStage(traj)
            traj = self.loadTrajectory("leftDiag-reef4", r.onRedSide)
            auto.addPathStage(traj)

        else:

            assert False

        return auto, initialPose
