import math
from copy import copy
from typing import TYPE_CHECKING, Callable

from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerTrajectory  # type: ignore


from real import angleWrap
from wpimath.geometry import Pose2d, Translation2d

if TYPE_CHECKING:
    from robot import Robot

StageFunc = Callable[["Robot"], bool | None]


class Stage:
    def __init__(self, f: StageFunc, name: str) -> None:
        self.func = f
        self.name = name
        self.nextStage: "Stage | None" = None
        self.abortStage: "Stage | None" = None

    def add(self, s: "Stage") -> tuple["Stage", "Stage"]:
        self.nextStage = s
        return self, s


class AutoBuilder:
    def __init__(self) -> None:
        self.firstStage: Stage | None = None
        self.currentBuildStage: Stage = None  # type: ignore
        self.currentRunStage: Stage | None = None

        self.stageStart = 0  # This is the trigger for restarting a seq. when 0, run will start from firstRunStage
        self.table = NetworkTableInstance.getDefault().getTable("autos")

    # Resets and makes sure StageBuilder will start up from the beginning of the sequence next run call
    def reset(self, time: float) -> None:
        self.stageStart = 0

    # runs one tick, returns if still running or finished
    def run(self, r: "Robot") -> bool:
        if self.stageStart == 0:
            self.stageStart = r.time.timeSinceInit
            self.currentRunStage = self.firstStage

        self.table.putNumber("stageTime", self.stageStart)
        if self.currentRunStage is not None:
            self.table.putString("stage", f"{self.currentRunStage.name}")
            done = self.currentRunStage.func(r)
            if done is True:
                self.currentRunStage = self.currentRunStage.nextStage
                self.stageStart = r.time.timeSinceInit
            elif done is None:
                self.currentRunStage = self.currentRunStage.abortStage
                self.stageStart = r.time.timeSinceInit

        return self.currentRunStage is None

    def add(self, new: Stage | None) -> "AutoBuilder":
        if new is None:
            return self

        if self.currentBuildStage is not None:
            self.currentBuildStage.nextStage = new
        self.currentBuildStage = new

        if self.firstStage is None:
            self.firstStage = new
        return self

    def addAbort(self, new: "AutoBuilder") -> "AutoBuilder":
        if self.currentBuildStage is not None:
            self.currentBuildStage.abortStage = new.firstStage
        return self

    # NOTE: TODO: doesn't make a copy of the new stages - so watch out
    def addSequence(self, s: "AutoBuilder") -> "AutoBuilder":
        self.add(s.firstStage)
        while self.currentBuildStage.nextStage is not None:
            self.currentBuildStage = self.currentBuildStage.nextStage
        return self

    def _newPathStage(self, t: PathPlannerTrajectory, trajName: str) -> Stage:
        def func(r: "Robot") -> bool | None:
            goal = t.sample(r.time.timeSinceInit - r.auto.stageStart)

            table = NetworkTableInstance.getDefault().getTable("autos")
            # table.putNumber("pathGoalX", goal.getTargetHolonomicPose().X())
            # table.putNumber("pathGoalY", goal.getTargetHolonomicPose().Y())

            table.putNumber("pathGoalX", goal.pose.X())
            table.putNumber("pathGoalY", goal.pose.Y())
            table.putNumber("pathGoalR", goal.pose.rotation().radians())

            table.putNumber(
                "odomR", r.swerveDrive.odometry.getPose().rotation().radians()
            )
            adjustedSpeeds = r.holonomicDriveController.calculateRobotRelativeSpeeds(
                r.swerveDrive.odometry.getPose(), goal
            )
            table.putNumber("pathVelX", adjustedSpeeds.vx)
            table.putNumber("pathVelY", adjustedSpeeds.vy)
            table.putNumber("pathVelR", adjustedSpeeds.omega)

            r.swerveDrive.updateWithoutSticks(r.hal, adjustedSpeeds)
            return (r.time.timeSinceInit - r.auto.stageStart) > t.getTotalTimeSeconds()

        s = Stage(func, f"path '{trajName}'")
        return s

    # NOTE: filename is *just* the title of the file, with no extension and no path
    # filename is directly passed to pathplanner.loadPath
    def addPathStage(
        self, t: PathPlannerTrajectory, trajName: str = "unnamed"
    ) -> "AutoBuilder":
        self.add(self._newPathStage(t, trajName))
        return self

    def addTelemetryStage(self, s: str) -> "AutoBuilder":
        def func(r: "Robot") -> bool | None:
            NetworkTableInstance.getDefault().getTable("autos").putString(
                "telemStageLog", s
            )
            return True

        self.add(Stage(func, f"logging {s}"))
        return self
