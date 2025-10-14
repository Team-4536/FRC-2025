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

