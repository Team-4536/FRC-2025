import math
from photonOdometry import photonVision
import rev
import robotHAL
import wpilib
import swerveDrive
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from timing import TimeData
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from elevator import ElevatorSubsystem
from robotHAL import RobotHAL, RobotHALBuffer
from swerveDrive import SwerveDrive
from wpimath.units import radians
from manipulator import ManipulatorSubsystem
from IntakeChute import IntakeChute
from led import LEDSignals
import pathplannerlib  # type: ignore
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants  # type: ignore
import autoStages
from profiler import Profiler

from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from robot import Robot

StageFunc = Callable[["Robot"], bool | None]


class TeleopAutos:

    def TeleopAutonomousInit(
        self,
        r: "Robot",
    ) -> None:
        r.hal.stopMotors()
        AUTO_SIDE_RED = "red"
        AUTO_SIDE_BLUE = "blue"
        r.onRedSide = r.autoSideChooser.getSelected() == AUTO_SIDE_RED
        r.autoStartTime = wpilib.getTime()
        r.holonomicDriveController = PPHolonomicDriveController(
            PIDConstants(7, 0, 0, 0), PIDConstants(1.5, 0, 0, 0)
        )

        r.auto = autoStages.chooseAuto(r.autoRoutineChooser.getSelected(), r)

        r.autoKeys = list(r.auto.keys())
        r.currentAuto = 0
        r.autoFinished = False

        if not r.currentAuto == len(r.autoKeys):  ## TDOO Fix

            r.auto[r.autoKeys[r.currentAuto]].autoInit(r)
        # Keep the lines below at the bottom of teleopPeriodic
        r.hal.publish()

        r.hardwareProfiler.start()
        r.hardware.update(r.hal, r.time)
        r.hardwareProfiler.end()

    def update(self, r: "Robot") -> None:
        r.hal.stopMotors()  # Keep this at the top of autonomousPeriodic

        # r.swerveDrive.resetOdometry(r, Pose2d(0, 0, Rotation2d(radians(0))), r.hal, -1)
        # r.swerveDrive.resetOdometry(Pose2d(), r.hal,-1)

        if r.currentAuto >= len(r.autoKeys):
            r.autoFinished = True
        else:
            r.table.putString("Current Stage", r.autoKeys[r.currentAuto])
            r.table.putNumber("Current Stage Number", r.currentAuto)

        r.table.putBoolean("Auto finished", r.autoFinished)

        if not r.autoFinished:
            r.auto[r.autoKeys[r.currentAuto]].run(r)
            if r.auto[r.autoKeys[r.currentAuto]].isDone(r):
                r.currentAuto += 1
                if not r.currentAuto >= len(r.autoKeys):
                    r.auto[r.autoKeys[r.currentAuto]].autoInit(r)

        r.intakeChuteProfiler.start()
        r.intakeChute.update(
            r.hal,
            False,
            False,
            False,
            False,
        )
        r.intakeChuteProfiler.end()

        # if (wpilib.getTime() - r.autoStartTime) < 5:
        #     r.swerveDrive.updateWithoutSticks(r.hal, ChassisSpeeds(-0.25, 0, 0))
        # else:
        #     r.swerveDrive.updateWithoutSticks(r.hal, ChassisSpeeds(0, 0, 0))

        r.hardwareProfiler.start()
        r.hardware.update(
            r.hal, r.time
        )  # Keep this at the bottom of autonomousPeriodic
        r.hardwareProfiler.end()
