import copy
import math

import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
from timing import TimeData

#demoknighttf2eylanderstangeifier
#ask about cassiopeia sensors

class RobotHALBuffer:
    def __init__(self) -> None:
        pass

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)
