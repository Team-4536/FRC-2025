import math
from wpimath.geometry import Rotation2d, Translation2d
import wpilib
import numpy

from wpimath.geometry import Rotation2d, Translation2d


class Scalar:
    def __init__(self, deadZone=0.1, exponent=1):
        self.deadzone = deadZone
        self.exponent = exponent

    def scale(self, input):
        if abs(input) <= self._deadZone:
            return 0
        else:
            delta = abs(input) - self._deadZone
            sign = math.copysign(1, input)
            return sign * (delta / self._scale) ** self._exponent

    def __call__(self, input):
        return self.scale(input)

    @property
    def deadzone(self):
        return self._deadZone

    def setExponent(self, exponent):
        self.exponent = exponent

    @deadzone.setter
    def deadzone(self, deadZone):
        deadZone = abs(deadZone)
        self._deadZone = deadZone
        self._scale = 1 - deadZone

    @property
    def exponent(self):
        return self._exponent

    @exponent.setter
    def exponent(self, exponent):
        self._exponent = exponent


class CircularScalar:
    def __init__(self, deadzone: float, exponent: int):
        self.scalar = Scalar(deadzone, exponent)

    def Scale(self, x: float, y: float):
        angle = math.atan2(y, x)
        mag = math.hypot(x, y)

        ScaledMag = self.scalar.scale(mag)

        stickXY = Translation2d(ScaledMag, 0)
        stickXY = stickXY.rotateBy(Rotation2d(angle))

        return stickXY.x, stickXY.y
