from __future__ import annotations
import math


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def invLerp(a, b, pt):
    return (pt - a) / (b - a)


# CLEANUP: this
# returns input angle between (-pi, pi]
# Converts any radian to a CCW Radian
def angleWrap(a: float) -> float:
    while a > math.pi:
        a -= math.pi * 2
    while a <= -math.pi:
        a += math.pi * 2
    return a


def signum(x: float) -> float:
    return float((x > 0) - (x < 0))
