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
    ret = math.fmod(a + math.pi, math.pi * 2)
    if ret < 0:
        ret += math.pi * 2

    return ret - math.pi


def signum(x: float) -> float:
    return float((x > 0) - (x < 0))
