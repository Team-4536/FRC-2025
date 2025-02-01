from __future__ import annotations
import math
import numpy as np


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
    while a < -math.pi:
        a += math.pi * 2
    return a


def signum(x: float) -> float:
    return float((x > 0) - (x < 0))

def arcsinList(self, desY, unboundrotations)->list:
        
        unboundAngle = (unboundrotations * (2*math.pi))/25

        referenceValue = np.arcsin(desY)
        wrongList: list[float] = referenceValue

        if referenceValue >= 0:
            
            referenceValue2 = ((((math.pi)/25)/2)+(((math.pi/25)/2) - referenceValue))
            wrongList.append(referenceValue2)
            wrongList.append(referenceValue+((2*math.pi)/25))
            wrongList.append(referenceValue2+((2*math.pi)/25))

            wrongList.append(referenceValue - ((2*math.pi)/25))
            wrongList.append(referenceValue2 - ((2*math.pi)/25))
            wrongList.append(referenceValue - ((4*math.pi)/25))
            wrongList.append(referenceValue2 - ((4*math.pi)/25))

        elif referenceValue < 0:

            referenceValue2 = (-((math.pi/25)/2)-(((math.pi/25)/2) - referenceValue))
            wrongList.append(referenceValue2)
            wrongList.append(referenceValue-((2*math.pi)/25))
            wrongList.append(referenceValue2-((2*math.pi)/25))

            wrongList.append(referenceValue + ((2*math.pi)/25))
            wrongList.append(referenceValue2 + ((2*math.pi)/25))
            wrongList.append(referenceValue + ((4*math.pi)/25))
            wrongList.append(referenceValue2 + ((4*math.pi)/25))
        else:

            wrongList.append((2*math.pi)/25)
            wrongList.append((-(2*math.pi))/25)
            wrongList.append((4*math.pi)/25)
            wrongList.append((-(4*math.pi))/25)

        subtraction = math.remainder(unboundAngle/((2*math.pi)/25))
        period = (unboundAngle - subtraction)/((2*math.pi)/25)

        goalList: list[float] = [wrongList[i  + (period*((2*math.pi)/25))] for i in wrongList]

        for i in goalList:
            if not (unboundAngle - (math.pi/25))< goalList[i] < (unboundAngle + (math.pi/25)):
                wrongList.remove[i]



        return goalList

def optimizeTarget(
desiredRelativeRotations, unboundRotations
):
    # -> SwerveModuleState
    unboundAngle = unboundRotations*(2*math.pi)
    
    desiredRelativeRadians = desiredRelativeRotations*(2*math.pi)
    desY = np.sin(25*(desiredRelativeRadians))
    goal: list[float] = arcsinList(desY, unboundAngle)
    for i in goal:
        if goal[i] > 2*math.pi:
            remainderRadians = goal[i] % ((2*math.pi)/25)
            if (desiredRelativeRadians - 0.01) < remainderRadians < (desiredRelativeRadians + 0.01):
                target = (goal[i]/(2*math.pi))
        elif goal[i] < -2*math.pi:
            remainderRadians = goal[i] % ((2*math.pi)/25)
            if (desiredRelativeRadians - 0.01) < remainderRadians < (desiredRelativeRadians + 0.01):
                target = (goal[i]/(2*math.pi))
        else:
            if (desiredRelativeRadians - 0.01) < goal[i] < (desiredRelativeRadians + 0.01):
                target = (goal[i]/(2*math.pi))
        
    return target

