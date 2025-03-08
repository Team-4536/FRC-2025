import wpilib
from ntcore import NetworkTableInstance
from wpimath.units import seconds

timeList: list[seconds] = []


def start():
    global timeList
    timeList.append(wpilib.getTime())


def end(title: str):
    global timeList
    NetworkTableInstance.getDefault().getTable("profiling").putNumber(
        title + " ms", (wpilib.getTime() - timeList.pop()) * 1000
    )
