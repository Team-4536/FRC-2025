import wpilib
from ntcore import NetworkTableInstance
from wpimath.units import seconds


class Profiler:
    profilingNT = NetworkTableInstance.getDefault().getTable("profiling")

    def __init__(self, name: str) -> None:
        self.entry = self.profilingNT.getEntry(name + " (ms)")
        self.startTime: seconds = 0

    def start(self) -> None:
        self.startTime = wpilib.getTime()

    def end(self) -> None:
        self.entry.setFloat((wpilib.getTime() - self.startTime) * 1000)
