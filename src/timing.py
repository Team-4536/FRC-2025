import wpilib

class TimeData:
    def __init__(self, prev) -> None:
        time = wpilib.getTime()

        if prev is None:
            self.initTime = time
            self.prevTime = time
            self.dt = 0
            self.timeSinceInit = 0
        else:
            self.dt = time - prev.prevTime
            self.timeSinceInit = time - prev.initTime
            self.prevTime = time
            self.initTime = prev.initTime
