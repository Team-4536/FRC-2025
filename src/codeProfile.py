import wpilib
from ntcore import NetworkTableInstance
from typing import Callable


# this code is a bit crazy but the basic idea is that it allows for a decorator that takes arguments
def profile(instanceName: str):  # profile decorator factory
    def _profile(func: Callable):  # actual decorator
        table = (
            NetworkTableInstance.getDefault()
            .getTable("telemetry")
            .getSubTable("profiled code")
        )

        def wrapper(*args, **kwargs):
            startTime = wpilib.getTime()
            func(*args, **kwargs)
            # table.putNumber(func.__ func.__name__ + " time ms", (wpilib.getTime() - startTime) * 1000)
            # func + func.__name__,
            table.putNumber(
                instanceName + " " + func.__qualname__ + " time ms",
                (wpilib.getTime() - startTime) * 1000,
            )

        return wrapper

    return _profile
