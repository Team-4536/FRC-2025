import robot
from typing import TYPE_CHECKING, Callable
from ntcore import NetworkTableInstance, NetworkTable

# from __future__ import annotations

if TYPE_CHECKING:
    from robot import Robot


class AutoStage:
    def __init__(self):
        self.name = (
            "give every AutoStage a unique name with __init__ args"  # + str(numSec)
        )
        # all auto stages should report their name and optionally other debug info
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        # This is the constructor (startup code), declare your variables here.
        # It will be called when the robot starts.
        pass

    def setNext(self, nextAS: "AutoStage"):
        self.nextStage = nextAS
        pass

    def autoInit(self, r: "Robot"):
        # This is similar to __init__ but it will be called immediately before the Auto Stage
        # begins, it is useful for resetting encode positions or reseting gyros, etc.
        #  Anything that should be reset each auto run.
        pass

    def run(self, r: "Robot"):
        # this will be called repeatedly every 20ms (e.g., loop())
        pass

    def isDone(self, r: "Robot") -> bool:
        # this will be called at the end of the run() and if it returns true will stop the stage
        return True

    def getNext(self, r: "Robot"):
        # This Function contains a string that indicates which stage runs next. It must match
        # those strings created in robot.py (TOOD: Find a better way to link strings to AS objs.)
        self.nextStage
        pass


class myCustomAS(AutoStage):
    def __init__(self, numSec: float = 20):
        self.name = "myCustomAS:" + str(numSec)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.myCounterStart: float = 5 * numSec  # 20ms per cycle, 50 cycles = 1 second
        self.myCounter = self.myCounterStart

        pass

    def run(self, r: "Robot"):
        self.myCounter = self.myCounter - 1
        r.Motor.set(0.2)
        self.table.putString("AutoStage", self.name)
        self.table.putNumber("Counter Remaining", self.myCounter)

        pass

    def isDone(self, r: "Robot") -> bool:
        self.table.putNumber("test", self.myCounter)
        if self.myCounter <= 0:
            r.Motor.set(0.0)
            return True  # are these backward?
        return False

    def autoInit(self, r: "Robot"):
        self.myCounter = self.myCounterStart  # 20ms per cycle, 50 cycles = 1 second
        pass

    def setNext(self, nextAS: AutoStage):
        self.nextStage = nextAS
        pass

    # Example setNext() function for branching auto
    # def setNext(self, nextAS1: AutoStage, nextAS2: AutoStage):
    #     self.nextStage1 = nextAS1
    #     self.nextStage2 = nextAS2
    #     pass

    def getNext(self, r: "Robot"):
        # Example Branching Auto Code:
        # if True:
        #     self.nextStage = self.nextStage1
        # else:
        #     self.nextStage = self.nextStage2
        return self.nextStage
        pass


class allStopAS(AutoStage):
    # Every Robot must have a Auto Stage that safely stops all activity!
    def __init__(self):
        self.name = "allStopAS:"
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        pass

    def run(self, r: "Robot"):
        self.table.putString("AutoStage", self.name)
        r.Motor.set(0.0)
        pass

    def isDone(self, r: "Robot") -> bool:
        return False

    def autoInit(self, r: "Robot"):
        pass

    def setNext(self):
        pass

    def getNext(self, r: "Robot"):
        return self
        pass


## TODO: Replace counter with a Timer Object
## TODO: Remove "Robot" notation and include __future__ import
## TODO: Seperate linear example from branching example autostage
## TODO: Add a pause Autostage (of veriable length)
## TODO: Move r: "Robot" into contructor so you don't have to call it in
##          every stage...open question about what we should name it
##          it would be confusing to use self.r.motor.set() in AS and self.motor.set()
##          integration with the HAL may clear this up anyway...
## TODO: Remove djo from lib name
