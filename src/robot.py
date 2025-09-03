# The goal of this first practice problem is to create a timer 
# using timing.py that prints "Boo!" after 100 milleseconds

# Comments will be at the end as hints to help guide you
# --------------------------------------------------------------------------


import wpilib
import timing
from timing import TimeData

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.timer = TimeData(None)
        
    def robotPeriodic(self) -> None:
        self.time = self.timer.timeSinceInit

        if self.time > 5:
            print("Boo!")

    def teleopInit(self) -> None:
        pass
    
    def teleopPeriodic(self) -> None:
        
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)

# ------------------------------------------------------------------------------

# import wpilib

# ---1. import desired file
# ---2. from the desired file import the desired class definition

# class Robot(wpilib.TimedRobot):
#     def robotInit(self) -> None:

# ---3. create new time variable using instance of the time class
            # ---* creating an instance of the class requires one argument try to
            #      figure out what it is

#     def robotPeriodic(self) -> None:

# ---4. update the time variable to the timeSinceInit variable from the timer class
# ---5. create an if statement to check if enough time has passed
# ---6. if enough time has passed use the print function to print "Boo!"

#     def teleopInit(self) -> None:
#         pass
#     def teleopPeriodic(self) -> None:
        
#         pass

#     def autonomousInit(self) -> None:
#         pass

#     def autonomousPeriodic(self) -> None:
#         pass

#     def disabledInit(self) -> None:
#         self.disabledPeriodic()

#     def disabledPeriodic(self) -> None:
#         pass


# if __name__ == "__main__":
#     wpilib.run(Robot)
