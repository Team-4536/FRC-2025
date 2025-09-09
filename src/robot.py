import wpilib
from timing import TimeData    

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.time = TimeData(None)

        self.driveCtrlr = wpilib.XboxController(0)
        
    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        
    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.stopMotors()  # Keep this at the top of teleopPeriodic
        
    def autonomousInit(self) -> None:
        self.stopMotors()

    def autonomousPeriodic(self) -> None:
        self.stopMotors()  # Keep this at the top of autonomousPeriodic
    
    def stopMotors(self) -> None:
        self.manipulatorVolts = 0
        self.armVolts = 0

       

if __name__ == "__main__":
    wpilib.run(Robot)
