import wpilib
from timing import TimeData  
import rev
from rev import SparkMax

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.driveCtrlr = wpilib.XboxController(0)

        self.Motor =  SparkMax(2, rev.SparkMax.MotorType.kBrushless)
        self.Motor.set(0)
        
    def robotPeriodic(self) -> None:
       pass
        
    def teleopInit(self) -> None:
        self.Motor.set(0.2)

    def teleopPeriodic(self):
        pass
       

if __name__ == "__main__":
    wpilib.run(Robot)
