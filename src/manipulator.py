from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from timing import TimeData
import wpilib

class ManipulatorSubsystem:

    IDLE = 0
    INTAKE = 1
    STORED = 2
    SHOOT = 3
    MANUAL = -1

    def __init__(self):
        
        self.state = self.IDLE
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        

    def update(self, buf:RobotHALBuffer, AButton:bool, LBumper:bool):
        
        if self.state == self.IDLE:
            buf.manipulatorVolts = 0
            if buf.manipulatorSensorReverse:
                self.state = self.INTAKE
            if LBumper:
                self.state = self.MANUAL

        elif self.state == self.INTAKE:
            buf.manipulatorVolts = 1

            if not buf.manipulatorSensorForward and not buf.manipulatorSensorReverse:
                self.state = self.IDLE

            if buf.manipulatorSensorForward and not buf.manipulatorSensorReverse:
                self.state = self.STORED
            if LBumper:
                self.state = self.MANUAL
                
                
        elif self.state == self.STORED:
            buf.manipulatorVolts = 0
            if AButton:
                self.state = self.SHOOT

            self.startTime = wpilib.getTime()
            if LBumper:
                self.state = self.MANUAL
            
        elif self.state == self.SHOOT:
            
            buf.manipulatorVolts = 5
            if wpilib.getTime() - self.startTime > 1.5:
                self.state = self.IDLE
            if LBumper:
                self.state = self.MANUAL

        elif self.state == self.MANUAL:
            if AButton:
                buf.manipulatorVolts = 5
            else:
                buf.manipulatorVolts = 0
            if LBumper:
                self.state = self.IDLE
            

        self.table.putNumber("state", self.state) 