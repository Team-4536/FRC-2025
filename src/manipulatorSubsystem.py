
from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from timing import TimeData
import wpilib
class manipulatorSub:

    IDLE = 0
    INTAKE = 1
    STORED = 2
    SHOOT = 3

    def __init__(self):
        
        self.state = self.IDLE
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        

    def update(self, buf:RobotHALBuffer, YButton:bool, backSensor:bool):
        
        if self.state == self.IDLE:
            buf.rollerVoltage = 0
            if backSensor:
                self.state = self.INTAKE

        elif self.state == self.INTAKE:
            buf.rollerVoltage = 1

            if buf.motorSensorValue and not backSensor:
                self.state = self.IDLE

            if not buf.motorSensorValue:
                self.state = self.STORED
                
                

        elif self.state == self.STORED:
            buf.rollerVoltage = 0
            if YButton:
                self.state = self.SHOOT

            self.startTime = wpilib.getTime()
            
        elif self.state == self.SHOOT:
            
            buf.rollerVoltage = 1.75
            if wpilib.getTime() - self.startTime > 2.6:
                self.state = self.IDLE

        self.table.putNumber("state", self.state) 
        self.table.putBoolean("sensor", buf.motorSensorValue)