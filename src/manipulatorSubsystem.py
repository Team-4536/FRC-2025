
from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance

class manipulatorSub:

    IDLE = 0
    INTAKE = 1
    STORED = 2
    SHOOT = 3

    def __init__(self):
        
        self.state = self.IDLE
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        

    def update(self, buf:RobotHALBuffer, YButton:bool, XButton:bool):
        
        if self.state == self.IDLE:
            buf.rollerVoltage = 0
            if XButton:
                self.state = self.INTAKE

        elif self.state == self.INTAKE:
            buf.rollerVoltage = 1

            if not buf.motorSensorValue and not XButton:
                self.state = self.IDLE

            if buf.motorSensorValue:
                self.state = self.STORED
                
                

        elif self.state == self.STORED:
            buf.rollerVoltage = 0
            if YButton:
                self.state = self.SHOOT
            
        elif self.state == self.SHOOT:
            buf.rollerVoltage = 1
            if not buf.motorSensorValue:
                self.state = self.IDLE

        self.table.putNumber("state", self.state) 
        self.table.putBoolean("sensor", buf.motorSensorValue)