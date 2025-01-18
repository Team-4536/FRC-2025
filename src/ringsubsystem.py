from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from timing import TimeData
import wpilib

class ringSubsystem:
    
    IDLE = 0
    INTAKE = 1
    STORED = 2
    EJECT = 3

    def __init__(self):
        self.state = self.IDLE

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        
       


    def update(self, halBuf: RobotHALBuffer, YButton: bool, XButton: bool, time: TimeData, wpilib: wpilib):
        
        if self.state == self.IDLE:
            halBuf.intakeVoltage = 0
            halBuf.intakeVoltageTwo = 0
            
            if YButton:
                self.state = self.INTAKE

        elif self.state == self.INTAKE:
            halBuf.intakeVoltage = 1.56
            halBuf.intakeVoltageTwo = 1.56
            
            if halBuf.ringSensorValue:
                self.state = self.STORED

            if not halBuf.ringSensorValue and not YButton:
                self.state = self.IDLE

        elif self.state == self.STORED:
            halBuf.intakeVoltage = 0
            halBuf.intakeVoltageTwo = 0

            self.startTime = wpilib.getTime()

            if XButton:
                self.state = self.EJECT
        
        elif self.state == self.EJECT:
            halBuf.intakeVoltage = -2
            halBuf.intakeVoltageTwo = -2

            if(wpilib.getTime() - self.startTime > 1.0):
                self.state = self.IDLE

        self.table.putNumber("state", self.state)    

            


        #if YButton:
            #halBuf.intakeVoltage = 0.9
            #halBuf.intakeVoltageTwo = 0.9

        #if XButton:
            #halBuf.intakeVoltage = -0.9
            #halBuf.intakeVoltageTwo = -0.9
        #else: 
            #if halBuf.ringSensorValue:
                #halBuf.intakeVoltage = 0
                #halBuf.intakeVoltageTwo = 0  
        

        
        
        


