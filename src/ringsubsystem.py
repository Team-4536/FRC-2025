from robotHAL import RobotHALBuffer


class ringSubsystem:
    
    def __init__(self):
        pass

    def update(self, halBuf: RobotHALBuffer, YButton: bool, XButton: bool):
        
        if YButton:
            halBuf.intakeVoltage = 0.9
            halBuf.intakeVoltageTwo = 0.9

        if XButton:
            halBuf.ejectionVoltage = -0.9
            halBuf.ejectionVoltage = -0.9

        if halBuf.ringSensorValue:
            halBuf.intakeVoltage = 0
            halBuf.intakeVoltageTwo = 0
        

        
        
        


