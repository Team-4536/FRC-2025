import wpilib
from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance

class IntakeChute:
    
    def __init__(self):
        
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.chuteSpeed = 5
        self.setChuteVoltage = 0
        self.table.putNumber("Set Chute Voltage", self.setChuteVoltage)
        self.setChuteControlMode = 3
        self.table.putNumber("Chute Control Mode", self.setChuteControlMode)
        self.ctrlrBumber = 0
        self.resetMode3 = True

    def update(self, hal: RobotHALBuffer, rightBumber, leftBumber):
        
        self.table.putNumber("Intake Chute Voltage", hal.chuteMotorVoltage)
        self.table.getNumber("Chute Control Mode", self.setChuteControlMode)
        self.currentTime = wpilib.getTime()
        self.table.putBoolean("limit switch", hal.chuteLimitSwitch)

        if leftBumber:
            self.ctrlrBumber = -1
        elif rightBumber:
            self.ctrlrBumber = 1
        else:
            self.ctrlrBumber = 0
        

        if self.setChuteControlMode == 1:
            
            hal.setChuteVoltage =  self.ctrlrBumber*self.chuteSpeed # POSITIVE Pulls down Negative lets up

        elif self.setChuteControlMode == 2:
            
            if hal.chuteLimitSwitch == False: #returns true if the hal.chuteLimitSwitch is pressed
                
                hal.setChuteVoltage = self.chuteSpeed

            else:
                
                hal.setChuteVoltage = 0
                self.resetMode3 = True
                self.setChuteControlMode = 1
                self.table.putNumber("Chute Control Mode", self.setChuteControlMode)
        
        elif self.setChuteControlMode == 3:

            self.table.putBoolean("mode 3 reset", self.resetMode3)

            if self.resetMode3:
                self.startTime = wpilib.getTime()
                self.resetMode3 = False

            self.currentTime = wpilib.getTime()

            if self.currentTime - self.startTime < 2:

                hal.setChuteVoltage = -self.chuteSpeed
                

            else:
                
                hal.setChuteVoltage = 0