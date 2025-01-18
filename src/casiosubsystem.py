from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance

class Cassiosubsystem:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(self, controllerinput:bool, halBuffer:RobotHALBuffer):
        if not halBuffer.laservalue:
            if controllerinput:
                self.table.putBoolean("Is Ready", False)
                halBuffer.bottommotorvolts = 1
                halBuffer.topmotorvolts = 1
            else:
                self.table.putBoolean("Is Ready", False)
                halBuffer.bottommotorvolts = 0
                halBuffer.topmotorvolts = 0
            
        else:
            if controllerinput:
                self.table.putBoolean("Is Ready", True)
                halBuffer.bottommotorvolts = -1
                halBuffer.topmotorvolts = -1
            else:
                self.table.putBoolean("Is Ready", True)
                halBuffer.bottommotorvolts = 0
                halBuffer.topmotorvolts = 0
            