
from robotHAL import RobotHALBuffer
from robotHAL import RobotHAL
from ntcore import NetworkTableInstance
from timing import TimeData
import wpilib

class ElevatorSubsystem:

    IDLE = 0
    LVLONE = 1
    LVLTWO = 2
    LVLTHREE = 3
    
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.state = self.IDLE

    def update(self, pov:int, buf: RobotHALBuffer):
        
        if self.state == self.IDLE:
            buf.elevatorVoltage = 0
            buf.elevatorRotations = 0
            if pov == 90:
                buf.elevatorVoltage = 1
                if buf.elevatorRotations == 5:
                    self.state = self.LVLONE
            elif pov == 180:
                buf.elevatorVoltage = 1
                if buf.elevatorRotations == 10:
                    self.state = self.LVLTWO
            elif pov == 270:
                buf.elevatorVoltage = 1
                if buf.elevatorRotations == 15:
                    self.state = self.LVLTHREE
            

        elif self.state == self.LVLONE:
            buf.elevatorVoltage = 0
            buf.elevatorRotations = 0
            if pov == 0:
                buf.elevatorVoltage = -1
                if buf.elevatorRotations == 5:
                    self.state = self.IDLE
            elif pov == 180:
                buf.elevatorVoltage = 1
                if buf.elevatorRotations == 5:
                    self.state = self.LVLTWO
            elif pov == 270:
                buf.elevatorVoltage = 1
                if buf.elevatorRotations == 10:
                    self.state = self.LVLTHREE


        elif self.state == self.LVLTWO:
            buf.elevatorVoltage = 0
            buf.elevatorRotations = 0
            if pov == 0:
                buf.elevatorVoltage = -1
                if buf.elevatorRotations == 10:
                    self.state = self.IDLE
            elif pov == 90:
                buf.elevatorVoltage = -1
                if buf.elevatorRotations == 5:
                    self.state = self.LVLONE
            elif pov == 270:
                buf.elevatorVoltage = 1
                if buf.elevatorRotations == 5:
                    self.state = self.LVLTHREE


        elif self.state == self.LVLTHREE:
            buf.elevatorVoltage = 0
            buf.elevatorRotations = 0
            if pov == 0:
                buf.elevatorVoltage = -1
                if buf.elevatorRotations == 15:
                    self.state = self.IDLE
            elif pov == 90:
                buf.elevatorVoltage = -1
                if buf.elevatorRotations == 10:
                    self.state = self.LVLONE
            elif pov == 180:
                buf.elevatorVoltage = -1
                if buf.elevatorRotations == 5:
                    self.state = self.LVLTWO

        self.table.putNumber("state", self.state)
        self.table.putNumber("elevator volts", buf.elevatorVoltage)

