from wpilib import CAN, CANData
from manipulator import ManipulatorSubsystem
import math


class LEDSignals:
    def __init__(self, deviceId: int = 0):
        self.can = CAN(deviceId)
        self.LEDcounter = 0

    def update(self, manipulatorState: int, elevatorPos: float):
        self.LEDcounter += 1
        if self.LEDcounter >= 10:
            byte_array = bytearray(2)
            byte_array[0] = math.floor(elevatorPos * (20 / 9))
            byte_array[1] = manipulatorState
            self.can.writePacket(byte_array, 0)
            self.LEDcounter = 0
