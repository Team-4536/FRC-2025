from wpilib import CAN, CANData
from manipulator import ManipulatorSubsystem
import math


class LEDSignals:
    def __init__(self, deviceId: int = 0):
        self.can = CAN(deviceId)
        self.period = 5
        self.counter = 0

    def update(self, manipulatorState: int, elevatorPos: float):
        if self.counter <= 0:
            try:
                byte_array = bytearray(2)
                byte_array[0] = max(math.floor(elevatorPos * (20 / 9)), 0)
                byte_array[1] = manipulatorState
                self.can.writePacket(byte_array, 0)
                self.LEDcounter = 0
            except e:
                print(e)

            self.counter = self.period
        else:
            self.counter = self.counter - 1
