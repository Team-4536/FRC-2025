from wpilib import CAN, CANData
from manipulator import ManipulatorSubsystem
import math


class LEDSignals:
    def __init__(self, deviceId: int = 0):
        self.can = CAN(deviceId)

    def update(self, manipulatorState: int, elevatorPos: float):
        byte_array = bytearray(2)
        # LEDdata: list[int] = [math.floor(elevatorPos * (20 / 9)), manipulatorState]
        # byte_array = bytearray(LEDdata)
        byte_array[0] = math.floor(elevatorPos * (20 / 9))
        byte_array[1] = manipulatorState
        self.can.writePacket(byte_array, 0)
