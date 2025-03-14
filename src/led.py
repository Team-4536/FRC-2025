from wpilib import CAN, CANData
from manipulator import ManipulatorSubsystem
import math


class LEDSignals:
    def __init__(self, deviceId: int = 0):
        self.can = CAN(deviceId)
        self.LEDcounter = 0

    def update(
        self,
        manipulatorState: int,
        elevatorPos: float,
        elevMode,
        mechPOV: int,
    ):
        if elevatorPos <= 1 and elevatorPos >= 0:
            self.elevState = 0
        elif elevatorPos <= 12 and elevatorPos >= 11:
            self.elevState = 1
        elif elevatorPos <= 25.5 and elevatorPos >= 23.5:
            self.elevState = 2
        elif elevatorPos <= 45 and elevatorPos >= 44:
            self.elevState = 3
        else:
            self.elevState = 100

        self.targetElevLevel = math.ceil(mechPOV / 90)

        if elevMode == 0:
            self.elevatorMode = 0
        else:
            self.elevatorMode = 1

        self.LEDcounter += 1
        if self.LEDcounter >= 10:
            byte_array = bytearray(5)
            byte_array[0] = math.floor(elevatorPos * (20 / 9))
            byte_array[1] = self.elevatorMode
            byte_array[2] = self.targetElevLevel
            byte_array[3] = manipulatorState
            byte_array[4] = self.elevState
            self.can.writePacket(byte_array, 0)
            self.LEDcounter = 0
