from wpilib import CAN, CANData
from manipulator import ManipulatorSubsystem
from elevator import ElevatorSubsystem
import math


class LEDSignals:
    def __init__(self, deviceId: int = 0):
        self.can = CAN(deviceId)
        self.period = 5
        self.counter = 0
        self.elevatorSubsystem = ElevatorSubsystem

    def update(
        self,
        manipulatorState: int,
        elevatorPos: float,
        elevatorMode: int,
        elevatorSetPoint: float,
        chuteState: int,
    ):
        if abs(self.elevatorSubsystem.INTAKE_POS - elevatorPos) <= 1:
            self.currentSetPoint = 0
        elif abs(self.elevatorSubsystem.L2_POS - elevatorPos) <= 1:
            self.currentSetPoint = 1
        elif abs(self.elevatorSubsystem.L3_POS - elevatorPos) <= 1:
            self.currentSetPoint = 2
        elif abs(self.elevatorSubsystem.L4_POS - elevatorPos) <= 1:
            self.currentSetPoint = 3
        else:
            self.currentSetPoint = 100

        if elevatorSetPoint == self.elevatorSubsystem.INTAKE_POS:
            self.simplifiedSetPoint = 0
        elif elevatorSetPoint == self.elevatorSubsystem.L2_POS:
            self.simplifiedSetPoint = 1
        elif elevatorSetPoint == self.elevatorSubsystem.L3_POS:
            self.simplifiedSetPoint = 2
        elif elevatorSetPoint == self.elevatorSubsystem.L4_POS:
            self.simplifiedSetPoint = 3
        else:
            self.simplifiedSetPoint = 100

        if self.counter <= 0:
            try:
                byte_array = bytearray(4)
                byte_array[0] = manipulatorState
                byte_array[1] = max(math.floor(elevatorPos * (20 / 9)), 0)
                byte_array[2] = elevatorMode
                byte_array[3] = self.simplifiedSetPoint
                byte_array[4] = chuteState
                self.can.writePacket(byte_array, 0)
                self.LEDcounter = 0
            except Exception as e:
                print(e)

            self.counter = self.period
        else:
            self.counter = self.counter - 1


from wpilib import CAN, CANData
from manipulator import ManipulatorSubsystem
import math


# class LEDSignals:
#     def __init__(self, deviceId: int = 0):
#         self.can = CAN(deviceId)
#         self.LEDcounter = 0

#     def update(
#         self,
#         manipulatorState: int,
#         elevatorPos: float,
#         elevMode,
#         mechPOV: int,
#     ):
#         if elevatorPos <= 1 and elevatorPos >= 0:
#             self.elevState = 0
#         elif elevatorPos <= 12 and elevatorPos >= 11:
#             self.elevState = 1
#         elif elevatorPos <= 25.5 and elevatorPos >= 23.5:
#             self.elevState = 2
#         elif elevatorPos <= 45 and elevatorPos >= 44:
#             self.elevState = 3
#         else:
#             self.elevState = 100

#         self.targetElevLevel = math.ceil(mechPOV / 90)

#         if elevMode == 0:
#             self.elevatorMode = 0
#         else:
#             self.elevatorMode = 1

#         self.LEDcounter += 1
#         if self.LEDcounter >= 10:
#             byte_array = bytearray(5)
#             byte_array[0] = math.floor(elevatorPos * (20 / 9))
#             byte_array[1] = self.elevatorMode
#             byte_array[2] = self.targetElevLevel
#             byte_array[3] = manipulatorState
#             byte_array[4] = self.elevState
#             self.can.writePacket(byte_array, 0)
#             self.LEDcounter = 0
