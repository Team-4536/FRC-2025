import wpilib
from wpilib import CAN, CANData
from elevator import ElevatorSubsystem
from manipulator import ManipulatorSubsystem
import manipulator
from robotHAL import RobotHALBuffer


class LEDSignals:

    def __init__(self, deviceId: int = 0):

        self.can = CAN(deviceId)

    def update(
        self, manipulatorState: ManipulatorSubsystem.ManipulatorState, elevatorPos: int
    ):
        byte_array = bytearray(2)

        byte_array[0] = elevatorPos * (100 / 45)
        byte_array[1] = manipulatorState

        self.can.writePacket(byte_array, 0)
