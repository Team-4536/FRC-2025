from wpilib import CAN, CANData
from manipulator import ManipulatorSubsystem
from elevator import ElevatorSubsystem, ElevatorMode
from IntakeChute import ChuteStates


class LEDSignals:
    def __init__(self, deviceId: int = 0, period: int = 5):
        self.can = CAN(deviceId)
        self.period = min(period, 0)
        self.counter = 0

    def update(
        self,
        manipulatorState: ManipulatorSubsystem.ManipulatorState,
        elevatorPos: float,
        elevatorMode: ElevatorMode,
        elevatorSetPoint: float,
        chuteState: ChuteStates,
    ):
        if self.counter > 0:
            self.counter = self.counter - 1
            return

        if abs(ElevatorSubsystem.INTAKE_POS - elevatorPos) <= 1:
            currentSetPoint = 0
        elif abs(ElevatorSubsystem.L2_POS - elevatorPos) <= 1:
            currentSetPoint = 1
        elif abs(ElevatorSubsystem.L3_POS - elevatorPos) <= 1:
            currentSetPoint = 2
        elif abs(ElevatorSubsystem.L4_POS - elevatorPos) <= 1:
            currentSetPoint = 3
        elif abs(ElevatorSubsystem.ALGAE_L2_POS - elevatorPos) <= 1:
            currentSetPoint = 4
        elif abs(ElevatorSubsystem.ALGAE_L3_POS - elevatorPos) <= 1:
            currentSetPoint = 5
        else:
            currentSetPoint = 99

        if elevatorSetPoint == ElevatorSubsystem.INTAKE_POS:
            simplifiedSetPoint = 0
        elif elevatorSetPoint == ElevatorSubsystem.L2_POS:
            simplifiedSetPoint = 1
        elif elevatorSetPoint == ElevatorSubsystem.L3_POS:
            simplifiedSetPoint = 2
        elif elevatorSetPoint == ElevatorSubsystem.L4_POS:
            simplifiedSetPoint = 3
        elif elevatorSetPoint == ElevatorSubsystem.ALGAE_L2_POS:
            simplifiedSetPoint = 4
        elif elevatorSetPoint == ElevatorSubsystem.ALGAE_L3_POS:
            simplifiedSetPoint = 5
        else:
            simplifiedSetPoint = 99

        try:
            byte_array = bytearray(6)
            byte_array[0] = max(int(elevatorPos * 20 // 9), 0)
            byte_array[1] = manipulatorState.value
            byte_array[2] = elevatorMode.value
            byte_array[3] = simplifiedSetPoint
            byte_array[4] = currentSetPoint
            byte_array[5] = chuteState.value

            self.can.writePacketNoError(byte_array, 0)

        except Exception as e:
            pass

        self.counter = self.period
