from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance


class Cassiosubsystem:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(self, LT: int, RT: int, halBuffer: RobotHALBuffer):
        if LT > 0.05:
            halBuffer.elevatorVoltage = (LT * 2) * -1
        elif RT > 0.05:
            halBuffer.elevatorVoltage = RT * 2
        else:
            halBuffer.elevatorVoltage = 0
