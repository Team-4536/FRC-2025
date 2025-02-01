from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance


class ElevatorSubsystem:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("Elevator setpoint offset", 0)
        self.table.putNumber("Elevator arbFF offset", 0)
        self.setpoint = 3

    def update(self, hal: RobotHALBuffer, up: float, down: float):
        self.table.putNumber("Elevator up", up)
        self.table.putNumber("Elevator down", down)
        if up < 0.1:
            up = 0
        if down < 0.1:
            down = 0

        self.setpoint += 0.5 * up + (-0.5 * down)
        if self.setpoint < 1:
            self.setpoint = 1
        if self.setpoint > 50:
            self.setpoint = 50
        hal.elevatorSetpoint = self.setpoint + self.table.getNumber(
            "Elevator setpoint offset", 0
        )
        hal.elevatorArbFF = 0.3 + self.table.getNumber("Elevator arbFF offset", 0)
