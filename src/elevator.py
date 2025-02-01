from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance


class ElevatorSubsystem:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("Elevator setpoint offset", 0)
        self.table.putNumber("Elevator arbFF offset", 0)
        self.setpoint = 0

    def update(self, hal: RobotHALBuffer, up: bool, down: bool):
        self.table.putNumber("Elevator up", up)
        self.table.putNumber("Elevator down", down)
        if up:
            self.setpoint += 0.001
        if down:
            self.setpoint -= 0.001
        if self.setpoint < 0:
            self.setpoint = 0
        if self.setpoint > 10:
            self.setpoint = 10
        hal.elevatorSetpoint = self.setpoint + self.table.getNumber(
            "Elevator setpoint offset", 0
        )
        hal.elevatorArbFF = self.table.getNumber("Elevator arbFF offset", 0)
