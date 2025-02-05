from robotHAL import RobotHALBuffer
from ntcore import NetworkTableInstance
from wpilib import Timer


class ElevatorSubsystem:
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("Elevator setpoint offset", 0)
        self.table.putNumber("Elevator arbFF offset", 0)
        self.setpoint = 3
        self.speed = 0
        self.maxSpeed = 1
        self.previousTime = Timer.getFPGATimestamp()
        self.repeats = 0

    def update(self, hal: RobotHALBuffer, up: float, down: float):
        self.currentTime = Timer.getFPGATimestamp()
        self.elapsed = self.currentTime - self.previousTime
        self.previousTime = self.currentTime
        self.repeats = 0.2 / self.elapsed

        self.table.putNumber("Elevator up", up)
        self.table.putNumber("Elevator down", down)
        if up < 0.1:
            up = 0
        if down < 0.1:
            down = 0

        if self.speed < self.maxSpeed:
            if up > 0.1:
                if (
                    self.maxSpeed
                    >= self.speed + ((up * 0.1) * self.maxSpeed) / self.repeats
                ):
                    self.speed += ((up / 10) * self.maxSpeed) / self.repeats
                else:
                    self.speed = self.maxSpeed
        elif self.speed > self.maxSpeed * -1:
            if down > 0.1:
                if (
                    self.maxSpeed * -1
                    >= self.speed - ((down / 10) * self.maxSpeed) / self.repeats
                ):
                    self.speed -= ((down / 10) * self.maxSpeed) / self.repeats
                else:
                    self.speed = self.maxSpeed * -1
        elif self.speed >= 0 + (self.maxSpeed / 10) / self.repeats:
            self.speed -= (self.maxSpeed / 10) / self.repeats
        elif self.speed <= 0 - (self.maxSpeed / 10) / self.repeats:
            self.speed += (self.maxSpeed / 10) / self.repeats
        else:
            self.speed = 0

        # self.setpoint += 0.5 * self.up + (-0.5 * self.down)
        self.setpoint += self.speed

        if self.setpoint < 0:
            self.setpoint = 0
        if self.setpoint > 50:
            self.setpoint = 50
        hal.elevatorSetpoint = self.setpoint + self.table.getNumber(
            "Elevator setpoint offset", 0
        )
        hal.elevatorArbFF = 0.3 + self.table.getNumber("Elevator arbFF offset", 0)
