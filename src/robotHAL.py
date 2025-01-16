import copy
import math
import robot


import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
from timing import TimeData
from ntcore import NetworkTableInstance

class RobotHALBuffer:
    def __init__(self) -> None:
        
        self.driveVolts = 0
    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


        



class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

        self.driveMotorFL = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorFR = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBL = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBR = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.driveMotorP = 0.0
        self.table.putNumber("driveMotorP",self.driveMotorP)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        self.driveMotorFL.set(0.1)
        self.driveMotorFL.setVoltage(1.2)
        self.table.putNumber('hal drive volts', buf.driveVolts)

        bob = self.driveMotorFL.getEncoder()
        SpeedOfMotor = bob.getVelocity()
        self.table.putNumber("DriveMotorFL Speed",SpeedOfMotor)
        PositionOfMotor = bob.getPosition()
        self.table.putNumber("My Position Title in DashBoard",PositionOfMotor)
        self.driveMotorFL.getAppliedOutput()

        self.table.getNumber("driveMotorP",DriveMotorP)

        bobsactualvolt = self.driveMotorFL.getAppliedOutput()*self.driveMotorFL.getBusVoltage()
        myConfig = rev.SparkBaseConfig()
        myConfig.closedLoop.pid(0.01,0.0,0.0,rev.ClosedLoopSlot.kSlot0)
        self.driveMotorFL.configure(
            myConfig,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters)
        self.driveMotorFL.getClosedLoopController().setReference(1.0)
        


