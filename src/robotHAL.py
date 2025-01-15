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
    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        self.driveMotorFL.setVoltage(buf.driveVolts*1.6)
        self.table.putNumber('hal drive volts', buf.driveVolts)

        FL_Encoder = self.driveMotorFL.getEncoder()
        FL_PercentVoltage = self.driveMotorFL.getAppliedOutput()
        FL_Speed = FL_Encoder.getVelocity()
        FL_Position = FL_Encoder.getPosition()
        self.table.putNumber("DriveMotorFL Speed", FL_Speed)
        self.table.putNumber("DriveMotorFL Position", FL_Position)
        self.table.putNumber("DriveMotor percent voltage", FL_PercentVoltage)
        
        FL_Voltage = self.driveMotorFL.getAppliedOutput()*self.driveMotorFL.getBusVoltage()
        self.table.putNumber("DriveMotorFL voltage", FL_Voltage )

        myConfig = rev.SparkBaseConfig()
        myConfig.closedLoop.pid(.0001, 0, 0, rev.ClosedLoopSlot.kSlot0)
        # self.driveMotorFL.configure(
        #     myConfig, 
        #     rev.SparkBase.ResetMode.kNoResetSafeParameters, 
        #     rev.SparkBase.PersistMode.kNoPersistParameters)
        # #self.driveMotorFL.




