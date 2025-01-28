import copy
import math
import robot
import robotHAL
import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
from timing import TimeData
from ntcore import NetworkTableInstance

class revPID:
    def __init__(self, driveMotorIn: rev.SparkMax)->None:
        
        self.driveMotor = driveMotorIn
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveMotorEncoder = self.driveMotor.getEncoder()
        self.driveMotorPercentVoltage = self.driveMotor.getAppliedOutput()
        self.driveMotorSpeedFeedback = self.driveMotor.getVelocity()
        self.driveMotorPosition = self.driveMotor.getPosition()
        self.driveMotorVoltage = self.driveMotor.getAppliedOutput()*self.driveMotor.getBusVoltage()

        self.table.putNumber("DriveMotorFL Speed", self.driveMotorSpeedFeedback)
        self.table.putNumber("DriveMotorFL Position", self.driveMotorPosition)
        self.table.putNumber("DriveMotorFL percentVoltage", self.driveMotorPercentVoltage)
        self.table.putNumber("DriveMotorFL Voltage", self.driveMotorVoltage)

        self.driveMotorClosedLoopControl = self.driveMotor.getClosedLoopController()
        self.driveMotorUniversalConfig = rev.SparkBaseConfig()
        self.driveMotorUniversalConfig.closedLoop.pidf(robotHAL.driveMotorUniversalP, 0, 0, 1/473, rev.ClosedLoopSlot.kSlot0)

        

    def periodic(self)->None:
        pass

    def update(self)->None:

        self.driveMotor.configure(
            robotHAL.driveMotorUniversalConfig, 
            rev.SparkBase.ResetMode.kNoResetSafeParameters, 
            rev.SparkBase.PersistMode.kNoPersistParameters)
        
        
        self.driveMotorClosedLoopControl.setReference(robotHAL.driveMotorUniversalDesiredSpeed, rev.SparkBase.ControlType.kVelocity)
       
        self.table.putNumber("desiredSpeedConfirm", robotHAL.driveMotorUniversalDesiredSpeed)

        self.driveMotorEncoder = self.driveMotor.getEncoder()
        self.driveMotorPercentVoltage = self.driveMotor.getAppliedOutput()
        self.driveMotorSpeedFeedback = self.driveMotor.getVelocity()
        self.driveMotorPosition = self.driveMotor.getPosition()
        self.driveMotorVoltage = self.driveMotor.getAppliedOutput()*self.driveMotor.getBusVoltage()

        self.table.putNumber("DriveMotorFL Speed", self.driveMotorSpeedFeedback)
        self.table.putNumber("DriveMotorFL Position", self.driveMotorPosition)
        self.table.putNumber("DriveMotorFL percentVoltage", self.driveMotorPercentVoltage)
        self.table.putNumber("DriveMotorFL Voltage", self.driveMotorVoltage)