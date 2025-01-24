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
        self.driveVolts = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


        



class RobotHAL:
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

        self.driveMotorFL = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorFR = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBL = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        self.driveMotorBR = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)

        self.driveMotorFLPID = revPID(self.driveMotorFL)
        self.driveMotorFLPID = revPID(self.driveMotorFR)
        self.driveMotorFLPID = revPID(self.driveMotorBL)
        self.driveMotorFLPID = revPID(self.driveMotorBL)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        

        # self.driveMotorFL.setVoltage(buf.driveVolts*1.6)
        # self.table.putNumber('hal drive volts', buf.driveVolts)  

        # driveMotorFREncoder = self.driveMotorFR.getEncoder()
        # driveMotorFRPercentVoltage = self.driveMotorFR.getAppliedOutput()
        # driveMotorFRSpeedFeedback = driveMotorFREncoder.getVelocity()
        # driveMotorFRPosition = driveMotorFREncoder.getPosition()
        # driveMotorFRVoltage = self.driveMotorFR.getAppliedOutput()*self.driveMotorFR.getBusVoltage()
        # self.table.putNumber("DriveMotorFL Speed", driveMotorFRSpeedFeedback)
        # self.table.putNumber("DriveMotorFL Position", driveMotorFRPosition)
        # self.table.putNumber("DriveMotorFL percentVoltage", driveMotorFRPercentVoltage)
        # self.table.putNumber("DriveMotorFL Voltage", driveMotorFRVoltage)

        # driveMotorBLEncoder = self.driveMotorBL.getEncoder()
        # driveMotorBLPercentVoltage = self.driveMotorBL.getAppliedOutput()
        # driveMotorBLSpeedFeedback = driveMotorBLEncoder.getVelocity()
        # driveMotorBLPosition = driveMotorBLEncoder.getPosition()
        # driveMotorBLVoltage = self.driveMotorBL.getAppliedOutput()*self.driveMotorBL.getBusVoltage()
        # self.table.putNumber("DriveMotorFL Speed", driveMotorBLSpeedFeedback)
        # self.table.putNumber("DriveMotorFL Position", driveMotorBLPosition)
        # self.table.putNumber("DriveMotorFL percentVoltage", driveMotorBLPercentVoltage)
        # self.table.putNumber("DriveMotorFL Voltage", driveMotorBLVoltage)

        # driveMotorBREncoder = self.driveMotorBR.getEncoder()
        # driveMotorBRPercentVoltage = self.driveMotorBR.getAppliedOutput()
        # driveMotorBRSpeedFeedback = driveMotorBREncoder.getVelocity()
        # driveMotorBRPosition = driveMotorBREncoder.getPosition()
        # driveMotorBRVoltage = self.driveMotorBR.getAppliedOutput()*self.driveMotorBR.getBusVoltage()
        # self.table.putNumber("DriveMotorFL Speed", driveMotorBRSpeedFeedback)
        # self.table.putNumber("DriveMotorFL Position", driveMotorBRPosition)
        # self.table.putNumber("DriveMotorFL percentVoltage", driveMotorBRPercentVoltage)
        # self.table.putNumber("DriveMotorFL Voltage", driveMotorBRVoltage)
        

        #driveMotorUniversalConfig.closedLoop.P(self.driveMotorFLP)
        
    

class revPID:
    def __init__(self, driveMotorIn)->None:

        self.driveMotor = driveMotorIn
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveMotorUniversalP = 0
        self.table.putNumber("DriveMotorP", self.driveMotorFLP)
        self.driveMotorUniversalDesiredSpeed = 0
        self.table.putNumber("DriveMotorFLDesiredSpeed", self.driveMotorUniversalDesiredSpeed)
        

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
        self.driveMotorUniversalConfig.closedLoop.pidf(self.driveMotorUniversalP, 0, 0, 1/473, rev.ClosedLoopSlot.kSlot0)

        

    def periodic(self)->None:
        pass

    def update(self)->None:

        self.driveMotorUniversalP = self.table.getNumber("DriveMotorP", self.driveMotorUniversalP)
        self.driveMotorUniversalDesiredSpeed = self.table.getNumber("DriveMotor UniversalDesiredSpeed", self.driveMotorUniversalDesiredSpeed)
        
        self.driveMotor.configure(
            self.driveMotorUniversalConfig, 
            rev.SparkBase.ResetMode.kNoResetSafeParameters, 
            rev.SparkBase.PersistMode.kNoPersistParameters)
        
        
        self.driveMotorClosedLoopControl.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity)
       
        self.table.putNumber("desiredSpeedConfirm", self.desiredSpeed)

        self.driveMotorEncoder = self.driveMotor.getEncoder()
        self.driveMotorPercentVoltage = self.driveMotor.getAppliedOutput()
        self.driveMotorSpeedFeedback = self.driveMotor.getVelocity()
        self.driveMotorPosition = self.driveMotor.getPosition()
        self.driveMotorVoltage = self.driveMotor.getAppliedOutput()*self.driveMotor.getBusVoltage()

        self.table.putNumber("DriveMotorFL Speed", self.driveMotorSpeedFeedback)
        self.table.putNumber("DriveMotorFL Position", self.driveMotorPosition)
        self.table.putNumber("DriveMotorFL percentVoltage", self.driveMotorPercentVoltage)
        self.table.putNumber("DriveMotorFL Voltage", self.driveMotorVoltage)
            
        




