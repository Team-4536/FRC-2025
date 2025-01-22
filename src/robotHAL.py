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


        self.driveMotorFLPID = revPID(self.driveMotorFL) 
        self.driveMotorFRPID = revPID(self.driveMotorFR) 
        self.driveMotorBLPID = revPID(self.driveMotorBL) 
        self.driveMotorBRPID = revPID(self.driveMotorBR) 

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        # self.desiredSpeed = 0
        # self.table.putNumber('desired speed', self.desiredSpeed)
        # self.DriveMotorP = 0
        # self.table.putNumber("P", self.FLMotorP)

        # self.driveFLClosedLoopController = self.driveMotorFL.getClosedLoopController()
        # self.driveFRClosedLoopController = self.driveMotorFR.getClosedLoopController()
        # self.driveBLClosedLoopController = self.driveMotorBL.getClosedLoopController()
        # self.driveBRClosedLoopController = self.driveMotorBR.getClosedLoopController()
    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        
        #self.driveMotorFL.setVoltage(buf.driveVolts*1.6)
        # self.table.putNumber('hal drive volts', buf.driveVolts)

        



        
        
        # self.MotorP = self.table.getNumber('p', self.FLMotorP)
        

        # driveUniversalConfig = rev.SparkBaseConfig()
        

        
        # driveUniversalConfig.closedLoop.pidf(self.MotorP, 0, 0, 1/473, rev.ClosedLoopSlot.kSlot0)
        # self.driveMotorFL.configure(
        #     driveUniversalConfig, 
        #     rev.SparkBase.ResetMode.kNoResetSafeParameters, 
        #     rev.SparkBase.PersistMode.kNoPersistParameters)
        
        # self.desiredSpeed = self.table.getNumber('desired speed', self.desiredSpeed)
        # self.driveFLClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        # self.driveFRClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        # self.driveBLClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        # self.driveBRClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        #self.FLController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)


        #self.table.putNumber("error", self.Error)
        # #self.driveMotorFL.


class revPID:

    def __init__(self, drivemotorIn)->None:
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.MotorP = 0
        self.driveMotor = drivemotorIn
        self.driveMotorName = str(drivemotorIn)

        self.driveUniversalConfig = rev.SparkBaseConfig()

                    
        drivemotorIn.configure(
            self.driveUniversalConfig,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters
        )
        # RobotHAL.driveMotorFL.configure(
        #     self.driveUniversalConfig, 
        #     rev.SparkBase.ResetMode.kNoResetSafeParameters, 
        #     rev.SparkBase.PersistMode.kNoPersistParameters)
        
        # RobotHAL.driveMotorFR.configure(
        #     self.driveUniversalConfig, 
        #     rev.SparkBase.ResetMode.kNoResetSafeParameters, 
        #     rev.SparkBase.PersistMode.kNoPersistParameters)
        
        # RobotHAL.driveMotorBL.configure(
        #     self.driveUniversalConfig, 
        #     rev.SparkBase.ResetMode.kNoResetSafeParameters, 
        #     rev.SparkBase.PersistMode.kNoPersistParameters)
        
        # RobotHAL.driveMotorBR.configure(
        #     self.driveUniversalConfig, 
        #     rev.SparkBase.ResetMode.kNoResetSafeParameters, 
        #     rev.SparkBase.PersistMode.kNoPersistParameters)

        self.desiredSpeed = 0
        self.table.putNumber('desired speed', self.desiredSpeed)
        self.DriveMotorP = 0
        self.table.putNumber("P", self.MotorP)

        self.driveClosedLoopController = self.driveMotor.getClosedLoopController()
        
        driveFLEncoder = self.driveMotorFL.getEncoder()
        driveFLPercentVoltage = self.driveMotorFL.getAppliedOutput()
        driveFLSpeedFeedback = driveFLEncoder.getVelocity()
        driveFLPosition = driveFLEncoder.getPosition()
        driveFLVoltage = self.driveMotorFL.getAppliedOutput()*self.driveMotorFL.getBusVoltage()
        self.table.putNumber(self.driveMotorName + "voltage", driveFLVoltage )
        self.table.putNumber(self.driveMotorName + "Speed feedback", driveFLSpeedFeedback)
        self.table.putNumber(self.driveMotorName + "Position", driveFLPosition)
        self.table.putNumber(self.driveMotorName + "percent voltage", driveFLPercentVoltage)

        



    def periodic(self)->None:
        pass

    def update (self)->None:
        self.DriveMotorP = self.table.getNumber('P', self.DriveMotorP)
        self.driveUniversalConfig.closedLoop.pid(self.MotorP, 0, 0, rev.ClosedLoopSlot.kSlot0)
        #self.driveUniversalConfig.closedLoop.pidf(self.MotorP, 0, 0, 1/473, rev.ClosedLoopSlot.kSlot0)

        self.desiredSpeed = self.table.getNumber('desired speed', self.desiredSpeed)
        self.driveClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        

        



