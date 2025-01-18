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

        self.desiredSpeed = 0
        self.table.putNumber('desired speed', self.desiredSpeed)
        self.DriveMotorP = 0
        self.table.putNumber("P", self.FLMotorP)

        self.driveFLClosedLoopController = self.driveMotorFL.getClosedLoopController()
        self.driveFRClosedLoopController = self.driveMotorFR.getClosedLoopController()
        self.driveBLClosedLoopController = self.driveMotorBL.getClosedLoopController()
        self.driveBRClosedLoopController = self.driveMotorBR.getClosedLoopController()
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

        driveFLEncoder = self.driveMotorFL.getEncoder()
        driveFLPercentVoltage = self.driveMotorFL.getAppliedOutput()
        driveFLSpeedFeedback = driveFLEncoder.getVelocity()
        driveFLPosition = driveFLEncoder.getPosition()
        driveFLVoltage = self.driveMotorFL.getAppliedOutput()*self.driveMotorFL.getBusVoltage()
        self.table.putNumber("DriveMotorFL voltage", driveFLVoltage )
        self.table.putNumber("DriveMotorFL Speed feedback", driveFLSpeedFeedback)
        self.table.putNumber("DriveMotorFL Position", driveFLPosition)
        self.table.putNumber("DriveMotorFL percent voltage", driveFLPercentVoltage)

        driveFREncoder = self.driveMotorFR.getEncoder()
        driveFRPercentVoltage = self.driveMotorFR.getAppliedOutput()
        driveFRSpeedFeedback = driveFREncoder.getVelocity()
        driveFRPosition = driveFREncoder.getPosition()
        driveFRVoltage = self.driveMotorFR.getAppliedOutput()*self.driveMotorFR.getBusVoltage()
        self.table.putNumber("DriveMotorFR voltage", driveFRVoltage )
        self.table.putNumber("DriveMotorFR Speed feedback", driveFRSpeedFeedback)
        self.table.putNumber("DriveMotorFR Position", driveFRPosition)
        self.table.putNumber("DriveMotorFR percent voltage", driveFRPercentVoltage)

        driveBLEncoder = self.driveMotorBL.getEncoder()
        driveBLPercentVoltage = self.driveMotorBL.getAppliedOutput()
        driveBLSpeedFeedback = driveBLEncoder.getVelocity()
        driveBLPosition = driveBLEncoder.getPosition()
        driveBLVoltage = self.driveMotorBL.getAppliedOutput()*self.driveMotorBL.getBusVoltage()
        self.table.putNumber("DriveMotorBL voltage", driveBLVoltage )
        self.table.putNumber("DriveMotorBL Speed feedback", driveBLSpeedFeedback)
        self.table.putNumber("DriveMotorBL Position", driveBLPosition)
        self.table.putNumber("DriveMotorBL percent voltage", driveBLPercentVoltage)

        driveBREncoder = self.driveMotorBR.getEncoder()
        driveBRPercentVoltage = self.driveMotorBR.getAppliedOutput()
        driveBRSpeedFeedback = driveBREncoder.getVelocity()
        driveBRPosition = driveBREncoder.getPosition()
        driveBRVoltage = self.driveMotorBR.getAppliedOutput()*self.driveMotorBR.getBusVoltage()
        self.table.putNumber("DriveMotorBR voltage", driveBRVoltage )
        self.table.putNumber("DriveMotorBR Speed feedback", driveBRSpeedFeedback)
        self.table.putNumber("DriveMotorBR Position", driveBRPosition)
        self.table.putNumber("DriveMotorBR percent voltage", driveBRPercentVoltage)



        # FL_Encoder = self.driveMotorFL.getEncoder()
        # FL_PercentVoltage = self.driveMotorFL.getAppliedOutput()
        # driveFLSpeedFeedback = FL_Encoder.getVelocity()
        # FL_Position = FL_Encoder.getPosition()
        # FL_Voltage = self.driveMotorFL.getAppliedOutput()*self.driveMotorFL.getBusVoltage()
        # self.table.putNumber("DriveMotorFL voltage", FL_Voltage )
        # self.table.putNumber("DriveMotorFL Speed feedback", driveFLSpeedFeedback)
        # self.table.putNumber("DriveMotorFL Position", FL_Position)
        # self.table.putNumber("DriveMotor percent voltage", FL_PercentVoltage)
        
        
        self.MotorP = self.table.getNumber('p', self.FLMotorP)
        

        driveUniversalConfig = rev.SparkBaseConfig()
        # #myConfig.closedLoop.pid(.0001, 0, 0, rev.ClosedLoopSlot.kSlot0)

        #myConfig.closedLoop.P(self.FLMotorP, rev.ClosedLoopSlot.kSlot0)
        #myConfig.closedLoop.velocityFF(1/473, rev.ClosedLoopSlot.kSlot0)
        driveUniversalConfig.closedLoop.pidf(self.MotorP, 0, 0, 1/473, rev.ClosedLoopSlot.kSlot0)
        self.driveMotorFL.configure(
            driveUniversalConfig, 
            rev.SparkBase.ResetMode.kNoResetSafeParameters, 
            rev.SparkBase.PersistMode.kNoPersistParameters)
        
        self.desiredSpeed = self.table.getNumber('desired speed', self.desiredSpeed)
        self.driveFLClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        self.driveFRClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        self.driveBLClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        self.driveBRClosedLoopController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        #self.FLController.setReference(self.desiredSpeed, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)


        self.table.putNumber("error", self.Error)
        # #self.driveMotorFL.


    class PIDs:

        def __init__(self)->None:
            self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        

        def __periodic_(self)->None:
            pass


        def update (self)->None:
            pass



