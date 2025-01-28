import copy
import math
import robot
import revPID

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

        self.maxVelocity = 2000
        self.maxAcceleration = 4000
        self.closedLoopError = 1

        self.driveMotorUniversalP = 0.00019
        self.table.putNumber("DriveMotorP", self.driveMotorUniversalP)
        self.driveMotorUniversalDesiredSpeed = 0
        self.table.putNumber("DriveMotorSetPoint", self.driveMotorUniversalDesiredSpeed)
        self.driveMotorUniversalFF = 0.00002
        self.table.putNumber("driveMotorFF", self.driveMotorUniversalFF)

        self.driveMotorUniversalConfig = rev.SparkBaseConfig()
        self.driveMotorUniversalConfig.closedLoop.pidf(
            self.driveMotorUniversalP, 0, 0, 1/473, rev.ClosedLoopSlot.kSlot0
            ).setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).outputRange(
            -1.0, 1.0, rev.ClosedLoopSlot.kSlot0
        )
        
        self.driveMotorUniversalConfig.closedLoop.maxMotion.maxVelocity(
            self.maxVelocity, rev.ClosedLoopSlot.kSlot0
        ).maxAcceleration(
            self.maxAcceleration, rev.ClosedLoopSlot.kSlot0
        ).allowedClosedLoopError(
            self.closedLoopError
        )

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        pass

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        if (
             abs(self.table.getNumber("DriveMotorP", self.driveMotorUniversalP) - self.driveMotorUniversalP) < 1e-6
             or abs(self.table.getNumber("driveMotorFF", self.driveMotorUniversalFF) - self.driveMotorUniversalFF) < 1e-6
        ):
            self.driveMotorUniversalP = self.table.getNumber("DriveMotorP", self.driveMotorUniversalP)
            self.driveMotorUniversalDesiredSpeed = self.table.getNumber("DriveMotor UniversalDesiredSpeed", self.driveMotorUniversalDesiredSpeed)
            
        
    


            
        




