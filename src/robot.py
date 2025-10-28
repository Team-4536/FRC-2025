import wpilib
import rev
import djoAuto
from rev import SparkMax
from ntcore import NetworkTableInstance, NetworkTable


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.driveCtrlr = wpilib.XboxController(0)

        self.Motor = SparkMax(4, rev.SparkMax.MotorType.kBrushless)
        self.Motor.set(0)

        self.myOneAutoStage = djoAuto.myCustomAS(5)
        self.myTwoAutoStage = djoAuto.myCustomAS(10)

        auto1_start = djoAuto.myCustomAS(10)
        auto1_shoot = djoAuto.myCustomAS(5)
        auto1_moveBack = djoAuto.myCustomAS(2)
        auto1_allStop = djoAuto.allStopAS()
        auto1_start.setNext(auto1_shoot)
        auto1_shoot.setNext(auto1_moveBack)
        auto1_moveBack.setNext(auto1_allStop)
        self.curAutoStage = auto1_start
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def robotPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self.curAutoStage.autoInit(self)
        self.table.putNumber("test", 2)
        pass

    def autonomousPeriodic(self):
        if self.curAutoStage.isDone(self) == False:
            self.curAutoStage.run(self)
        else:
            self.curAutoStage = self.curAutoStage.getNext(self)
            self.table.putString("NextStage", self.curAutoStage.getNext(self).name)

            self.curAutoStage.autoInit(self)
        pass

    def teleopInit(self) -> None:

        self.on = False

    def teleopPeriodic(self):

        pressedA = self.driveCtrlr.getAButtonPressed()

        if pressedA:
            self.on = not self.on

        if self.on:
            self.Motor.set(0.2)

        else:
            self.Motor.set(0)


if __name__ == "__main__":
    wpilib.run(Robot)

    # self.autoDict = {
    #     "stage2": djoAuto.myCustomAS(10, "stage3"),
    #     "stage1": djoAuto.myCustomAS(5, "stage2"),
    #     "stage3": self.myOneAutoStage,
    #     # "stageN": djoAuto.MoveToShoot(
    #     #     "../paths/mypath.json", "stageIDforInPos", "stageIDforOutPos"
    #     # ),
    # }

    # self.autos = {"three piece", forward}
    # self.ThreePiece = forward
    # self.curAutoStage = self.autoDict["stage1"]

    # # self.autoDictionary["stage3"] = djoAuto.myCustomAS(2)
