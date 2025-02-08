from robotHAL import RobotHALBuffer


class intakeSubsystem:
    def __init__(self) -> None:

        pass
        # take ring in with one button stop then spit back out with another button

    def update(self, halBuffer: RobotHALBuffer, YButton: bool, XButton: bool):
        if not halBuffer.limitSwitchValue and XButton:
            topWheels = 1
            alsoTopWheels = 1
        # motors 11 & 12 maybe 13
        elif halBuffer.limitSwitchValue:
            topWheels = 0
            alsoTopWheels = 0
        elif halBuffer.limitSwitchValue and YButton:
            topWheels = -1
            alsoTopWheels = -1
            bottomWheels = -1
        else:
            topWheels = 0
            alsoTopWheels = 0
        pass
