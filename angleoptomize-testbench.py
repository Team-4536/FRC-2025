import math
import numpy as np

def arcsinList(desY, unboundrotations)->list:
        
        unboundAngle = (unboundrotations * (2*math.pi))/25

        referenceValue = math.asin(desY)
        wrongList: list[float] = [referenceValue]

        if referenceValue > 0:
            
            referenceValue2 = ((((math.pi)/25)/2)+(((math.pi/25)/2) - referenceValue))
            wrongList.append(referenceValue2)
            wrongList.append((referenceValue+((2*math.pi)/25)))
            wrongList.append((referenceValue2+((2*math.pi)/25)))

            wrongList.append((referenceValue - ((2*math.pi)/25)))
            wrongList.append((referenceValue2 - ((2*math.pi)/25)))
            wrongList.append((referenceValue - ((4*math.pi)/25)))
            wrongList.append((referenceValue2 - ((4*math.pi)/25)))

        elif referenceValue < 0:

            referenceValue2 = (-((math.pi/25)/2)-(((math.pi/25)/2) - referenceValue))
            wrongList.append(referenceValue2)
            wrongList.append((referenceValue-((2*math.pi)/25)))
            wrongList.append((referenceValue2-((2*math.pi)/25)))

            wrongList.append((referenceValue + ((2*math.pi)/25)))
            wrongList.append((referenceValue2 + ((2*math.pi)/25)))
            wrongList.append((referenceValue + ((4*math.pi)/25)))
            wrongList.append((referenceValue2 + ((4*math.pi)/25)))
        else:

            wrongList.append(((2*math.pi)/25))
            wrongList.append(((-(2*math.pi))/25))
            wrongList.append(((4*math.pi)/25))
            wrongList.append(((-(4*math.pi))/25))

        print(wrongList)

        subtraction = unboundAngle % ((2*math.pi)/25)
        period = (unboundAngle - subtraction)/((2*math.pi)/25)

        #wrongList[i]  + (period*((2*math.pi)/25)) for i in wrongList
        goalList: list[float] = []
        print(goalList)

        for i in wrongList:

            goalList.append((i) + (period*((2*math.pi)/25)))
        
        print(goalList)
        x = 0
        for i in goalList:
            
            if not (unboundAngle - (math.pi/25)) < (i) < (unboundAngle + (math.pi/25)):
                del goalList[x]
            x += 1

        print(goalList)

        return goalList

def optimizeTarget(
    desiredRelativeRotations, unboundRotations
):
    # -> SwerveModuleState
    unboundAngle = unboundRotations*(2*math.pi)
    
    desiredRelativeRadians = desiredRelativeRotations*((2*math.pi))
    desY = math.sin(25*(desiredRelativeRadians))
    goal: list[float] = arcsinList(desY, unboundAngle)

    print(goal)
    for i in goal:

        print(i)   

        if i > 2*math.pi:
            remainderRadians = i % ((2*math.pi)/25)
            if (desiredRelativeRadians - 0.01) < remainderRadians < (desiredRelativeRadians + 0.01):
                target = (str(i/(2*math.pi))+'Hu')

        elif i < -2*math.pi:
            remainderRadians = i % ((2*math.pi)/25)
    
            if (desiredRelativeRadians - 1) < remainderRadians*15 < (desiredRelativeRadians + 1):
                target = (str(i/(2*math.pi))+'hu')

            else:
                 target = remainderRadians
                 print(str(remainderRadians)+'hi')
                 print(desiredRelativeRadians)
                 print(desiredRelativeRadians/remainderRadians)

        elif -2*math.pi < i < 2*math.pi:
            if (-1*(desiredRelativeRadians) - 0.1) < i < (desiredRelativeRadians + 0.1):
                target = (i/(2*math.pi))

        else:
             target = "sad"
        
    return target



print(optimizeTarget(0.3, -10))

