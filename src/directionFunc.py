import math

# targetPos, motorPos, distance, and unWrappedPos are subject to name change

pi = 1 #math.pi #pi is 1 for testing radians
gearRatio = 25 # Change to actual gear ratio
def func(motorPos, targetPos):

    print("current motor pos: " +str(motorPos))
    print("current wheel pos: " + str(motorPos/gearRatio))
    print("wheel target pos on joystick: " + str(targetPos))

    unWrappedPos = motorPos%(2*pi)
    
    print("motor pos after mod 2pi: " + str(unWrappedPos))
    
    if unWrappedPos> pi:
        unWrappedPos -= 2*pi
    elif unWrappedPos<-pi:
        unWrappedPos += 2*pi
    
    print("motor pos unwrapped: " + str(unWrappedPos))
    
    distance = targetPos-unWrappedPos
    
    print("distance between motor pos and target pos: " + str(distance))
    
    if distance> pi:
        distance -= 2*pi
    elif distance<-pi:
        distance += 2*pi
    
    print("distance between motor pos and target pos (closest distance): " + str(distance))

    print("new wheel position: " + str(motorPos/gearRatio + distance))
    print("full wheel rotation: " + str(distance))
    print("full motor: " + str(distance*gearRatio))
    
    return (motorPos+(distance*gearRatio))
    

#For testing:

#Test variables
# listB = [99.25*pi, -99.25*pi, 0.25*pi, -0.25*pi, 0.75*pi, -0.75*25]
# listA = [-0.5*pi, 0.5*pi, -0.75*pi, 0.75*pi]

# for i in range(len(listB)*len(listA)):
#     targetPos = i%len(listA)
#     y = i%len(listB)
#     print("motor new position: " + str(func(listB[y], listA[targetPos])))
#     print("finis")

    