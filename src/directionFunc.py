import math
pi = 1 #math.pi
def func(b, a):
    print("current motor pos: " +str(b))
    print("current wheel pos: " + str(b/25))
    print("wheel target pos on joystick: " + str(a))
    c = b%(2*pi)
    print("wheel pos after mod 2pi: " + str(c))
    if c> pi:
        c -= 2*pi
    elif c<-pi:
        c += 2*pi
    print("wheel pos unwrapped: " + str(c))
    delta = a-c
    print("distance between motor pos and target pos: " + str(delta))
    if delta> pi:
        delta -= 2*pi
    elif delta<-pi:
        delta += 2*pi
    print("distance between motor pos and target pos (closest distance): " + str(delta))

    print("new wheel position: " + str(b + delta))
    print("full wheel rotation: " + str(delta))
    print("full motor: " + str(delta*25))
    return (b+delta)*25
    

listB = [99.25*pi, -99.25*pi, 0.25*pi, -0.25*pi, 0.75*pi, -0.75*25]
listA = [-0.5*pi, 0.5*pi, -0.75*pi, 0.75*pi]

for i in range(len(listB)*len(listA)):
    a = i%4
    y = i%6
    print("motor new position: " + str(func(listB[y], listA[a])))
    print("finis")

    