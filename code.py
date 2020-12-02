#!/usr/bin/python3

from vpython import *
import math

ARM_RAD = 1
ARM_LENGTH = 4

T1 = 0
T2 = 0
T3 = 0
D = 0

ARM1 = cylinder(length=ARM_LENGTH, radius=ARM_RAD, color=color.green)
# ARM1 = box(size=vec(3, 3, 3))

def originAxis(l=1):
    arrow(axis=(vec(l, 0, 0)), color=color.red)
    arrow(axis=(vec(0, l, 0)), color=color.green)
    arrow(axis=(vec(0, 0, l)), color=color.blue)

def forwardKinematics(T1, T2, T3, D):

    ARM1_pos = vec(0, 0, 0)
    ARM1_up = vec(0, 1, 0)




    return ARM1_pos, ARM1_up

originAxis(10)



i = 0

while True:
    rate(200)

        
    ARM1.pos, ARM1.up = forwardKinematics(0, 0, 0, 0)


    
    ARM1.up = ARM1.up.rotate(math.pi/2, axis=vec(0, 0, 1))
        

    # ARM1.up = vec(-1, 0, 0)
    i += 1