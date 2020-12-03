#!/usr/bin/python3

from vpython import *
import math

ARM_RAD = 1
ARM_LENGTH = 4

T1 = 0
T2 = 0
T3 = 0
D = 0

# ARM1 = cylinder(length=ARM_LENGTH, radius=ARM_RAD, color=color.green)
# ARM2 = cylinder(length=ARM_LENGTH, radius=ARM_RAD, color=color.red)
# ARM1 = box(size=vec(3, 3, 3))
# ARM2 = box(size=vec(3, 3, 3))

ARM1 = arrow(color=color.green)
ARM2 = arrow(color=color.red)
ARM3 = arrow(color=color.blue)

# testArrow = arrow(color=color.red)

def originAxis(l=1):
    arrow(axis=(vec(l, 0, 0)), color=color.red)
    arrow(axis=(vec(0, l, 0)), color=color.green)
    arrow(axis=(vec(0, 0, l)), color=color.blue)

def forwardKinematics(T1, T2, T3, D):

    ARM1_pos = vec(0, 0, 0)
    ARM1_fwd = vec(0, ARM_LENGTH, 0)

    ARM2_pos = ARM1_pos + ARM1_fwd
    ARM2_fwd = vec(ARM_LENGTH, 0, 0)

    ARM2_fwd = ARM2_fwd.rotate(T2, axis=vec(0, 0, 1))
    ARM2_fwd = ARM2_fwd.rotate(T1, axis=vec(0, 1, 0))

    ARM3_pos = ARM2_pos + ARM2_fwd
    ARM3_fwd = vec(ARM_LENGTH, 0, 0)

    return ARM1_pos, ARM1_fwd, ARM2_pos, ARM2_fwd, ARM3_pos, ARM3_fwd

# originAxis(10)

def to01(i):
    return (1-cos(i))/2

def scale(x, bot, top):
    return bot + (top - bot) * x

i = 0

while True:
    rate(60)

    x = to01(i/100)

    T1 = scale(x, 0, 2*math.pi)
    T2 = scale(x, -math.pi/2, math.pi/2)

    armState = forwardKinematics(T1, T2, 0, 0)
        
    ARM1.pos  = armState[0]
    ARM1.axis = armState[1]

    ARM2.pos  = armState[2]
    ARM2.axis = armState[3]

    ARM3.pos  = armState[4]
    ARM3.axis = armState[5]

    # ARM1.up = vec(-1, 0, 0)
    i += 1