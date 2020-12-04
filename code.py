#!/usr/bin/python3

from vpython import *
import math

ARM_RAD = 1
ARM_LENGTH = 4
GRAB_MIN = 0.25
GRAB_MAX = 1
GRAB_LEN = 1

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
GRAB_BAR1 = arrow(color=color.white)
GRAB_BAR2 = arrow(color=color.white)

# testArrow = arrow(color=color.red)

def originAxis(l=1):
    arrow(axis=(vec(l, 0, 0)), color=color.red)
    arrow(axis=(vec(0, l, 0)), color=color.green)
    arrow(axis=(vec(0, 0, l)), color=color.blue)

def to01(i):
    return (1-cos(i))/2

def scale(x, bot, top):
    return bot + (top - bot) * x

def forwardKinematics(T1, T2, T3, D):

    ARM1_pos = vec(0, 0, 0)
    ARM1_fwd = vec(0, ARM_LENGTH, 0)

    ARM2_pos = ARM1_pos + ARM1_fwd
    ARM2_fwd = vec(ARM_LENGTH, 0, 0)

    ARM2_fwd = ARM2_fwd.rotate(T2, axis=vec(0, 0, 1))
    ARM2_fwd = ARM2_fwd.rotate(T1, axis=vec(0, 1, 0))

    ARM3_pos = ARM2_pos + ARM2_fwd
    ARM3_fwd = ARM2_fwd.rotate(T3, ARM2_fwd.cross(vec(0, 1, 0)))

    armbar = ARM3_fwd.cross(vec(0, 1, 0)).norm() * scale(D, GRAB_MIN, GRAB_MAX)

    GRAB_BAR1_pos = ARM3_pos + ARM3_fwd + armbar
    GRAB_BAR1_fwd = ARM3_fwd.norm() * GRAB_LEN

    GRAB_BAR2_pos = ARM3_pos + ARM3_fwd - armbar
    GRAB_BAR2_fwd = ARM3_fwd.norm() * GRAB_LEN
    
    return ARM1_pos, ARM1_fwd, ARM2_pos, ARM2_fwd, ARM3_pos, ARM3_fwd, GRAB_BAR1_pos, GRAB_BAR1_fwd, GRAB_BAR2_pos, GRAB_BAR2_fwd

# originAxis(10)

i = 0

while True:
    rate(60)

    T1 = scale(to01(i/100), 0, 2*math.pi)
    T2 = scale(to01(i/200), -math.pi/2, math.pi/2)
    T3 = scale(to01(i/100), -math.pi/2, math.pi/2)
    D  = to01(i/100)

    armState = forwardKinematics(0, 0, 0, D)
        
    ARM1.pos  = armState[0]
    ARM1.axis = armState[1]

    ARM2.pos  = armState[2]
    ARM2.axis = armState[3]

    ARM3.pos  = armState[4]
    ARM3.axis = armState[5]

    GRAB_BAR1.pos = armState[6]
    GRAB_BAR1.axis = armState[7]

    GRAB_BAR2.pos = armState[8]
    GRAB_BAR2.axis = armState[9]

    # ARM1.up = vec(-1, 0, 0)
    i += 1