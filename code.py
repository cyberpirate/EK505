#!/usr/bin/python3

from vpython import *
import math
import numpy as np
import AStar
import CSpace
import Collision

ARM_RAD = 0.25
ARM_LENGTH = 4
GRAB_MIN = 0.25
GRAB_MAX = 1
GRAB_LEN = 1
GRAB_RAD = 0.1

# ARM1 = cylinder(length=ARM_LENGTH, radius=ARM_RAD, color=color.green)
# ARM2 = cylinder(length=ARM_LENGTH, radius=ARM_RAD, color=color.red)
# ARM1 = box(size=vec(3, 3, 3))
# ARM2 = box(size=vec(3, 3, 3))

ARM1 = cylinder(radius=ARM_RAD, color=color.green)
ARM2 = cylinder(radius=ARM_RAD, color=color.red)
ARM3 = cylinder(radius=ARM_RAD, color=color.blue)
GRAB_BAR1 = cylinder(radius=GRAB_RAD, color=color.white)
GRAB_BAR2 = cylinder(radius=GRAB_RAD, color=color.white)

OBS = [
    sphere(pos=vec(1, 1, 0), radius=GRAB_MIN, color=color.red),
    sphere(pos=vec(2, 5, -1), radius=1, color=color.red)
]

def followPath(path):

    for node in path:
        rate(30)

        state = CSpace.cspaceToState(node.T1I, node.T2I, node.T3I, node.DI)
        renderForwardKinematics(*state)

def originAxis(l=1):
    arrow(axis=(vec(l, 0, 0)), color=color.red)
    arrow(axis=(vec(0, l, 0)), color=color.green)
    arrow(axis=(vec(0, 0, l)), color=color.blue)

def to01(i):
    return (1-cos(i))/2

def scale(x, bot, top):
    return bot + (top - bot) * x

def checkNoCollision(T1I, T2I, T3I, DI):
    state = CSpace.cspaceToState(T1I, T2I, T3I, DI)

    worldState = forwardKinematics(*state)

    for i in range(0, len(worldState), 2):
        pos = worldState[i]
        fwd = worldState[i+1]

        for o in OBS:
            if Collision.sphereInter(pos, fwd, o.pos, o.radius):
                return False

    return True

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
    
    return (
        ARM1_pos, ARM1_fwd,
        ARM2_pos, ARM2_fwd,
        ARM3_pos, ARM3_fwd,
        GRAB_BAR1_pos, GRAB_BAR1_fwd,
        GRAB_BAR2_pos, GRAB_BAR2_fwd
    )

def renderForwardKinematics(T1, T2, T3, D):
    armState = forwardKinematics(T1, T2, T3, D)
        
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

def inverseKinematics(x, y, z):

    L1 = ARM_LENGTH
    L2 = ARM_LENGTH
    L3 = ARM_LENGTH

    T1 = math.atan(y / x)

    T2 = math.atan((z - L1) / math.sqrt(x**2 + y**2))

    T2_ = math.acos((L3**2 - L2**2 - x**2 - y**2 - (z - L1)**2) / (-2*L2*math.sqrt(x**2 + y**2 + (z - L1)**2)))

    T3 = math.pi - math.acos((x**2 + y**2 + (z - L1)**2 - L2**2 - L3**2) / (-2*L2*L3))

    return (
        (T1, T2 + T2_, -T3),
        (T1, T2 - T2_, T3)
    )

# originAxis(10)

# print(inverseKinematics(-ARM_LENGTH/2, ARM_LENGTH, 0))
# print(inverseKinematics((1 + math.sqrt(3)) / 2, 0, (3 + sqrt(3)) / 2))
# print(inverseKinematics(1, 1, 1))

renderForwardKinematics(0, 0, 0, 0)

startCspace = CSpace.stateToCspace(0, 0, 0, 0)
endCspace = CSpace.stateToCspace(math.pi, math.pi/2, -math.pi/2, 1)

startNode = CSpace.CSpaceNode(*startCspace, [checkNoCollision])
endNode = CSpace.CSpaceNode(*endCspace, [])

path = AStar.AStar(startNode, endNode)

i = 0

while True:

    followPath(path)

    # rate(60)

    # T1 = scale(to01(i/100), 0, 2*math.pi)
    # T2 = scale(to01(i/200), -math.pi/2, math.pi/2)
    # T3 = scale(to01(i/100), -math.pi/2, math.pi/2)
    # D  = to01(i/100)

    # armState = forwardKinematics(0, 0, 0, D)
        
    # ARM1.pos  = armState[0]
    # ARM1.axis = armState[1]

    # ARM2.pos  = armState[2]
    # ARM2.axis = armState[3]

    # ARM3.pos  = armState[4]
    # ARM3.axis = armState[5]

    # GRAB_BAR1.pos = armState[6]
    # GRAB_BAR1.axis = armState[7]

    # GRAB_BAR2.pos = armState[8]
    # GRAB_BAR2.axis = armState[9]

    # # ARM1.up = vec(-1, 0, 0)
    # i += 1