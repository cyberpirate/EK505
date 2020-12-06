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
    sphere(pos=vec(0, 0, 0), radius=GRAB_MIN, color=color.red),
    sphere(pos=vec(0, 0, 0), radius=GRAB_MIN, color=color.red),
    sphere(pos=vec(0, 0, 0), radius=GRAB_MIN, color=color.red),
    sphere(pos=vec(0, 0, 0), radius=GRAB_MIN, color=color.red),
    sphere(pos=vec(0, 0, 0), radius=GRAB_MIN, color=color.red)
]

attachedObs = None

def initOBS():
    OBS[0].pos = vec(2, 0, -2)
    OBS[1].pos = vec(2, 0, -1)
    OBS[2].pos = vec(3, 0, -2)
    OBS[3].pos = vec(3, 0, -1)
    OBS[4].pos = vec(2.5, 0, -1.5)

def followPath(path):

    for node in path:
        rate(30)

        renderForwardKinematics(*node.getState())

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

def checkStillClosed(T1I, T2I, T3I, DI):
    return DI == 0

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

    # print(ARM3.pos + GRAB_BAR1.axis)

    if attachedObs is not None:
        attachedObs.pos = ARM3.pos + ARM3.axis
        attachedObs.pos += ARM3.axis.norm() * (GRAB_LEN - attachedObs.radius)

def link2ForwardKinematics(T1, T2, L1, L2):
    x1 = L1*math.cos(T1)
    y1 = L1*math.sin(T1)

    origin = vec(0, 0, 0)
    mid = vec(x1, y1, 0)
    end = vec(x1 + L2*math.cos(T1 + T2), y1 + L2*math.sin(T1 + T2), 0)

    return (
        origin, mid,
        mid, end - mid,
        end
    )

def link2CheckInv(T1, T2, L1, L2, pos):
    ans = link2ForwardKinematics(T1, T2, L1, L2)[4]
    return (ans - pos).mag < 0.01

def link2InverseKinematics(x, y, L1, L2):
    cT2 = (x**2 + y**2 - L1**2 - L2**2) / (2*L1*L2)
    sT2_pos = math.sqrt(1 - cT2**2)
    sT2_neg = -sT2_pos

    T2_pos = math.atan2(sT2_pos, cT2)
    T2_neg = math.atan2(sT2_neg, cT2)

    cT1_pos = (x*(L1 + L2*cT2) + y*L2*sT2_pos) / (x**2 + y**2)
    cT1_neg = (x*(L1 + L2*cT2) + y*L2*sT2_neg) / (x**2 + y**2)

    sT1_pos = math.sqrt(1 - cT1_pos**2)
    sT1_neg = -sT1_pos

    T1_pos = math.atan2(sT1_pos, cT1_pos)
    T1_neg = math.atan2(sT1_neg, cT1_pos)

    sT1_pos2 = math.sqrt(1 - cT1_neg**2)
    sT1_neg2 = -sT1_pos2

    T1_pos2 = math.atan2(sT1_pos2, cT1_neg)
    T1_neg2 = math.atan2(sT1_neg2, cT1_neg)

    ret = [
        (T1_neg, T2_pos),   # color.red
        (T1_pos, T2_pos),   # color.green
        (T1_neg2, T2_neg),  # color.white
        (T1_pos2, T2_neg)   # color.yellow
    ]

    ret = [ r for r in ret if link2CheckInv(*r, L1, L2, vec(x, y, 0))]

    return ret

def inverseKinematics(x, y, z):

    L1 = ARM_LENGTH
    L2 = ARM_LENGTH
    L3 = ARM_LENGTH

    pp = vec(x, 0, z)

    T1 = math.acos(pp.dot(vec(1, 0, 0)) / pp.mag)

    if z > 0:
        T1 = math.pi + (math.pi - T1)

    xp = pp.mag
    yp = y - L1

    link2State = link2InverseKinematics(xp, yp, L2, L3)

    return [
        (T1, *link2State[0]),
        (T1, *link2State[1])
    ]

# def inverseKinematics(x, y, z):

#     L1 = ARM_LENGTH
#     L2 = ARM_LENGTH
#     L3 = ARM_LENGTH

#     # T1 = math.atan(y / x)

#     # T2 = math.atan((z - L1) / math.sqrt(x**2 + y**2))

#     # T2_ = math.acos((L3**2 - L2**2 - x**2 - y**2 - (z - L1)**2) / (-2*L2*math.sqrt(x**2 + y**2 + (z - L1)**2)))

#     # T3 = math.pi - math.acos((x**2 + y**2 + (z - L1)**2 - L2**2 - L3**2) / (-2*L2*L3))

#     # ret = [
#     #     (T1, T2 + T2_, -T3),
#     #     (T1, T2 - T2_, T3)
#     # ]

#     # ret = [ r for r in ret if CSpace.notOutOfBounds(*CSpace.stateToCspace(*r, 0))]

#     # if len(ret) == 0:
#     #     raise RuntimeError("No inverse kinematics found")

#     # return ret

#     # T3 = math.acos((x**2 + y**2 + (z-L1)**2 - L2**2 - L3**2) / (2*L2*L3))

#     # a = L3*math.sin(T3)
#     # b = L2 + L3*math.cos(T3)
#     # c = z - L1
#     # r = math.sqrt(a**2 + b**2)

#     # T2 = math.atan(c / math.sqrt(r**2 - c**2)) - math.atan(a/b)
#     # T1 = math.atan(y/x)

#     # return [(T1, T2, T3)]

#     pp = vec(x, 0, z)
#     T1 = math.acos(pp.dot(vec(1, 0, 0)) / pp.mag)

#     planeNorm = pp.cross(vec(0, 1, 0)).norm()

#     v = goal.pos - vec(0, ARM_LENGTH, 0)
#     w = planeNorm
#     vt = v - (v.dot(w)/w.mag**2)*w
#     goalp = vec(0, ARM_LENGTH, 0) + vt

#     xp = vec(goalp.x, 0, goalp.z).mag
#     yp = goalp.y

#     cT3 = (xp**2 + yp**2 - L2**2 - L3**2) / (2*L2*L3)
#     sT3_pos = math.sqrt(1 - cT3**2)
#     sT3_neg = -sT3_pos

#     T3_pos = math.atan2(sT3_pos, cT3)
#     T3_neg = math.atan2(sT3_neg, cT3)

#     cT2_pos = (xp*(L2 + L2*cT3) + yp*L3*sT3_pos) / (xp**2 + yp**2)
#     cT2_neg = (xp*(L2 + L2*cT3) + yp*L3*sT3_neg) / (xp**2 + yp**2)

#     return [(0, 0, 0)]

def travelAndOpen(startState, endXYZ):
    goal = inverseKinematics(*endXYZ)

    startNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*startState), [checkNoCollision])
    endNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*goal[0], 1), [checkNoCollision])

    return AStar.AStar(startNode, endNode)

def closeHere(startState):

    goal = (startState[0], startState[1], startState[2], 0)

    startNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*startState), [])
    endNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*goal), [])

    return AStar.AStar(startNode, endNode)

def travelKeepClosed(startState, endXYZ):
    goal = inverseKinematics(*endXYZ)

    startNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*startState), [checkNoCollision, checkStillClosed])
    endNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*goal[0], 0), [checkNoCollision, checkStillClosed])

    return AStar.AStar(startNode, endNode)

def openHere(startState):

    goal = (startState[0], startState[1], startState[2], 1)

    startNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*startState), [])
    endNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*goal), [])

    return AStar.AStar(startNode, endNode)

def travel(startState, endState):

    startNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*startState), [checkNoCollision])
    endNode = CSpace.CSpaceNode(*CSpace.stateToCspace(*endState), [checkNoCollision])

    return AStar.AStar(startNode, endNode)

# while True:
#     initOBS()
#     startState = (0, 0, 0, 0)
#     robotState = startState
#     renderForwardKinematics(*robotState)

#     for i in range(len(OBS)):

#         print("Moving {}".format(i))

#         # Travel to the object to pick up
#         path = travelAndOpen(robotState, (OBS[i].pos.x, OBS[i].pos.y, OBS[i].pos.z))
#         followPath(path)
#         robotState = path[-1].getState()

#         # Close around the object
#         path = closeHere(robotState)
#         followPath(path)
#         robotState = path[-1].getState()

#         # Set the object as "grabbed"
#         attachedObs = OBS[i]

#         # Drag object to where we place it
#         placePos = vec(2 - i*GRAB_MAX*1.5, ARM_LENGTH, -2)
#         path = travelKeepClosed(robotState, (placePos.x, placePos.y, placePos.z))
#         followPath(path)
#         robotState = path[-1].getState()

#         # Open end effector
#         path = openHere(robotState)
#         followPath(path)
#         robotState = path[-1].getState()

#         # Drop object
#         attachedObs = None

#     # back to start state
#     path = travel(robotState, startState)
#     followPath(path)
#     robotState = path[-1].getState()

renderForwardKinematics(0, 0, 0, 0)

goal = OBS[0]
goal.pos = vec(4, 5, -4)

state = inverseKinematics(goal.pos.x, goal.pos.y, goal.pos.z)
print(state)

i = 0
while True:

    rate(1)

    renderForwardKinematics(*state[i % len(state)], 0)

    i += 1

# i = 0

# orbit = OBS[0]
# goal = OBS[1]
# goalp = OBS[2]
# norm = OBS[3]

# goal.pos = vec(4, 5, -4)

# arr = arrow()

# while True:
#     i += 1

#     rate(30)
    

#     orbit.pos = vec(ARM_LENGTH, 0, 0).rotate(i/100, axis=vec(0, 1, 0))

#     pp = vec(orbit.pos.x, 0, orbit.pos.z)

#     T1 = math.acos(pp.dot(vec(1, 0, 0)) / pp.mag)

#     if orbit.pos.z > 0:
#         T1 = math.pi + (math.pi - T1)

#     renderForwardKinematics(T1, 0, 0, 0)
    

#     planeNorm = pp.cross(vec(0, 1, 0))
#     norm.pos = planeNorm.norm()

#     v = goal.pos - vec(0, ARM_LENGTH, 0)
#     w = planeNorm
#     vt = v - (v.dot(w)/w.mag**2)*w
#     goalp.pos = vec(0, ARM_LENGTH, 0) + vt
#     arr.pos = goalp.pos
#     arr.axis = goal.pos - goalp.pos
