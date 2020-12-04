#!/usr/bin/python3

from vpython import *
import math
import numpy as np
import AStar

cspaceLim = (
    (0, 2*math.pi),
    (-math.pi/2, math.pi/2),
    (-math.pi/2, math.pi/2),
    (0, 1)
)

cspaceSegs = (
    200,
    100,
    100,
    100
)

cspace = np.ones(cspaceSegs, np.bool)

ARM_RAD = 1
ARM_LENGTH = 4
GRAB_MIN = 0.25
GRAB_MAX = 1
GRAB_LEN = 1

# ARM1 = cylinder(length=ARM_LENGTH, radius=ARM_RAD, color=color.green)
# ARM2 = cylinder(length=ARM_LENGTH, radius=ARM_RAD, color=color.red)
# ARM1 = box(size=vec(3, 3, 3))
# ARM2 = box(size=vec(3, 3, 3))

ARM1 = arrow(color=color.green)
ARM2 = arrow(color=color.red)
ARM3 = arrow(color=color.blue)
GRAB_BAR1 = arrow(color=color.white)
GRAB_BAR2 = arrow(color=color.white)

def cspaceToState(T1I, T2I, T3I, DI):

    T1_step = (cspaceLim[0][1] - cspaceLim[0][0]) / cspaceSegs[0]
    T2_step = (cspaceLim[1][1] - cspaceLim[1][0]) / cspaceSegs[1]
    T3_step = (cspaceLim[2][1] - cspaceLim[2][0]) / cspaceSegs[2]
    D_step  = (cspaceLim[3][1] - cspaceLim[3][0]) / cspaceSegs[3]

    return (
        cspaceLim[0][0] + T1_step*T1I,
        cspaceLim[1][0] + T2_step*T2I,
        cspaceLim[2][0] + T3_step*T3I,
        cspaceLim[3][0] + D_step*DI
    )

def stateToCspace(T1, T2, T3, D):

    T1_step = (cspaceLim[0][1] - cspaceLim[0][0]) / cspaceSegs[0]
    T2_step = (cspaceLim[1][1] - cspaceLim[1][0]) / cspaceSegs[1]
    T3_step = (cspaceLim[2][1] - cspaceLim[2][0]) / cspaceSegs[2]
    D_step  = (cspaceLim[3][1] - cspaceLim[3][0]) / cspaceSegs[3]

    return (
        math.floor((T1 - cspaceLim[0][0]) / T1_step),
        math.floor((T2 - cspaceLim[1][0]) / T2_step),
        math.floor((T3 - cspaceLim[2][0]) / T3_step),
        math.floor((D  - cspaceLim[3][0]) / D_step)
    )

class CSpaceNode(AStar.AStarNode):

    def __init__(self, T1I, T2I, T3I, DI, nodeFilters):
        self.T1I = T1I
        self.T2I = T2I
        self.T3I = T3I
        self.DI = DI
        self.nodeFilters = nodeFilters

    def __eq__(self, other):

        if not hasattr(other, "T1I") or self.T1I != other.T1I:
            return False

        if not hasattr(other, "T2I") or self.T2I != other.T2I:
            return False

        if not hasattr(other, "T3I") or self.T3I != other.T3I:
            return False

        if not hasattr(other, "DI") or self.DI != other.DI:
            return False

        return True
    
    def __str__(self):
        return "({}, {}, {}, {})".format(*cspaceToState(self.T1I, self.T2I, self.T3I, self.DI))
    
    def __repr__(self):
        return "CSpaceNode{}".format(str(self))

    def validNode(self):

        for nodeFilter in self.nodeFilters:
            if not nodeFilter(self.T1I, self.T2I, self.T3I, self.DI):
                return False

        return True


    def adj(self):

        ret = [
            CSpaceNode(self.T1I+1, self.T2I, self.T3I, self.DI, self.nodeFilters),
            CSpaceNode(self.T1I-1, self.T2I, self.T3I, self.DI, self.nodeFilters),
            CSpaceNode(self.T1I, self.T2I+1, self.T3I, self.DI, self.nodeFilters),
            CSpaceNode(self.T1I, self.T2I-1, self.T3I, self.DI, self.nodeFilters),
            CSpaceNode(self.T1I, self.T2I, self.T3I+1, self.DI, self.nodeFilters),
            CSpaceNode(self.T1I, self.T2I, self.T3I-1, self.DI, self.nodeFilters),
            CSpaceNode(self.T1I, self.T2I, self.T3I, self.DI+1, self.nodeFilters),
            CSpaceNode(self.T1I, self.T2I, self.T3I, self.DI-1, self.nodeFilters)
        ]

        ret = [node for node in ret if self.validNode()]

        return ret

    def estCost(self, node):

        T1I_cost = abs(self.T1I - node.T1I)
        T2I_cost = abs(self.T2I - node.T2I)
        T3I_cost = abs(self.T3I - node.T3I)
        DI_cost  = abs(self.DI  - node.DI)

        return T1I_cost + T2I_cost + T3I_cost + DI_cost

    def cost(self, node):
        if self.estCost(node) != 1:
            raise RuntimeError("Node not adjacent")
        return 1

def followPath(path):

    for node in path:
        rate(30)

        state = cspaceToState(node.T1I, node.T2I, node.T3I, node.DI)
        renderForwardKinematics(*state)



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

# originAxis(10)

renderForwardKinematics(0, 0, 0, 0)

startCspace = stateToCspace(0, 0, 0, 0)
endCspace = stateToCspace(math.pi, math.pi/2, -math.pi/2, 1)

startNode = CSpaceNode(*startCspace, [])
endNode = CSpaceNode(*endCspace, [])

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