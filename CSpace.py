import AStar
import math
import numpy as np

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
    50
)

cspace = np.ones(cspaceSegs, np.bool)

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

    if T1 == cspaceLim[0][1]:
        T1 -= T1_step/2

    if T2 == cspaceLim[1][1]:
        T2 -= T2_step/2

    if T3 == cspaceLim[2][1]:
        T3 -= T3_step/2

    if D  == cspaceLim[3][1]:
        D  -= D_step/2

    return (
        math.floor((T1 - cspaceLim[0][0]) / T1_step),
        math.floor((T2 - cspaceLim[1][0]) / T2_step),
        math.floor((T3 - cspaceLim[2][0]) / T3_step),
        math.floor((D  - cspaceLim[3][0]) / D_step)
    )

def allComb(combArr):

    ret = map(lambda x: [x], combArr[0])

    for i in range(len(combArr)-1):
        newRet = []

        for r in ret:
            newRet += map(lambda x: r + [x], combArr[i+1])
        
        ret = newRet

    return ret

def notOutOfBounds(T1I, T2I, T3I, DI):
    if T1I < 0 or T2I < 0 or T3I < 0 or DI < 0:
        return False

    if T1I >= cspaceSegs[0]:
        return False

    if T2I >= cspaceSegs[1]:
        return False

    if T3I >= cspaceSegs[2]:
        return False

    if DI >= cspaceSegs[3]:
        return False

    return True

class CSpaceNode(AStar.AStarNode):

    def __init__(self, T1I, T2I, T3I, DI, nodeFilters):
        self.T1I = T1I
        self.T2I = T2I
        self.T3I = T3I
        self.DI = DI
        self.nodeFilters = [notOutOfBounds] + nodeFilters
        # self.nodeFilters = nodeFilters

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

    def getState(self):
        return cspaceToState(self.T1I, self.T2I, self.T3I, self.DI)

    def adj(self):

        ret = map(lambda c: CSpaceNode(*c, self.nodeFilters), allComb([
            [self.T1I-1, self.T1I, self.T1I+1],
            [self.T2I-1, self.T2I, self.T2I+1],
            [self.T3I-1, self.T3I, self.T3I+1],
            [self.DI-1 , self.DI , self.DI+1 ],
        ]))

        ret = [node for node in ret if node.validNode()]

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
