
class AStarNode:
    """
    A node in A*, this class MUST implement __eq__
    """

    def adj(self):
        """
        Returns array of adjacent nodes
        """
        raise NotImplementedError()

    def estCost(self, node):
        """
        Returns the estimated cost to the goal node.
        """
        raise NotImplementedError()

    def cost(self, node):
        """
        Returns the true cost of traveling to an adjacent node
        """
        raise NotImplementedError()

class AStarPath:

    def __init__(self, oldPath=None, node=None):

        self.cost = oldPath.cost if oldPath != None else 0
        self.nodes = oldPath.nodes.copy() if oldPath != None else []

        if node != None:
            self.nodes.append(node)
    
    def __str__(self):
        ret = "cost: {}\n".format(self.cost)

        for node in self.nodes:
            ret += str(node) + "\n"
        
        return ret
    
    def estCost(self, node):
        return self.cost + self.nodes[-1].estCost(node)

    def getAdjPaths(self):
        endNode = self.nodes[-1]

        adjNodes = endNode.adj()

        return [ AStarPath(self, n) for n in adjNodes if n not in self.nodes]


def AStar(startNode, endNode):

    pathList = [AStarPath(node=startNode)]

    while True:

        if len(pathList) == 0:
            raise RuntimeError("Could not find path")

        path = pathList.pop(0)

        if path.nodes[-1] == endNode:
            return path.nodes
        
        pathList += path.getAdjPaths()
        pathList.sort(key=lambda p: p.estCost(endNode))

if __name__ == "__main__":

    class XYNode(AStarNode):

        def __init__(self, x, y):
            self.x = x
            self.y = y

        def __eq__(self, other):
            if not hasattr(other, "x") or not hasattr(other, "y"):
                return False
            return self.x == other.x and self.y == other.y
        
        def __str__(self):
            return "({}, {})".format(self.x, self.y)
        
        def __repr__(self):
            return "XYNode({}, {})".format(self.x, self.y)

        def adj(self):
            return [
                XYNode(self.x+1, self.y),
                XYNode(self.x-1, self.y),
                XYNode(self.x, self.y+1),
                XYNode(self.x, self.y-1)
            ]

        def estCost(self, node):
            return abs(self.x - node.x) + abs(self.y - node.y)

        def cost(self, node):
            if self.estCost(node) != 1:
                raise RuntimeError("Node not adjacent")
            return 1
    
    print(AStar(XYNode(0, 0), XYNode(5, 5)))
    