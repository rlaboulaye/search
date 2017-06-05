from node import Node

class RRTNode(Node):

    def __init__(self, pos):
        super(RRTNode, self).__init__(pos)
        self.parent = None
        self.children = []
        self.cost = 0

    def set_parent(self, node):
        self.parent = node
        self.cost = node.cost + self.get_dist(node)

    def add_child(self, node):
        self.children.append(node)

    def get_distance(self, node):
        return super(RRTNode, self).get_distance(node)

    def get_neighborhood(self, nodes, neighborhood_length):
        neighbors = []
        for node in nodes:
            dist = self.get_dist(node)
            if (dist <= neighborhood_length):
                neighbors.append(node)
        return neighbors
