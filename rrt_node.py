from node import Node

class RRTNode(Node):

    def __init__(self, pos):
        super(RRTNode, self).__init__(pos)
        self.parent = None
        self.children = []

    def set_parent(self, node):
        self.parent = node

    def add_child(self, node):
        self.children.append(node)

    def get_distance(self, node):
        return super(RRTNode, self).get_distance(node)
