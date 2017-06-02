import math
import queue

from search import Search

def reconstruct_path(came_from, current_node):
    path = [current_node]
    while current_node in came_from.keys():
        current_node = came_from[current_node]
        path.append(current_node)
    return array.reverse(path)

def dist(pos1, pos2):
    return math.sqrt(((pos2[0] - pos1[0]) ** 2) + ((pos2[1] - pos1[1]) ** 2))

class AStar(Search):

    def __init__(self, field_dim, tag_radius, robot_radius):
        super(AStar, self).__init__(field_dim, tag_radius, robot_radius)
        self.error_tolerance = 0

    def init_graph(self, obstacle_pos_list, unit_length):
        graph = {}
        x = 0
        y = 0
        while y <= self.field_dim[1]:
            while x <= self.field_dim[0]:
                graph[Node([x, y])] = {}
                x = x + unit_length
            y = y + unit_length
        exclude_length = self.tag_radius + self.robot_radius
        for node in graph.keys():
            for obstacle_pos in obstacle_pos_list:
                if dist(node.pos, obstacle_pos) <= exclude_dist:
                    del graph[node]
        for node1 in graph.keys():
            for node2 in graph.keys():
                if node1.__eq__(node2):
                    continue
                if node1 in graph[node2] and node2 in graph[node1]:
                    continue
                if (node1.pos[0] - node2.pos[0] >= -unit_length and node1.pos[0] - node2.pos[0] <= unit_length and \
                    node1.pos[1] - node2.pos[1] >= -unit_length and node1.pos[1] - node2.pos[1] <= unit_length):
                    graph[node1].add(node2)
                    graph[node2].add(node1)
        return graph


# start_pos = [x, y], end_pos = [x, y], obstacle_pos = [[x0, y0], [x1, y1], ...], unit_length = d
    def get_path(self, start_pos, end_pos, obstacle_pos, unit_length):
        self.error_tolerance = unit_length / 20.
        graph = self.init_graph(unit_length)
        came_from = {}
        open_queue = queue.PriorityQueue()
        closed_set = set()
        start_node = graph[Node(start_pos)]
        start_node.cost_from_start = 0
        start_node.total_cost = start_node.get_heuristic(end_pos)
        open_queue.put((start_node.total_cost, start_node))
        while not open_queue.empty():
            current_node = open_queue.get()[1]
            if current_node.get_heuristic(end_pos) < self.error_tolerance:
                return reconstruct_path(came_from, current_node)
            closed_set.add(current_node)
            neighbor_nodes = graph[current_node]
            for neighbor_node in neighbor_nodes:
                if neighbor_node in closed_set:
                    continue
                tentative_cost_from_start = current_node.cost_from_start + dist(current_node.pos, neighbor_node.pos)
                if neighbor_node not in open_queue:
                    open_queue.put((tentative_cost_from_start + neighbor_node.get_heuristic, neighbor_node))
                elif tentative_cost_from_start >= neighbor_node.cost_from_start:
                    continue
                came_from[neighbor_node] = current_node
                neighbor_node.cost_from_start = tentative_cost_from_start
                neighbor_node.total_cost = neighbor_node.cost_from_start + neighbor_node.get_heuristic(end_pos)
        return None



class Node(object):

    def __init__(self, pos):
        self.pos = pos
        self.cost_from_start = float('inf')
        self.total_cost = float('inf')

    def __eq__(self, other):
        return (self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1])

    def __hash__(self):
        return pos[0] + pos[1]

    def get_heuristic(self, end_pos):
        return math.sqrt(((end_pos[0] - self.current_pos[0]) ** 2) + ((end_pos[1] - self.current_pos[1]) ** 2))
