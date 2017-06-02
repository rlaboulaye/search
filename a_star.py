import math
import queue

from search import Search

def reconstruct_path(came_from, current_node):
    path = [current_node]
    while current_node in came_from.keys():
        current_node = came_from[current_node]
        path.append(current_node)
    return path.reverse()

def dist(pos1, pos2):
    return math.sqrt(((pos2[0] - pos1[0]) ** 2) + ((pos2[1] - pos1[1]) ** 2))

class AStar(Search):

    def __init__(self, field_dim, tag_radius, robot_radius):
        super(AStar, self).__init__(field_dim, tag_radius, robot_radius)
        self.graph = {}
        self.error_tolerance = 0.05

    def collision(self, node_pos, obstacle_pos_list, exclude_dist):
        for obstacle_pos in obstacle_pos_list:
            if dis(node.pos, obstacle_pos) <= exclude_dist:
                return true
        return false

    def init_graph(self, obstacle_pos_list, unit_length):
        exclude_dist = self.tag_radius + self.robot_radius
        x = 0
        y = 0
        while y <= self.field_dim[1]:
            x = 0
            while x <= self.field_dim[0]:
                if not self.collision([x, y], obstacle_pos_list, exclude_dist):
                    pos = Position([x, y])
                    graph[pos] = (Node(pos), set())
                    left_pos = Position([x - unit_length, y])
                    up_pos = Position([x, y - unit_length])
                    up_left_pos = Position([x - unit_length, y - unit_length])
                    up_right_pos = Position([x + unit_length, y - unit_length])
                    if left_pos in graph.keys():
                        graph[pos][1].add(graph[left_pos][0])
                        graph[left_pos][1].add(graph[pos][0])
                    if up_pos in graph.keys():
                        graph[pos][1].add(graph[up_pos][0])
                        graph[up_pos][1].add(graph[pos][0])
                    if up_left_pos in graph.keys():
                        graph[pos][1].add(graph[up_left_pos][0])
                        graph[up_left_pos][1].add(graph[pos][0])
                    if up_right_pos in graph.keys():
                        graph[pos][1].add(graph[up_right_pos][0])
                        graph[up_right_pos][1].add(graph[pos][0])
                x = x + unit_length
            y = y + unit_length
        return
#        for node in graph.keys():
#            for obstacle_pos in obstacle_pos_list:
#                if dist(node.pos, obstacle_pos) <= exclude_dist:
#                    del graph[node]
#        for node1 in graph.keys():
#            for node2 in graph.keys():
#                if node1.__eq__(node2):
#                    continue
#                if node1 in graph[node2] and node2 in graph[node1]:
#                    continue
#                if (node1.pos[0] - node2.pos[0] >= -unit_length and node1.pos[0] - node2.pos[0] <= unit_length and \
#                    node1.pos[1] - node2.pos[1] >= -unit_length and node1.pos[1] - node2.pos[1] <= unit_length):
#                    graph[node1].add(node2)
#                    graph[node2].add(node1)
#        return graph


# start_pos = [x, y], end_pos = [x, y], obstacle_pos = [[x0, y0], [x1, y1], ...], unit_length = d
    def get_path(self, start_pos, end_pos, obstacle_pos, unit_length):
        self.init_graph(obstacle_pos, unit_length)
        came_from = {}
        open_queue = queue.PriorityQueue()
        closed_set = set()
        start_pos_obj = Position(start_pos)
        start_node = graph[start_pos_obj][0]
        start_node.cost_from_start = 0
        start_node.total_cost = start_node.get_heuristic(end_pos)
        open_queue.put((start_node.total_cost, start_node))
        while not open_queue.empty():
            current_node = open_queue.get()[1]
            if current_node.get_heuristic(end_pos) < self.error_tolerance:
                return reconstruct_path(came_from, current_node)
            closed_set.add(current_node)
            neighbor_nodes = graph[current_node.pos][1]
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



class Position(object):

    def __init__(self, pos):
        self.x = pos[0]
        self.y = pos[1]

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y)

    def __hash__(self):
        return (x * 53) + (y * 7)


class Node(object):

    def __init__(self, pos):
        self.pos = pos
        self.cost_from_start = float('inf')
        self.total_cost = float('inf')

    def __eq__(self, other):
        return (self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1])

    def __hash__(self):
        return (pos[0] * 53) + (pos[1] * 7)

    def get_heuristic(self, end_pos):
        return math.sqrt(((end_pos[0] - self.current_pos[0]) ** 2) + ((end_pos[1] - self.current_pos[1]) ** 2))
