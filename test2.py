from rrt_star import RRTStar
from a_star import AStar

field_dim = [200,200]
tag_radius = 10
robot_radius = 10

start_pos = [50,50]
end_pos = [170, 160]
obstacle_pos = [[100,100]]
unit_length = 15

s = AStar(field_dim, tag_radius, robot_radius)
path = s.get_path(start_pos, end_pos, obstacle_pos, unit_length)
s = RRTStar(field_dim, tag_radius, robot_radius)
path = s.get_path(start_pos, end_pos, obstacle_pos, unit_length)
