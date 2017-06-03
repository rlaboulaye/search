from rrt_search import RRTSearch

field_dim = [50,60]
tag_radius = 2
robot_radius = 2

start_pos = [5,5]
end_pos = [43,43]
obstacle_pos = [[12,12],[30,30],[40,20],[24,39]]
unit_length = 3

s = RRTSearch(field_dim, tag_radius, robot_radius)
path = s.get_path(start_pos, end_pos, obstacle_pos, unit_length)
