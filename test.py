from rrt_search import RRTSearch

field_dim = [2000,1000]
tag_radius = 100
robot_radius = 100

start_pos = [600,300]
end_pos = [1500, 700]
obstacle_pos = [[1000,400]]
unit_length = 100

s = RRTSearch(field_dim, tag_radius, robot_radius)
path = s.get_path(start_pos, end_pos, obstacle_pos, unit_length)
