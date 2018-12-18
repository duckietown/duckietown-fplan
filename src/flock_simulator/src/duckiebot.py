import random
import duckietown_world as dw


class Duckiebot(object):
    def __init__(self, id, lane, dt_map):
        self.id = id
        self.pose = dw.SE2Transform([0.0, 0.0], 0.0)
        self.velocity = {'linear': 0.0, 'angular': 0.0}
        self.next_point = {
            'lane': None,
            'point_index': 0,
            'pose': dw.SE2Transform([0.0, 0.0], 0.0)
        }
        self.status = 'IDLE'
        self.in_fov = []
        self.collision_level = 0
        self.initialize(lane, dt_map)

    def initialize(self, lane, dt_map):
        point_index = random.choice(
            range(len(dt_map.lanes[lane].control_points)))
        pose = dt_map.lanes[lane].control_points[point_index]
        nodes = dt_map.laneToNodes(lane)
        node_next = random.choice(list(dt_map.graph.neighbors(nodes[1])))
        lane_next = dt_map.graph.get_edge_data(nodes[1], node_next)[0]['lane']
        pose_next = dt_map.lanes[lane_next].control_points[0]

        self.pose = pose
        self.next_point = {
            'lane': lane_next,
            'point_index': 0,
            'pose': pose_next
        }
