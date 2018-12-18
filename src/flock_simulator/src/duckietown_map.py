import duckietown_world as dw


class DuckietownMap(object):
    def __init__(self, map_name):
        self.map = dw.load_map(map_name)
        self.tile_size = self.map.tile_size
        self.skeleton_graph = dw.get_skeleton_graph(self.map['tilemap'])
        self.graph = self.skeleton_graph.G
        self.edges = list(self.graph.edges(data=True))
        self.nodes = list(self.graph.nodes(data=True))
        self.lanes = self.skeleton_graph.root2.children

    def laneToNodes(self, lane):
        edge_data = [edge for edge in self.edges if edge[2]['lane'] == lane]
        return [edge_data[0][0], edge_data[0][1]]

    def nodeToPose(self, node):
        return self.graph.nodes(data=True)[node]['point']
