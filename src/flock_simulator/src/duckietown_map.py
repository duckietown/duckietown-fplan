import random
import duckietown_world as dw
import networkx as nx


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

    def nodesToLane(self, nodes):
        return self.graph.get_edge_data(nodes[0], nodes[1])[0]['lane']

    def getRandomPath(self, start_node):
        end_node = start_node
        while end_node == start_node:
            end_node = random.choice(self.nodes)[0]
        return self.getPath(start_node, end_node)

    def getPath(self, start_node, end_node):
        return nx.dijkstra_path(self.graph, start_node, end_node)
