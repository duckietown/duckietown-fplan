import paths
import duckie_dynamics
import numpy as np
import networkx as nx
import duckietown_world as dw


class StateManager(object):
    def __init__(self):
        # Parameters
        self.n_duckies = 3  # Number of duckies
        self.max_vel = 1.0 / 3.0  # Tile / s
        self.max_acc = 2.0  # Tile / s^2

        # Map
        self.map = dw.load_map('4way')
        self.map_graph = self.generateMapGraph()

        # State of duckies
        self.duckies = duckie_dynamics.spawnDuckies(self.n_duckies,
                                                    self.map_graph)

        # Paths
        self.paths = paths.generateAllPaths(self.duckies, self.map_graph.nodes)

    def updateState(self, dt):
        # Update every duckie
        for duckie in self.duckies:
            # Check path
            path_node = self.paths[duckie][0]
            if duckie in self.map_graph.nodes['tile-%d-%d' %
                                              (path_node[0],
                                               path_node[1])]['duckies']:
                self.paths[duckie].pop(0)
            if len(self.paths[duckie]) < 3:
                paths.generatePath(self.duckies, self.map_graph.nodes, duckie,
                                   self.paths[duckie])

            # Update duckie state
            vel_des = duckie_dynamics.getDesiredVelocity(
                self.duckies, self.map_graph.nodes, self.paths[duckie],
                self.max_vel, duckie)
            vel = self.duckies[duckie]['velocity']
            vel_new = vel + np.sign(vel_des - vel) * self.max_acc * dt
            traveled = (vel + vel_new) / 2 * dt
            self.duckies[duckie]['velocity'] = vel_new
            state_new = duckie_dynamics.getNewDuckieState(
                self.duckies[duckie], self.paths[duckie], traveled)
            self.duckies[duckie].update(state_new)

        # Update nodes with duckies
        for node in self.map_graph:
            self.map_graph.nodes[node]['duckies'][:] = []
        for duckie in self.duckies:
            pose = self.duckies[duckie]['pose']
            coord = [round(pose['x']), round(pose['y'])]
            self.map_graph.nodes['tile-%d-%d' %
                                 (coord[0],
                                  coord[1])]['duckies'].append(duckie)
        print(self.duckies)

    def generateMapGraph(self):
        tilemap = self.map.children['tilemap']
        map_size = [tilemap.H, tilemap.W]
        graph = nx.Graph()

        # Add one node per drivable tile
        for name, tile in tilemap.children.items():
            str_parse = name.split('-')
            if str_parse[0] != 'tile' or not tile.drivable:
                continue
            graph.add_node(
                name, coord=[int(str_parse[1]),
                             int(str_parse[2])], duckies=[])

        # Add edges to adjacent tiles (does not consider type of tile!)
        for x in range(map_size[0]):
            for y in range(map_size[1]):
                current_tile = 'tile-%d-%d' % (x, y)
                right_tile = 'tile-%d-%d' % (x, y + 1)
                bottom_tile = 'tile-%d-%d' % (x + 1, y)
                if current_tile not in graph:
                    continue
                if right_tile in graph:
                    graph.add_edge(current_tile, right_tile)
                if bottom_tile in graph:
                    graph.add_edge(current_tile, bottom_tile)

        # Find exits and type of tile
        for node in graph:
            node_coords = graph.nodes[node]['coord']
            exits = []
            for neighbor in graph.neighbors(node):
                neighbor_coords = graph.nodes[neighbor]['coord']
                diff = [b - a for a, b in zip(node_coords, neighbor_coords)]
                exits.append(diff)
            if len(exits) > 2:
                graph.nodes[node]['type'] = 'intersection'
            elif [b + a for a, b in zip(exits[0], exits[1])] == [0, 0]:
                graph.nodes[node]['type'] = 'straight'
            else:
                graph.nodes[node]['type'] = 'curve'
            graph.nodes[node]['exits'] = exits
        return graph
