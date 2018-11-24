import paths
import duckie_dynamics
import numpy as np
import networkx as nx
import duckietown_world as dw


class StateManager(object):
    def __init__(self):
        # Parameters
        self.n_duckies = 3  # Number of duckies

        # Map
        self.map = dw.load_map('4way')
        self.skeleton_graph = dw.get_skeleton_graph(self.map['tilemap'])

        # State of duckies
        self.duckies = duckie_dynamics.spawnDuckies(self.n_duckies,
                                                    self.skeleton_graph)

    def updateState(self, commands, dt):
        duckies_update = {}
        for duckie_id in self.duckies:
            duckie = self.duckies[duckie_id]
            if duckie_id in commands:
                command = commands[duckie_id]
                self.duckies[duckie_id]['next_point'] = None
            else:
                command = duckie_dynamics.getRandomCommand(duckie)
            duckies_update[duckie_id] = duckie_dynamics.updateDuckie(
                self.duckies, duckie, command, self.skeleton_graph,
                self.map.tile_size, dt)
        self.duckies.update(duckies_update)
