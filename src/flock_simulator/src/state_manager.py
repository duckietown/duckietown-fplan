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

        # State of duckies
        self.duckies = duckie_dynamics.spawnDuckies(self.n_duckies,
                                                    dw.get_skeleton_graph(
                                                        self.map['tilemap']))

    def updateState(self, commands, dt):
        duckies_update = {}
        for duckie_id in self.duckies:
            duckie = self.duckies[duckie_id]
            if duckie_id in commands:
                command = commands[duckie_id]
            else:
                command = duckie_dynamics.getRandomCommand(
                    self.map, duckie, dt)
            duckies_update[duckie] = duckie_dynamics.updateDuckie(
                self.duckies, duckie, command, self.map.tile_size, dt)
        self.duckies = duckies_update