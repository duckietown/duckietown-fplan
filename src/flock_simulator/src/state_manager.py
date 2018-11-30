import duckie_dynamics
import numpy as np
import networkx as nx
import duckietown_world as dw


class StateManager(object):
    def __init__(self):
        # Parameters
        self.n_duckies = 3  # Number of duckies
        self.fov = [2.0 / 3.0 * np.pi,
                    2.0]  # Field of view (angle, distance in tiles)
        self.max_vel = 0.5  # Max. velocity in m/s
        self.stop_distance = 0.2  # Distance between duckies in m
        self.duckiebot_length = 0.2  # Length of duckiebot in m

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

            # Use commands if received, otherwise drive around randomly
            if duckie_id in commands:
                command = commands[duckie_id]
                self.duckies[duckie_id]['next_point'] = None
            else:
                command = duckie_dynamics.getRandomCommand(
                    self.duckies, duckie, self.stop_distance, self.max_vel,
                    self.map.tile_size, dt)

            # Update duckie's state
            duckies_update[duckie_id] = duckie_dynamics.updateDuckie(
                self.duckies, duckie, command, self.skeleton_graph,
                self.map.tile_size, dt)

            # Print duckie's pose
            print('%s: [%f, %f], %f' %
                  (duckie_id, duckies_update[duckie_id]['pose'].p[0],
                   duckies_update[duckie_id]['pose'].p[1],
                   duckies_update[duckie_id]['pose'].theta))

        # Update what every duckiebot sees
        for duckie_id in self.duckies:
            in_fov = []
            duckie_pose = self.duckies[duckie_id]['pose']
            for other_duckie in self.duckies:
                if duckie_id == other_duckie:
                    continue
                other_pose = self.duckies[other_duckie]['pose']
                if duckie_dynamics.distance(
                        duckie_pose, other_pose
                ) < self.fov[1] and duckie_dynamics.isInFront(
                        duckie_pose, other_pose, self.fov[0]):
                    in_fov.append(other_duckie)
            duckies_update[duckie_id]['in_fov'] = in_fov

        # collision_level detection in duckietown
        for duckie_id in self.duckies:
            for other_duckie in self.duckies:
                if duckie_id == other_duckie:
                    continue
                other_pose = self.duckies[other_duckie]['pose']
                self_pose = self.duckies[duckie_id]['pose']
                distance = duckie_dynamics.distance(other_pose, self_pose)
                lane_distance = duckie_dynamics.lane_distance(
                    self.duckies, duckie_id, self.skeleton_graph)
                if distance < self.duckiebot_length:
                    c_level = 1
                elif lane_distance:
                    c_level = 2
                else:
                    c_level = 0
            duckies_update[duckie_id]['collision_level'] = c_level

        self.duckies.update(duckies_update)
