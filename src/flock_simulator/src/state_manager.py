import duckie_dynamics
import duckietown_map
import utils
import random
import numpy as np
import networkx as nx
import duckietown_world as dw


class StateManager(object):
    def __init__(self, map_name, n_duckies, t_requests):
        # Parameters
        self.t_requests = t_requests  # Seconds between requests
        self.n_duckies = n_duckies  # Number of duckies
        self.fov = [2.0 / 3.0 * np.pi,
                    2.0]  # Field of view (angle, distance in tiles)
        self.max_vel = 0.5  # Max. velocity in m/s
        self.stop_distance = 0.1  # Distance between duckies in m
        self.duckiebot_length = 0.2  # Length of duckiebot in m

        # Map
        self.dt_map = duckietown_map.DuckietownMap(map_name)

        # State of duckies
        self.duckies = duckie_dynamics.spawnDuckies(self.n_duckies,
                                                    self.dt_map)

        # Requests
        self.requests = {}
        self.filled_requests = {}
        self.t_last_request = 0

        # Timestep
        self.timestep = 0

    def updateState(self, commands, dt):
        duckies_update = {}
        for duckie_id in self.duckies:
            duckie = self.duckies[duckie_id]

            # Use commands if received, otherwise drive around randomly
            if duckie_id in commands:
                command = commands[duckie_id]
            else:
                command = None

            # Update duckie's state
            duckies_update[duckie_id] = duckie_dynamics.updateDuckie(
                self.duckies, duckie, command, self.stop_distance,
                self.duckiebot_length, self.max_vel, self.dt_map, dt)

            # Print duckie's pose
            # print('%s: [%f, %f], %f' %
            #       (duckie_id, duckies_update[duckie_id]['pose'].p[0],
            #        duckies_update[duckie_id]['pose'].p[1],
            #        duckies_update[duckie_id]['pose'].theta))

        # Update what every duckiebot sees
        for duckie_id in self.duckies:
            in_fov = []
            duckie_pose = self.duckies[duckie_id]['pose']
            for other_duckie in self.duckies:
                if duckie_id == other_duckie:
                    continue
                other_pose = self.duckies[other_duckie]['pose']
                if utils.distance(
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
                distance = utils.distance(other_pose, self_pose)
                if distance < self.duckiebot_length:
                    c_level = 1
                # Only check if next_point is defined for duckie
                elif self.duckies[duckie_id]['next_point'] and duckie_dynamics.lane_distance(
                        self.duckies, duckie_id, self.dt_map):
                    c_level = 2
                else:
                    c_level = 0
            duckies_update[duckie_id]['collision_level'] = c_level

        self.duckies.update(duckies_update)

        # Requests
        self.updateRequests(commands)
        if self.timestep - self.t_last_request > self.t_requests / dt:
            request_id = 'request-%d' % (
                len(self.requests) + len(self.filled_requests))
            request = self.genRequest()
            self.requests[request_id] = request
            self.t_last_request = self.timestep
            print('New request added. Current open requests: %s' % list(
                self.requests))

        self.timestep += 1

    def genRequest(self):
        start_node = random.choice(self.dt_map.nodes)[0]
        end_node = start_node
        while end_node == start_node:
            end_node = random.choice(self.dt_map.nodes)[0]
        return {
            'timestep': self.timestep,
            'start_node': start_node,
            'end_node': end_node,
            'duckie_id': None
        }

    def updateRequests(self, commands):
        for duckie_id in commands:
            command = commands[duckie_id]
            if command['request_id']:
                request_id = command['request_id']
                if (self.duckies[duckie_id]['status'] == 'DRIVINGWITHCUSTOMER'
                    ) and (self.requests[request_id]['duckie_id'] !=
                           duckie_id):
                    print('Cannot reassign duckie driving with customer.')
                    continue

                # Update status
                pose_start = self.dt_map.nodeToPose(
                    self.requests[request_id]['start_node'])
                dist = utils.distance(self.duckies[duckie_id]['pose'],
                                      pose_start)
                if dist * self.dt_map.tile_size < self.duckiebot_length:
                    print('%s has been picked up.' % request_id)
                    self.requests[request_id]['duckie_id'] = duckie_id
                    self.duckies[duckie_id]['status'] = 'DRIVINGWITHCUSTOMER'
                else:
                    self.duckies[duckie_id]['status'] = 'DRIVINGTOCUSTOMER'
            else:
                self.duckies[duckie_id]['status'] = 'IDLE'

        # Update filled_requests
        requests = self.requests.copy()
        for request_id in self.requests:
            request = self.requests[request_id]
            duckie_id = request['duckie_id']
            if duckie_id and self.duckies[duckie_id]['status'] == 'DRIVINGWITHCUSTOMER':
                pose_end = self.dt_map.nodeToPose(request['end_node'])
                dist = utils.distance(self.duckies[duckie_id]['pose'],
                                      pose_end)
                if dist * self.dt_map.tile_size < self.duckiebot_length:
                    print('%s has been dropped off.' % request_id)
                    self.filled_requests[request_id] = request
                    del requests[request_id]
                    self.duckies[duckie_id]['status'] = 'IDLE'
        self.requests = requests.copy()
