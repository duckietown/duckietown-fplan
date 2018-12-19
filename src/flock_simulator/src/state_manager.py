import duckietown_map
import utils
import random
import numpy as np
import networkx as nx
import duckietown_world as dw
from duckiebot import Duckiebot
from request import Request


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

        # Duckies
        self.duckies = {}
        self.spawnDuckies()

        # Requests
        self.requests = {}
        self.filled_requests = {}
        self.t_last_request = 0

        # Timestep
        self.timestep = 0

    def updateState(self, commands, dt):
        for duckie_id in self.duckies:
            # Use commands if received, otherwise drive around randomly
            if duckie_id in commands:
                command = commands[duckie_id]
            else:
                command = None

            # Update duckie's state
            self.duckies[duckie_id].update(self.duckies, command, self.dt_map,
                                           dt)

        # Update interactions between duckies
        for duckie_id in self.duckies:
            duckie = self.duckies[duckie_id]
            duckie.in_fov[:] = []
            for other_duckie_id in self.duckies:
                if duckie_id == other_duckie_id:
                    continue

                other_duckie = self.duckies[other_duckie_id]
                distance = utils.distance(duckie.pose, other_duckie.pose)

                # Update field of view
                if distance < self.fov[1] and utils.isInFront(
                        duckie.pose, other_duckie.pose, self.fov[0]):
                    duckie.in_fov.append(other_duckie.id)

                # Update collision status
                if distance * self.dt_map.tile_size < self.duckiebot_length:
                    c_level = 1
                else:
                    c_level = 0
                duckie.collision_level = c_level

        # Requests
        self.updateRequests(commands)
        if self.timestep - self.t_last_request > self.t_requests / dt:
            request_id = 'request-%d' % (
                len(self.requests) + len(self.filled_requests))
            self.requests[request_id] = Request(request_id, self.dt_map.nodes,
                                                self.timestep)
            self.t_last_request = self.timestep
            print('New request added. Current open requests: %s' % list(
                self.requests))

        self.timestep += 1

    def updateRequests(self, commands):
        for duckie_id in commands:
            duckie = self.duckies[duckie_id]
            command = commands[duckie_id]
            if command['request_id']:
                request_id = command['request_id']
                request = self.requests[request_id]
                if duckie.status == 'DRIVINGWITHCUSTOMER' and request.duckie_id != duckie.id:
                    print('Cannot reassign duckie driving with customer.')
                    continue

                # Update status
                pose_start = self.dt_map.nodeToPose(request.start_node)
                dist = utils.distance(duckie.pose, pose_start)
                if dist * self.dt_map.tile_size < self.duckiebot_length:
                    print('%s has been picked up.' % request_id)
                    request.duckie_id = duckie.id
                    request.status = 'PICKEDUP'
                    duckie.status = 'DRIVINGWITHCUSTOMER'
                else:
                    duckie.status = 'DRIVINGTOCUSTOMER'
            else:
                duckie.status = 'IDLE'

        # Update filled_requests
        requests = self.requests.copy()
        for request_id in self.requests:
            request = self.requests[request_id]
            if not request.duckie_id:
                continue
            duckie = self.duckies[request.duckie_id]
            if request.status == 'PICKEDUP' and duckie.status == 'DRIVINGWITHCUSTOMER':
                pose_end = self.dt_map.nodeToPose(request.end_node)
                dist = utils.distance(duckie.pose, pose_end)
                if dist * self.dt_map.tile_size < self.duckiebot_length:
                    print('%s has been dropped off.' % request_id)
                    duckie.status = 'IDLE'
                    request.status = 'FILLED'
                    request.end_time = self.timestep
                    self.filled_requests[request_id] = request
                    del requests[request_id]
        self.requests = requests.copy()

    def spawnDuckies(self):
        occupied_lanes = []
        for i in range(self.n_duckies):
            spawn_is_occupied = True
            while spawn_is_occupied:
                lane = random.sample(self.dt_map.lanes, 1)[0]
                if lane not in occupied_lanes:
                    duckie_id = 'duckie-%d' % i
                    self.duckies[duckie_id] = Duckiebot(
                        duckie_id, lane, self.dt_map)
                    spawn_is_occupied = False
                    occupied_lanes.append(lane)
