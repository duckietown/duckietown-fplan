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
        # Give commands
        for duckie_id, duckie in self.duckies.items():
            # Use commands if received, otherwise drive around randomly
            command = commands[duckie_id] if duckie_id in commands else None

            # Update duckie's state
            duckie.update(self.duckies, command, self.dt_map, dt)

        # Update interactions between duckies
        for duckie in self.duckies.values():
            duckie.updateFov(self.duckies)
            duckie.updateCollision(self.duckies, self.dt_map.tile_size)

        # Requests
        self.updateRequests(commands, dt)

        self.timestep += 1

    def updateRequests(self, commands, dt):
        for duckie_id, command in commands.items():
            duckie = self.duckies[duckie_id]
            if command['request_id']:
                request_id = command['request_id']
                request = self.requests[request_id]
                if duckie.status == 'DRIVINGWITHCUSTOMER' and request.duckie_id != duckie.id:
                    print('Cannot reassign duckie driving with customer.')
                    continue

                # Update status
                pose_start = self.dt_map.nodeToPose(request.start_node)
                dist = utils.distance(duckie.pose, pose_start)
                if dist * self.dt_map.tile_size < duckie.length:
                    print('%s has been picked up.' % request_id)
                    request.duckie_id = duckie.id
                    request.status = 'PICKEDUP'
                    request.pickup_time = self.timestep
                    duckie.status = 'DRIVINGWITHCUSTOMER'
                else:
                    duckie.status = 'DRIVINGTOCUSTOMER'
            else:
                duckie.status = 'IDLE'

        # Update requests
        for request in self.requests.values():
            if not request.duckie_id:
                continue
            request.update(self.duckies[request.duckie_id], self.dt_map,
                           self.timestep)

        # Move from requests to filled_requests
        for request_id in list(self.requests.keys()):
            if self.requests[request_id].status == 'FILLED':
                self.filled_requests[request_id] = self.requests[request_id]
                del self.requests[request_id]

        # Add request
        if self.timestep - self.t_last_request > self.t_requests / dt:
            request_id = 'request-%d' % (
                len(self.requests) + len(self.filled_requests))
            self.requests[request_id] = Request(request_id, self.dt_map.nodes,
                                                self.timestep)
            self.t_last_request = self.timestep
            print('New request added. Number of open requests: %d' % len(
                self.requests))

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
