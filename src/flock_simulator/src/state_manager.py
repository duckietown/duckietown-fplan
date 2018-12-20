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

        # Timestep
        self.timestep = 0

        # Duckies
        self.duckies = {}
        self.spawnDuckies()

        # Requests
        self.requests = {}
        self.filled_requests = {}
        self.t_last_request = 0

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
            if command['request_id'] in self.requests:
                request = self.requests[command['request_id']] if command[
                    'request_id'] else None
                duckie = self.duckies[duckie_id]
            else:
                # If dispatcher assigns request to duckie that is not available anymore
                continue

            # Duckie can be IDLE, REBALANCING, DRIVINGTOCUSTOMER, DRIVINGWITHCUSTOMER
            # Request can be none, WAITING, PICKEDUP, FILLED
            # command['path'] can be empty or not

            if duckie.status != 'DRIVINGWITHCUSTOMER':
                if not request:
                    if command['path']:
                        duckie.status = 'REBALANCING'
                        duckie.path = command['path']
                elif request.status == 'WAITING':
                    if duckie.reachedNode(request.start_node, self.dt_map):
                        duckie.status = 'DRIVINGWITHCUSTOMER'
                        request.status = 'PICKEDUP'
                        request.duckie_id = duckie.id
                        request.pickup_time = self.timestep
                        print('%s has been picked up by %s' % (request.id,
                                                               duckie.id))
                    else:
                        duckie.status = 'DRIVINGTOCUSTOMER'
                    if command['path']:
                        duckie.path = command['path']
                    else:
                        print('%s assigned to %s but has no path to follow.' %
                              (duckie.id, request.id))
                elif request:
                    print('%s is %s but %s is assigned and %s!' %
                          (request.id, request.status, duckie.id,
                           duckie.status))
            elif duckie.status == 'DRIVINGWITHCUSTOMER':
                if not request:
                    print(
                        '%s is driving with customer but not assigned to a request!'
                        % duckie.id)
                elif request.status == 'PICKEDUP':
                    if duckie.reachedNode(request.end_node, self.dt_map):
                        duckie.status = 'IDLE'
                        request.status = 'FILLED'
                        request.end_time = self.timestep
                        print('%s has been dropped off by %s' % (request.id,
                                                                 duckie.id))
                    if command['path']:
                        duckie.path = command['path']
                    else:
                        print('%s picked up %s but has no path to follow.' %
                              (duckie.id, request.id))

        # Move from requests to filled_requests
        for request_id in list(self.requests.keys()):
            if self.requests[request_id].status == 'FILLED':
                self.filled_requests[request_id] = self.requests[request_id]
                del self.requests[request_id]

        # Add request
        if self.t_requests != 0 and self.timestep - self.t_last_request > self.t_requests / dt:
            self.addRequest()

    def addRequest(self):
        request_id = 'request-%d' % (
            len(self.requests) + len(self.filled_requests))
        self.requests[request_id] = Request(request_id, self.dt_map.nodes,
                                            self.timestep)
        self.t_last_request = self.timestep
        print('New request added. Number of open requests: %d' % len(
            self.requests))

    def spawnDuckies(self):
        occupied_poses = []
        for i in range(self.n_duckies):
            spawn_is_occupied = True
            while spawn_is_occupied:
                lane = random.sample(self.dt_map.lanes, 1)[0]
                point_index = random.choice(
                    range(len(self.dt_map.lanes[lane].control_points)))
                pose = self.dt_map.lanes[lane].control_points[point_index]
                point = {
                    'lane': lane,
                    'point_index': point_index,
                    'pose': pose
                }

                spawn_is_occupied = False
                for occupied_pose in occupied_poses:
                    if utils.distance(pose,
                                      occupied_pose) < self.dt_map.tile_size:
                        spawn_is_occupied = True
                        break

                if not spawn_is_occupied:
                    duckie_id = 'duckie-%d' % i
                    self.duckies[duckie_id] = Duckiebot(
                        duckie_id, point, self.dt_map)
                    occupied_poses.append(pose)
