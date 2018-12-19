#!/usr/bin/env python

import networkx as nx
import numpy as np
from flock_simulator.msg import FlockState, FlockCommand


class Dispatcher(object):
    def __init__(self, skeleton_graph):
        self.skeleton_graph = skeleton_graph

        # Commands
        self.commands = []

    def update(self, state):
        # EXAMPLE FOR STATE:
        # state = {
        #   'duckies': [
        #       'duckie-0': {
        #           'status': 'IDLE',
        #           'lane': 'l001',
        #       },
        #       'duckie-1': {
        #           'status': 'IDLE',
        #           'lane': 'l042',
        #       }, ...
        #   ]
        #   'requests': [
        #       'request-0': {
        #           'time': [time of request],
        #           'duckie_id': [duckie which is serving the request (empty if unassigned)],
        #           'start_node': [start node of graph (networkx)],
        #           'end_node': [end node of graph (networkx)],
        #       }, ...
        #   ]
        # }

        duckies = state['duckies']
        requests = state['requests']
        paths = []

        # Get open requests
        open_requests = {}
        requests_with_duckie = {}
        for request_id in requests:
            if requests[request_id]['duckie_id']:
                requests_with_duckie[request_id] = requests[request_id]
            else:
                open_requests[request_id] = requests[request_id]

        assigned_requests = []

        # Generate paths for duckies
        for duckie_id, duckie in duckies.items():
            current_node = self.node(
                duckie['lane'])  # Node the duckie is heading to

            if duckie['status'] == 'DRIVINGWITHCUSTOMER':
                assigned_request_id, assigned_request = next(
                    (request_id, request)
                    for request_id, request in requests_with_duckie.items()
                    if request['duckie_id'] == duckie_id)
                goal_node = assigned_request['end_node']

            else:
                # find closest open request
                closest_request_id = None
                closest_distance = np.inf
                for open_request_id in open_requests:
                    open_request = open_requests[open_request_id]
                    distance = self.dist(current_node, open_request)
                    if open_request_id not in assigned_requests and distance < closest_distance:
                        closest_distance = distance
                        closest_request_id = open_request_id
                if closest_request_id:
                    assigned_request_id = closest_request_id
                    assigned_request = open_requests[closest_request_id]
                    goal_node = assigned_request['start_node']
                    assigned_requests.append(assigned_request_id)
                else:
                    continue

            # generate path and assign
            path_pair = self.generatePathPair(duckie_id, assigned_request_id,
                                              current_node, goal_node)
            paths.append(path_pair)

        # generateCommands from path
        # EXAMPLE FOR PATHS:
        # paths = [
        #   {
        #       'duckie_id': 'duckie-0',
        #       'request_index': 2,
        #       'path': [list of nodes]
        #   },
        #   {
        #       'duckie_id': 'duckie-1',
        #       'request_index': 0,
        #       'path': [list of nodes]
        #   }, ...
        # ]

        self.commands = paths

    def getClosestRequest(self, open_requests, node):
        if not open_requests:  # if no open requests
            return
        closest_request = open_requests.keys()[0]
        for open_request in open_requests:
            if self.dist(node, open_request) < self.dist(
                    node, closest_request):
                closest_request = open_request

        return closest_request

    def dist(self, node, request):
        # generate dijkstra_distance (closest)
        return nx.dijkstra_path_length(self.skeleton_graph.G, node,
                                       request['start_node'])

    # generate dijkstra_path
    def generatePathPair(self, duckie_id, request_id, current_node, goal_node):
        path = {}
        path['duckie_id'] = duckie_id
        path['request_id'] = request_id
        path['path'] = nx.dijkstra_path(self.skeleton_graph.G, current_node,
                                        goal_node)
        return path

    def node(self, lane):
        # Get end node of lane
        edges = list(self.skeleton_graph.G.edges(data=True))
        edge_data = [edge for edge in edges if edge[2]['lane'] == lane]
        return edge_data[0][1]
