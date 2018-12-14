#!/usr/bin/env python

import networkx as nx
from flock_simulator.msg import FlockState, FlockCommand


# TODO Implement duckie initial(t=0) status = 'IDLE'


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
        #       {
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
        for request in requests:
            if not request['duckie_id']:
                open_requests.update(request)

        # check if there are requests
        if open_requests:

            # Update duckiestatus
            for duckie in duckies:

                # base stuff
                lane = duckie['lane']  # Lane the duckie is currently on
                current_node = self.node(duckie['lane'])  # Node the duckie is heading to
                duckie_status = duckie['status']  # Status of the duckie
                target_location = current_node

                # IDLE
                if duckie_status == 'IDLE':

                    # find closest open request
                    closest_request = next(iter(open_requests), None)
                    for open_request in open_requests:
                        if self.dist(current_node, open_request) < self.dist(
                                    current_node, closest_request):
                            closest_request = open_request

                    start_node = closest_request['start_node']
                    end_node = closest_request['end_node']

                    # pick up customer
                    if start_node == current_node:
                        # change status AND assign open_request
                        duckie['status'] = 'WITH_CUSTOMER'
                        target_location = end_node  # put to duckietarget location
                        closest_request['duckie_id'] = duckie  # assign duckie_id to request

                    # going to customer pickup c
                    elif start_node != current_node:
                        duckie['status'] = 'GOING_TO_COSTUMER'
                        target_location = start_node  # put to duckietarget location
                        closest_request['duckie_id'] = duckie  # TODO CHECK assign duckie_id to request

                    # REBALANCE no more requests
                    else:
                        duckie['status'] = 'IDLE'
                        target_location = end_node  # stay

                # WITH CUSTOMER
                elif duckie_status == 'WITH_COSTUMER':

                    # endlocation reached, request fullfilled
                    request = requests['duckie_id'](duckie)  # todo correct
                    if current_node == request['end_loc']:
                        duckie['status'] = 'IDLE'
                        target_location = current_node  # stay

                # GOING TO CUSTOMER
                elif duckie_status == 'GOING_TO_CUSTOMER':

                    # pick up customer
                    request = requests['duckie_id'](duckie)  # todo correct
                    if current_node == request['start_loc']:
                        # change status AND assign open_request
                        duckie['status'] = 'WITH_CUSTOMER'
                        target_location = end_node  # put to duckietarget location
                        closest_request['duckie_id'] = duckie  # assign duckie_id to request

                # generate path and assign
                path_pair = self.generatePathPair(duckie, current_node, target_location)
                paths.append(path_pair)
        else:
            for duckie in duckies:
                paths.append(self.generatePathPair(duckie, self.node(duckie['lane']),
                                                   self.node(duckie['lane'])))

        # delete open_requests list
        del open_requests

        # generateCommands from path
        # EXAMPLE FOR PATHS:
        # paths = [
        #   {
        #       'duckie_id': 'duckie-0',
        #       'path': [list of nodes]
        #   },
        #   {
        #       'duckie_id': 'duckie-1',
        #       'path': [list of nodes]
        #   }, ...
        # ]

        self.commands = self.generateCommands(paths)

    def generateCommands(self, paths):
        commands = []
        for path in paths:
            command = {
                'duckie_id': path['duckie_id'],
                'goal_node': path['path'][0]
            }
            self.commands.append(command)
        return commands

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
        return nx.dijkstra_path_length(self.skeleton_graph, node,
                                       request['start_node'])

    # generate dijkstra_path
    def generatePathPair(self, duckie, current_node, goal_node):
        path = {}
        path['duckie_id'] = duckie
        if current_node == goal_node:
            path['path'] = []
            return path
        path['path'] = nx.dijkstra_path(self.skeleton_graph, current_node, goal_node)
        return path

    def node(self, lane):
        # Get end node of lane
        edges = list(self.skeleton_graph.G.edges(data=True))
        edge_data = [edge for edge in edges if edge[2]['lane'] == lane]
        return edge_data[0][1]
