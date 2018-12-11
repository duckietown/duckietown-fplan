#!/usr/bin/env python

import duckietown_world as dw
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
        #   'requests: [
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

        for duckie_id in duckies:
            duckie = duckies[duckie_id]
            lane = duckie['lane']  # Lane the duckie is currently on
            node = self.node(lane)  # Node the duckie is heading to

            # TODO

            # # IDLE
            # if duckie['status'] == 'IDLE':

            #     # find closest costumer
            #     open_request = self.getClosestRequest(requests, pose)
            #     start_loc = open_request['startlocation']
            #     end_loc = open_request['endlocation']

            #     # pick up costumer
            #     if start_loc == pose:
            #         # change status AND delete open_request
            #         duckie['status'] = 'WITH_COSTUMER'
            #         target_location = end_loc  # put to duckietarget location
            #         requests.remove(open_request)

            #     # going to costumer pickup costumer
            #     elif start_loc != pose:
            #         duckie['status'] = 'GOINGTO_COSTUMER'
            #         target_location = start_loc  # put to duckietarget location

            #     # no requests + REBALANCE
            #     else:
            #         duckie['status'] = 'IDLE'
            #         target_location = pose  # stay

            # # WITH COSTUMER
            # elif duckie['status'] == 'WITH_COSTUMER':

            #     # endlocation reached, request fullfilled
            #     if pose == target_location:
            #         duckie['status'] = 'IDLE'
            #         target_location = pose  # stay

            # # generate path
            # path = self.generatePath(pose, target_location)

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

    def getClosestRequest(self, requests, node):
        if not requests:  # if no requests
            return
        closest_request = requests[0]
        for open_request in requests:
            if self.dist(node, open_request) < self.dist(
                    node, closest_request):
                closest_request = open_request

        return closest_request

    def dist(self, node, request):
        # generate dijkstra_distance (closest)
        return nx.dijkstra_path_length(self.skeleton_graph, node,
                                       request['end_node'])

    def generatePath(self, current_node, goal_node):
        # generate dijkstra_path
        return nx.dijkstra_path(self.skeleton_graph, current_node, goal_node)

    def node(self, lane):
        # Get end node of lane
        edges = list(self.skeleton_graph.G.edges(data=True))
        edge_data = [edge for edge in edges if edge[2]['lane'] == lane]
        return edge_data[0][1]
