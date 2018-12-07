#!/usr/bin/env python

import duckietown_world as dw
import networkx as nx
from flock_simulator.msg import FlockState, FlockCommand


# TODO Implement duckie initial(t=0) status = 'IDLE'

class Dispatcher(object):
    def __init__(self, map_name):
        # Commands
        self.commands = []

        # Load Grap
        self.map = dw.load_map(map_name)
        self.skeleton_graph = dw.get_skeleton_graph(self.map['tilemap'])

    def update(self, state):
        return  # TODO Remove once implemented
        duckies = state.duckies
        open_requests = state.open_requests
        paths = []

        for duckie_id in duckies:
            pose = duckies[duckie_id]['pose']
            target_location = duckies[duckie_id]['target_location']  # TODO ADD TO DUCKIESTATUS

            # IDLE
            if duckies[duckie_id]['status'] == 'IDLE':

                # find closest costumer
                open_request = self.getClosestRequest(self, open_requests, pose)
                start_loc = open_request['startlocation']
                end_loc = open_request['endlocation']

                # pick up costumer
                if start_loc == pose:
                    # change status AND delete open_request
                    duckies[duckie_id]['status'] = 'WITH_COSTUMER'
                    target_location = end_loc  # put to duckietarget location
                    open_requests.remove(open_request)

                # going to costumer pickup costumer
                elif start_loc != pose:
                    duckies[duckie_id]['status'] = 'GOINGTO_COSTUMER'
                    target_location = start_loc  # put to duckietarget location

                # no requests + REBALANCE
                else:
                    duckies[duckie_id]['status'] = 'IDLE'
                    target_location = pose  # stay

            # WITH COSTUMER
            elif duckies[duckie_id]['status'] == 'WITH_COSTUMER':

                # endlocation reached, request fullfilled
                if pose == target_location:
                    duckies[duckie_id]['status'] = 'IDLE'
                    target_location = pose  # stay

            # generate path
            paths[duckie_id]= self.generatePath(pose, target_location)

        # generateCommands from path
        return self.generateCommands(paths)

    def generateCommands(self, paths):
        # TODO generate commands from path # maybe implement in external file // no userfile //
        # ask cliff how commands are built
        return self.commands

    def getClosestRequest(self, open_requests, pose):
        if not open_requests:  # if no requests
            return
        closest_request = open_requests[0]
        for open_request in open_requests:
            if self.dist(pose, open_request) < self.dist(pose, closest_request):
                closest_request = open_request

        return closest_request

    def dist(self, pose, request):
        # generate dijkstra_distance (closest)
        return nx.dijkstra_path_length(self.skeleton_graph,
                                       self.node(pose),
                                       self.node(request['start_loc']))

    def generatePath(self, current_pose, target_location):
        # generate dijkstra_path
        return nx.dijkstra_path(self.skeleton_graph,
                               self.node(current_pose),
                               self.node(target_location))

    def node(self, pose):
        node = 0 #TODO
        return node
