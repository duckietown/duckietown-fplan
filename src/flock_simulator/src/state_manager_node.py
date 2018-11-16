#!/usr/bin/env python

import rospy
import random
import numpy as np
import networkx as nx
import duckietown_world as dw
from std_msgs.msg import String


class StateManagerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters
        self.sim_frequency = 20  # Frequency of simulation in Hz
        self.n_duckies = 3  # Number of duckies

        # Map
        self.map = dw.load_map('4way')
        self.map_graph = self.generateMapGraph()

        # Position of duckies
        self.duckies_pose = self.spawnDuckies()

        # Subscribers
        self.sub_requests = rospy.Subscriber(
            '~requests', String, self.cbRequests, queue_size=1)
        self.sub_locations = rospy.Subscriber(
            '~locations', String, self.cbLocations, queue_size=1)
        self.sub_paths = rospy.Subscriber(
            '~paths', String, self.cbPaths, queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher('~state', String, queue_size=1)
        self.pub_commands = rospy.Publisher('~commands', String, queue_size=1)

        # Timer
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.sim_frequency), self.cbTimer)

    def updateState(self, event):
        # Update state

        # Publish
        msg_state = String()
        self.pub_state.publish(msg_state)

    def generateMapGraph(self):
        tilemap = self.map.children['tilemap']
        map_size = [tilemap.H, tilemap.W]
        graph = nx.Graph()

        # Add one node per drivable tile
        for name, tile in tilemap.children.items():
            str_parse = name.split('-')
            if str_parse[0] != 'tile' or not tile.drivable:
                continue
            graph.add_node(name, coord=[int(str_parse[1]), int(str_parse[2])])

        # Add edges to adjacent tiles (does not consider type of tile!)
        for x in range(map_size[0]):
            for y in range(map_size[1]):
                current_tile = 'tile-%d-%d' % (x, y)
                right_tile = 'tile-%d-%d' % (x, y + 1)
                bottom_tile = 'tile-%d-%d' % (x + 1, y)
                if current_tile not in graph:
                    continue
                if right_tile in graph:
                    graph.add_edge(current_tile, right_tile)
                if bottom_tile in graph:
                    graph.add_edge(current_tile, bottom_tile)

        # Find exits and type of tile
        directions = {
            '[1, 0]': 'N',
            '[0, 1]': 'E',
            '[-1, 0]': 'S',
            '[0, -1]': 'W'
        }
        for node in graph:
            node_coords = graph.nodes[node]['coord']
            exits = []
            for neighbor in graph.neighbors(node):
                neighbor_coords = graph.nodes[neighbor]['coord']
                diff = [b - a for a, b in zip(node_coords, neighbor_coords)]
                exits.append(directions[str(diff)])
            if len(exits) > 2:
                graph.nodes[node]['type'] = 'intersection'
            elif all(i in ['N', 'S']
                     for i in exits) or all(i in ['E', 'W'] for i in exits):
                graph.nodes[node]['type'] = 'straight'
            else:
                graph.nodes[node]['type'] = 'curve'
            graph.nodes[node]['exits'] = exits
        return graph

    def spawnDuckies(self):
        poses = {}
        for i in range(self.n_duckies):
            offset = {
                'N': [0.25, 0, 0],
                'E': [0, -0.25, -np.pi / 2],
                'S': [-0.25, 0, np.pi],
                'W': [0, 0.25, np.pi / 2]
            }
            spawnIsValid = False
            while not spawnIsValid:
                spawn_node = random.choice([
                    node for node, data in self.map_graph.nodes(data=True)
                    if data['type'] == 'straight'
                ])
                direction = random.choice(
                    self.map_graph.nodes[spawn_node]['exits'])
                pose = {
                    'x':
                    self.map_graph.nodes[spawn_node]['coord'][0] +
                    offset[direction][0],
                    'y':
                    self.map_graph.nodes[spawn_node]['coord'][1] +
                    offset[direction][1],
                    'phi':
                    offset[direction][2]
                }
                coord = [pose['x'], pose['y']]
                spawnIsValid = True
                for duckie in poses:
                    duckie_coord = [poses[duckie]['x'], poses[duckie]['y']]
                    if np.linalg.norm(
                            np.asarray(duckie_coord) - np.asarray(coord)) < 1:
                        spawnIsValid = False
            poses['duckie-%d' % i] = pose
        print(poses)
        return poses

    def cbPaths(self, data):
        pass

    def cbRequests(self, data):
        # Do something with requests
        print(data)

    def cbLocations(self, data):
        # Do something with locations
        print(data)

    def cbTimer(self, event):
        self.updateState

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('state_manager_node', anonymous=False)
    state_manager_node = StateManagerNode()
    rospy.on_shutdown(state_manager_node.onShutdown)
    rospy.spin()
