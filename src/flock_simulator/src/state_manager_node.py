#!/usr/bin/env python

import rospy
import numpy as np
import networkx as nx
import duckietown_world as dw
import duckie_dynamics as dd
from std_msgs.msg import String


class StateManagerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters
        self.sim_frequency = 20.0  # Frequency of simulation in Hz
        self.dt = 1.0 / self.sim_frequency
        self.n_duckies = 3  # Number of duckies
        self.max_vel = 1.0 / 3.0  # Tile / s
        self.max_acc = 2.0  # Tile / s^2

        # Map
        self.map = dw.load_map('4way')
        self.map_graph = self.generateMapGraph()

        # State of duckies
        self.duckies = dd.spawnDuckies(self.n_duckies, self.map_graph)

        # Paths
        self.paths = {}

        # Subscribers
        self.sub_requests = rospy.Subscriber(
            '~requests', String, self.cbRequests, queue_size=1)
        self.sub_paths = rospy.Subscriber(
            '~paths', String, self.cbPaths, queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher('~state', String, queue_size=1)
        self.pub_commands = rospy.Publisher('~commands', String, queue_size=1)

        # Timer
        self.isUpdating = False
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(self.dt), self.cbTimer)

    def updateState(self):
        # Update every duckie
        for duckie in self.duckies:
            vel_des = dd.getDesiredVelocity(self.duckies, self.map_graph.nodes,
                                            self.paths[duckie], self.max_vel,
                                            duckie)
            vel = self.duckies[duckie]['velocity']
            vel_new = vel + np.sign(vel_des - vel) * self.max_acc * self.dt
            traveled = (vel + vel_new) / 2 * self.dt
            self.duckies[duckie]['velocity'] = vel_new
            dd.setNewPose(self.duckies[duckie], self.paths[duckie], traveled)

        # Update nodes with duckies
        for node in self.map_graph:
            self.map_graph.nodes[node]['duckies'][:] = []
        for duckie in self.duckies:
            pose = self.duckies[duckie]['pose']
            coord = np.around([pose['x'], pose['y']]).tolist()
            self.map_graph.nodes['tile-%d-%d' %
                                 (coord[0],
                                  coord[1])]['duckies'].append(duckie)

    def generateMapGraph(self):
        tilemap = self.map.children['tilemap']
        map_size = [tilemap.H, tilemap.W]
        graph = nx.Graph()

        # Add one node per drivable tile
        for name, tile in tilemap.children.items():
            str_parse = name.split('-')
            if str_parse[0] != 'tile' or not tile.drivable:
                continue
            graph.add_node(
                name, coord=[int(str_parse[1]),
                             int(str_parse[2])], duckies=[])

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
        for node in graph:
            node_coords = graph.nodes[node]['coord']
            exits = []
            for neighbor in graph.neighbors(node):
                neighbor_coords = graph.nodes[neighbor]['coord']
                diff = [b - a for a, b in zip(node_coords, neighbor_coords)]
                exits.append(diff)
            if len(exits) > 2:
                graph.nodes[node]['type'] = 'intersection'
            elif [b + a for a, b in zip(exits[0], exits[1])] == [0, 0]:
                graph.nodes[node]['type'] = 'straight'
            else:
                graph.nodes[node]['type'] = 'curve'
            graph.nodes[node]['exits'] = exits
        return graph

    def cbPaths(self, data):
        # Store paths in self.paths
        pass

    def cbRequests(self, data):
        # Do something with requests
        print(data)

    def cbTimer(self, event):
        # Don't update if last timer callback hasn't finished
        if self.isUpdating:
            rospy.logwarn('State not finished updating. Skipping timestep.')
            return

        # Update state
        self.isUpdating = True
        self.updateState()
        self.isUpdating = False

        # Publish
        msg_state = String()
        self.pub_state.publish(msg_state)

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('state_manager_node', anonymous=False)
    state_manager_node = StateManagerNode()
    rospy.on_shutdown(state_manager_node.onShutdown)
    rospy.spin()
