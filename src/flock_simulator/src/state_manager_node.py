#!/usr/bin/env python

import rospy

import networkx as nx
import duckietown_world as dw
from std_msgs.msg import String


class StateManagerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters
        self.sim_frequency = 20  # Frequency of simulation in Hz

        # Map
        self.map = dw.load_map('4way')
        self.map_graph = self.generateMapGraph(self.map)

        # Subscribers
        self.sub_requests = rospy.Subscriber(
            "~requests", String, self.cbRequests, queue_size=1)
        self.sub_locations = rospy.Subscriber(
            "~locations", String, self.cbLocations, queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher("~state", String, queue_size=1)

        # Timer
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.sim_frequency), self.cbTimer)

    def updateState(self, event):
        # Update state

        # Publish
        msg_state = String()
        self.pub_state.publish(msg_state)

    def generateMapGraph(self, map):
        tilemap = map.children['tilemap']
        map_size = [tilemap.H, tilemap.W]
        graph = nx.Graph()
        for name, tile in tilemap.children.items():
            str_parse = name.split("-")
            if str_parse[0] != "tile" or not tile.drivable:
                continue
            graph.add_node(name, coord=[str_parse[1], str_parse[2]])
        for x in range(map_size[0]):
            for y in range(map_size[1]):
                current_tile = "tile-%d-%d" % (x, y)
                right_tile = "tile-%d-%d" % (x, y + 1)
                bottom_tile = "tile-%d-%d" % (x + 1, y)
                if current_tile not in graph:
                    continue
                if right_tile in graph:
                    graph.add_edge(current_tile, right_tile)
                if bottom_tile in graph:
                    graph.add_edge(current_tile, bottom_tile)
        return graph

    def cbRequests(self, data):
        # Do something with requests
        print(data)

    def cbLocations(self, data):
        # Do something with locations
        print(data)

    def cbTimer(self, event):
        self.updateState

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node("state_manager_node", anonymous=False)
    state_manager_node = StateManagerNode()
    rospy.on_shutdown(state_manager_node.onShutdown)
    rospy.spin()
