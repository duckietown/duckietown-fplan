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
        self.sim_frequency = 20.0  # Frequency of simulation in Hz
        self.dt = 1.0 / self.sim_frequency
        self.n_duckies = 3  # Number of duckies
        self.max_vel = 1.0 / 3.0  # Tile / s
        self.max_acc = 2.0  # Tile / s^2

        # Map
        self.map = dw.load_map('4way')
        self.map_graph = self.generateMapGraph()

        # State of duckies
        self.duckies = self.spawnDuckies()

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
            vel_des = self.getDesiredVelocity(duckie)
            vel = self.duckies[duckie]['velocity']
            vel_new = vel + np.sign(vel_des - vel) * self.max_acc * self.dt
            traveled = (vel + vel_new) / 2 * self.dt
            self.duckies[duckie]['velocity'] = vel_new
            self.setNewPose(duckie, traveled)

        # Update nodes with duckies
        for node in self.map_graph:
            self.map_graph.nodes[node]['duckies'][:] = []
        for duckie in self.duckies:
            pose = self.duckies[duckie]['pose']
            coord = np.around([pose['x'], pose['y']]).tolist()
            self.map_graph.nodes['tile-%d-%d' %
                                 (coord[0],
                                  coord[1])]['duckies'].append(duckie)

    def getNewPose(self, pose, distance, tile, action, heading):
        if action == 'straight':
            pose_new = {
                'x': pose['x'] + heading[0] * distance,
                'y': pose['y'] + heading[1] * distance,
                'phi': pose['phi']
            }
        elif action == 'curve_left':
            phi_new = np.unwrap(pose['phi'] + distance / 0.75)
            center = {
                '[0, 1]': [tile[0] - 0.5, tile[1] + 0.5],
                '[1, 0]': [tile[0] + 0.5, tile[1] + 0.5],
                '[0, -1]': [tile[0] + 0.5, tile[1] - 0.5],
                '[-1, 0]': [tile[0] - 0.5, tile[1] - 0.5]
            }
            pose_new = {
                'x': center[str(heading)][0] + 0.75 * np.cos(phi_new),
                'y': center[str(heading)][1] + 0.75 * np.sin(phi_new),
                'phi': phi_new
            }
        elif action == 'curve_right':
            phi_new = np.unwrap(pose['phi'] - distance / 0.25)
            center = {
                '[0, 1]': [tile[0] + 0.5, tile[1] + 0.5],
                '[1, 0]': [tile[0] + 0.5, tile[1] - 0.5],
                '[0, -1]': [tile[0] - 0.5, tile[1] - 0.5],
                '[-1, 0]': [tile[0] - 0.5, tile[1] + 0.5]
            }
            pose_new = {
                'x': center[str(heading)][0] + 0.25 * np.cos(phi_new),
                'y': center[str(heading)][1] + 0.25 * np.sin(phi_new),
                'phi': phi_new
            }
        return pose_new

    def setNewPose(self, duckie, distance):
        pose = self.duckies[duckie]['pose']
        action = self.duckies[duckie]['action']
        heading = self.duckies[duckie]['heading']
        tile = np.around([pose['x'], pose['y']]).tolist()
        pose_new = self.getNewPose(pose, distance, tile, action, heading)
        tile_new = np.around([pose_new['x'], pose_new['y']]).tolist()

        if tile_new == tile:
            self.duckies[duckie]['pose'] = pose_new
            return

        heading_new = self.getNewHeading(self.paths[duckie])
        action_new = self.getNewAction(heading, heading_new)

        phi_trans = {
            '[0, 1]': 0,
            '[1, 0]': -1.0 / 2.0 * np.pi,
            '[0, -1]': np.pi,
            '[-1, 0]': 1.0 / 2.0 * np.pi
        }

        pose_trans = {
            'x': tile[0] + heading[0] * 0.5,
            'y': tile[1] + heading[1] * 0.5,
            'phi': phi_trans[str(heading)]
        }

        if action == 'straight':
            distance_over = (pose_new['x'] - pose_trans['x']) + (
                pose_new['y'] - pose_trans['y'])
        elif action == 'curve_left':
            distance_over = np.abs(
                pose_new['phi'] - phi_trans[str(heading)]) * 0.75
        else:
            distance_over = np.abs(
                pose_new['phi'] - phi_trans[str(heading)]) * 0.25

        pose_new = self.getNewPose(pose_trans, distance - distance_over,
                                   tile_new, action_new, heading_new)

        self.duckies[duckie]['pose'] = pose_new
        self.duckies[duckie]['action'] = action_new
        self.duckies[duckie]['heading'] = heading_new

    def getDesiredVelocity(self, duckie):
        pose = self.duckies[duckie]['pose']
        tile = np.around([pose['x'], pose['y']]).tolist()
        heading = self.duckies[duckie]['heading']

        # Check for duckies in front
        current_tile = 'tile-%d-%d' % (tile[0], tile[1])
        front_tile = 'tile-%d-%d' % (tile[0] + heading[0],
                                     tile[1] + heading[1])
        current_tile_duckies = self.map_graph.node[current_tile]['duckies']
        front_tile_duckies = self.map_graph.node[front_tile]['duckies']
        both_tiles_duckies = current_tile_duckies + front_tile_duckies
        if both_tiles_duckies:
            for tile_duckie in both_tiles_duckies:
                diff = [
                    self.duckies[tile_duckie]['pose']['x'] - pose['x'],
                    self.duckies[tile_duckie]['pose']['y'] - pose['y']
                ]
                distance_in_front = heading[0] * diff[0] + heading[1] * diff[1]
                if np.linalg.norm(diff) < 0.5 and distance_in_front > 0:
                    return 0

        # Check for right of way
        on_intersection = self.map_graph.nodes[current_tile][
            'type'] == 'intersection'
        next_tile_intersection = len(
            self.map_graph.nodes[front_tile]['type']) == 'intersection'
        if on_intersection or not next_tile_intersection or not front_tile_duckies:
            return self.max_vel

        heading_next = self.getNewHeading(self.paths[duckie])
        action_next = self.getNewAction(heading, heading_next)
        # For right curves, only check for duckies on intersection heading same direction
        if action_next == 'curve_right':
            for tile_duckie in front_tile_duckies:
                if tile_duckie['heading'] == heading_next:
                    return 0
        # Check for duckies on intersection heading to your right or straight
        right_heading = [heading[1], -heading[0]]
        for tile_duckie in front_tile_duckies:
            if tile_duckie['heading'] == heading or tile_duckie['heading'] == right_heading:
                return 0
        # Check for duckies on the right
        right_coord = [
            tile[0] + heading[0] + right_heading[0],
            tile[1] + heading[1] + right_heading[1]
        ]
        right_tile = 'tile-%d-%d' % (right_coord[0], right_coord[1])
        right_tile_duckies = self.map_graph.node[right_tile]['duckies']
        if right_tile_duckies:
            right_duckie_heading = [-right_heading[0], -right_heading[1]]
            for right_duckie in right_tile_duckies:
                diff_from_mid = [
                    right_duckie['pose']['x'] - right_coord[0],
                    right_duckie['pose']['y'] - right_coord[1]
                ]
                dist_from_mid = right_duckie_heading[0] * diff_from_mid[0] + right_duckie_heading[1] * diff_from_mid[1]
                if right_duckie['heading'] == right_duckie_heading and diff_from_mid > 0:
                    return 0

        return self.max_vel

    def getNewHeading(self, path):
        return [b - a for a, b in zip(path[0], path[1])]

    def getNewAction(self, heading, heading_new):
        heading_cross = np.cross(heading.append(0), heading_new.append(0))
        if heading_cross[2] == 0:
            return 'straight'
        elif heading_cross[2] > 0:
            return 'curve_left'
        else:
            return 'curve_right'

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

    def spawnDuckies(self):
        duckies = {}
        offset = {
            '[0, 1]': [0.25, 0, 0],
            '[1, 0]': [0, -0.25, -1.0 / 2.0 * np.pi],
            '[0, -1]': [-0.25, 0, np.pi],
            '[-1, 0]': [0, 0.25, 1.0 / 2.0 * np.pi]
        }
        for i in range(self.n_duckies):
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
                    offset[str(direction)][0],
                    'y':
                    self.map_graph.nodes[spawn_node]['coord'][1] +
                    offset[str(direction)][1],
                    'phi':
                    offset[str(direction)][2]
                }
                coord = [pose['x'], pose['y']]
                spawnIsValid = True
                for duckie in duckies:
                    duckie_coord = [
                        duckies[duckie]['pose']['x'],
                        duckies[duckie]['pose']['y']
                    ]
                    if np.linalg.norm(
                            np.asarray(duckie_coord) - np.asarray(coord)) < 1:
                        spawnIsValid = False
            duckies['duckie-%d' % i] = {
                'pose': pose,
                'velocity': 0,
                'action': 'straight',
                'heading': direction
            }
            self.map_graph.nodes[spawn_node]['duckies'].append('duckie-%d' % i)
        return duckies

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
