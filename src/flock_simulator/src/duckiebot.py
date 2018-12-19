import random
import utils
import numpy as np
import traffic_rules as tr
import duckietown_world as dw


class Duckiebot(object):
    def __init__(self, id, lane=None, dt_map=None):
        self.length = 0.2
        self.max_vel = 0.5
        self.stop_distance = 0.1

        self.id = id
        self.pose = dw.SE2Transform([0.0, 0.0], 0.0)
        self.velocity = {'linear': 0.0, 'angular': 0.0}
        self.next_point = {
            'lane': None,
            'point_index': 0,
            'pose': dw.SE2Transform([0.0, 0.0], 0.0)
        }
        self.status = 'IDLE'
        self.in_fov = []
        self.collision_level = 0
        if lane and dt_map:
            self.initialize(lane, dt_map)

    def update(self, duckies, command, dt_map, dt):
        if command is None or command['on_rails']:
            command = self.getCommandFromPoints(duckies, dt_map, dt)

        self.giveCommand(command, dt_map.tile_size, dt)

        if self.next_point and 'goal_node' in command:
            self.next_point = utils.getNextPoint(command['goal_node'], dt_map,
                                                 self.pose, self.next_point)
        else:
            self.next_point = utils.getNextPoint(None, dt_map, self.pose,
                                                 self.next_point)

    def initialize(self, lane, dt_map):
        point_index = random.choice(
            range(len(dt_map.lanes[lane].control_points)))
        pose = dt_map.lanes[lane].control_points[point_index]
        nodes = dt_map.laneToNodes(lane)
        node_next = random.choice(list(dt_map.graph.neighbors(nodes[1])))
        lane_next = dt_map.graph.get_edge_data(nodes[1], node_next)[0]['lane']
        pose_next = dt_map.lanes[lane_next].control_points[0]

        self.pose = pose
        self.next_point = {
            'lane': lane_next,
            'point_index': 0,
            'pose': pose_next
        }

    def giveCommand(self, command, tile_size, dt):
        v = command['linear'] / tile_size
        omega = command['angular']

        theta_new = utils.limitAngle(self.pose.theta + omega * dt)

        theta_avg = self.pose.theta + utils.subtractAngle(
            theta_new, self.pose.theta) / 2

        position_new = [
            self.pose.p[0] + v * np.cos(theta_avg) * dt,
            self.pose.p[1] + v * np.sin(theta_avg) * dt
        ]

        self.pose = dw.SE2Transform(position_new, theta_new)

        # If duckiebot on rails, put on rails
        if command['on_rails']:
            self.putOnRails(v, omega, dt)

        self.velocity = {
            'linear': command['linear'],
            'angular': command['angular']
        }

    def getCommandFromPoints(self, duckies, dt_map, dt):
        # If no next_point, stand still
        if not self.next_point:
            return {'linear': 0, 'angular': 0, 'on_rails': False}

        point_pose = self.next_point['pose']
        ang_diff = utils.limitAngle(point_pose.theta - self.pose.theta)

        linear = tr.getVelocity(duckies, self, self.stop_distance, self.length,
                                self.max_vel, dt_map)

        d_angle = linear / 0.28 * dt

        if ang_diff > d_angle / 2:
            radius = 0.72 * dt_map.tile_size
            angular = linear / radius
        elif ang_diff < -d_angle / 2:
            radius = 0.28 * dt_map.tile_size
            angular = -linear / radius
        else:
            angular = 0.0

        return {'linear': linear, 'angular': angular, 'on_rails': True}

    def putOnRails(self, v, omega, dt):
        # Fixes deviations from the i   deal path.
        # Assumes duckiebot is turning when angular velocity != 0 and places
        # duckiebot on curve depending on current orientation

        d_dist = v * dt
        d_angle = v / 0.28 * dt

        if omega == 0:
            if abs(self.pose.theta) < d_angle / 2:
                theta = 0.0
                position = [self.pose.p[0], np.floor(self.pose.p[1]) + 0.28]
            elif abs(utils.limitAngle(self.pose.theta - np.pi)) < d_angle / 2:
                theta = -np.pi
                position = [self.pose.p[0], np.floor(self.pose.p[1]) + 0.72]
            elif abs(self.pose.theta - np.pi / 2) < d_angle / 2:
                theta = np.pi / 2
                position = [np.floor(self.pose.p[0]) + 0.72, self.pose.p[1]]
            elif abs(self.pose.theta + np.pi / 2) < d_angle / 2:
                theta = -np.pi / 2
                position = [np.floor(self.pose.p[0]) + 0.28, self.pose.p[1]]
            else:
                theta = self.pose.theta
                position = self.pose.p
        elif omega > 0:
            radius = 0.72
            midpoint = [
                np.round(self.pose.p[0] +
                         np.cos(self.pose.theta + np.pi / 2) * radius),
                np.round(self.pose.p[1] +
                         np.sin(self.pose.theta + np.pi / 2) * radius),
            ]
            position = [
                midpoint[0] + np.sin(self.pose.theta) * radius,
                midpoint[1] - np.cos(self.pose.theta) * radius
            ]
            theta = self.pose.theta
        elif omega < 0:
            radius = 0.28
            midpoint = [
                np.round(self.pose.p[0] +
                         np.cos(self.pose.theta - np.pi / 2) * radius),
                np.round(self.pose.p[1] +
                         np.sin(self.pose.theta - np.pi / 2) * radius),
            ]
            position = [
                midpoint[0] - np.sin(self.pose.theta) * radius,
                midpoint[1] + np.cos(self.pose.theta) * radius
            ]
            theta = self.pose.theta

        pose_new = dw.SE2Transform(position, theta)

        if utils.distance(self.pose, pose_new) > 4 * d_dist:
            print('Not able to put duckie on rails.')
            return

        self.pose = pose_new