import random
import utils
import numpy as np
import traffic_rules as tr
import duckietown_world as dw


class Duckiebot(object):
    def __init__(self, id, lane=None, dt_map=None):
        self.fov = [2.0 / 3.0 * np.pi,
                    2.0]  # Field of view (angle, distance in tiles)
        self.max_vel = 0.5  # Max. velocity in m/s
        self.stop_distance = 0.1  # Distance between duckies in m
        self.length = 0.2  # Length (diameter) of duckiebot in m

        self.id = id
        self.pose = dw.SE2Transform([0.0, 0.0], 0.0)
        self.velocity = {'linear': 0.0, 'angular': 0.0}
        self.next_point = {
            'lane': None,
            'point_index': 0,
            'pose': dw.SE2Transform([0.0, 0.0], 0.0)
        }
        self.path = []
        self.status = 'IDLE'
        self.in_fov = []
        self.collision_level = 0
        if lane and dt_map:
            self.initialize(lane, dt_map)

    def update(self, duckies, command, dt_map, dt):
        if command is None or command['on_rails']:
            command = self.getCommandFromPoints(duckies, dt_map, dt)

        self.giveCommand(command, dt_map.tile_size, dt)

        path_command = command['path'] if 'path' in command else None
        self.updateNextPoint(path_command, dt_map)

    def initialize(self, point, dt_map):
        # Pose
        self.pose = point['pose']

        # Path
        nodes = dt_map.laneToNodes(point['lane'])
        self.path = dt_map.getRandomPath(nodes[1])

        # Next point
        self.next_point = point
        self.updateNextPoint(None, dt_map)

        print('Initialized %s.' % self.id)

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

    def updateNextPoint(self, new_path, dt_map):
        # Check new_path
        if new_path:
            # First node has to be same
            if new_path[0] != self.path[0]:
                print('Path from command for %s invalid!' % self.id)
            else:
                self.path = new_path

        lane_control_points = dt_map.lanes[self.next_point[
            'lane']].control_points
        current_point_lane = self.next_point['lane']
        current_point_index = self.next_point['point_index']
        current_point_pose = self.next_point['pose']

        # If current_point in front, keep it as next_point
        if utils.isInFront(self.pose, current_point_pose, np.pi):
            return

        # If current lane has more control points, set next one as next_point
        if len(lane_control_points) > current_point_index + 1:
            self.next_point = {
                'lane': current_point_lane,
                'point_index': current_point_index + 1,
                'pose': lane_control_points[current_point_index + 1]
            }
            return

        # Generate random new path if path has only one node left
        if len(self.path) == 1:
            self.path = dt_map.getRandomPath(self.path[0])

        next_lane = dt_map.nodesToLane([self.path[0], self.path[1]])
        self.next_point = {
            'lane': next_lane,
            'point_index': 1,
            'pose': dt_map.lanes[next_lane].control_points[1]
        }
        del self.path[0]

    def updateFov(self, duckies):
        self.in_fov[:] = []
        for duckie in duckies.values():
            if duckie.id == self.id:
                continue
            if utils.distance(self.pose,
                              duckie.pose) < self.fov[1] and utils.isInFront(
                                  self.pose, duckie.pose, self.fov[0]):
                self.in_fov.append(duckie.id)

    def updateCollision(self, duckies, tile_size):
        for duckie in duckies.values():
            if duckie.id == self.id or self.collision_level == 1:
                continue
            if utils.distance(
                    self.pose, duckie.pose
            ) * tile_size < self.length / 2 + duckie.length / 2:
                self.collision_level = 1
                self.max_vel = 0
                print('%s collided!' % self.id)

    def getCommandFromPoints(self, duckies, dt_map, dt):
        # If no next_point, stand still
        if not self.next_point or not utils.isInFront(
                self.pose, self.next_point['pose'], np.pi):
            print('None or invalid point to follow for %s' % self.id)
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

    def reachedNode(self, node, dt_map):
        pose = dt_map.nodeToPose(node)
        return utils.distance(self.pose, pose) < self.length / 2