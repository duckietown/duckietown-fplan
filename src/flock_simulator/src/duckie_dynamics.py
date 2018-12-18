import random
import utils
import numpy as np
import traffic_rules as tr
import duckietown_world as dw
from duckiebot import Duckiebot


def getCommandFromPoints(duckies, duckie, stop_distance, duckiebot_length,
                         max_vel, dt_map, dt):
    # If no next_point, stand still
    if not duckie.next_point:
        return {'linear': 0, 'angular': 0, 'on_rails': False}

    duckie_pose = duckie.pose
    point_pose = duckie.next_point['pose']
    ang_diff = utils.limitAngle(point_pose.theta - duckie_pose.theta)

    linear = tr.getVelocity(duckies, duckie, stop_distance, duckiebot_length,
                            max_vel, dt_map)

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


def getNextPoint(goal_node, dt_map, current_pose, current_point):
    lane_control_points = dt_map.lanes[current_point['lane']].control_points
    current_point_pose = current_point['pose']

    # If current_point in front, keep it as next_point
    if isInFront(current_pose, current_point_pose, np.pi):
        return current_point

    # If current lane has more control points, set next one as next_point
    if len(lane_control_points) > current_point['point_index'] + 1:
        return {
            'lane': current_point['lane'],
            'point_index': current_point['point_index'] + 1,
            'pose': lane_control_points[current_point['point_index'] + 1]
        }

    # Find random next lane
    nodes = dt_map.laneToNodes(current_point['lane'])
    node_next = goal_node if goal_node is not None else random.choice(
        list(dt_map.graph.neighbors(nodes[1])))
    lane_next = dt_map.graph.get_edge_data(nodes[1], node_next)[0]['lane']
    pose_next = dt_map.lanes[lane_next].control_points[0]

    return {'lane': lane_next, 'point_index': 0, 'pose': pose_next}


def updateDuckie(duckies, duckie_id, command, stop_distance, duckiebot_length,
                 max_vel, dt_map, dt):
    duckie = duckies[duckie_id]
    if command is None or command['on_rails']:
        command = getCommandFromPoints(duckies, duckie, stop_distance,
                                       duckiebot_length, max_vel, dt_map, dt)

    pose_new = commandToPose(duckie.pose, command, dt_map.tile_size, dt)
    velocity_new = {'linear': command['linear'], 'angular': command['angular']}
    if duckie.next_point and 'goal_node' in command:
        next_point_new = getNextPoint(command['goal_node'], dt_map, pose_new,
                                      duckie.next_point)
    else:
        next_point_new = getNextPoint(None, dt_map, pose_new,
                                      duckie.next_point)

    duckie.pose = pose_new
    duckie.velocity = velocity_new
    duckie.next_point = next_point_new


def spawnDuckies(n, dt_map):
    duckies = {}
    occupied_lanes = []
    for i in range(n):
        spawn_is_occupied = True
        while spawn_is_occupied:
            lane = random.sample(dt_map.lanes, 1)[0]
            if lane not in occupied_lanes:
                duckie_id = 'duckie-%d' % i
                duckies[duckie_id] = Duckiebot(duckie_id, lane, dt_map)
                spawn_is_occupied = False
                occupied_lanes.append(lane)
    return duckies


def commandToPose(pose, command, tile_size, dt):
    v = command['linear'] / tile_size
    omega = command['angular']

    theta_new = utils.limitAngle(pose.theta + omega * dt)

    theta_avg = pose.theta + utils.subtractAngle(theta_new, pose.theta) / 2

    position_new = [
        pose.p[0] + v * np.cos(theta_avg) * dt,
        pose.p[1] + v * np.sin(theta_avg) * dt
    ]

    pose_new = dw.SE2Transform(position_new, theta_new)

    # If duckiebot on rails, put on rails
    if command['on_rails']:
        pose_new = putOnRails(pose_new, v, omega, dt)

    return pose_new


def putOnRails(pose, v, omega, dt):
    # Fixes deviations from the ideal path.
    # Assumes duckiebot is turning when angular velocity != 0 and places
    # duckiebot on curve depending on current orientation

    d_dist = v * dt
    d_angle = v / 0.28 * dt

    if omega == 0:
        if abs(pose.theta) < d_angle / 2:
            theta = 0.0
            position = [pose.p[0], np.floor(pose.p[1]) + 0.28]
        elif abs(utils.limitAngle(pose.theta - np.pi)) < d_angle / 2:
            theta = -np.pi
            position = [pose.p[0], np.floor(pose.p[1]) + 0.72]
        elif abs(pose.theta - np.pi / 2) < d_angle / 2:
            theta = np.pi / 2
            position = [np.floor(pose.p[0]) + 0.72, pose.p[1]]
        elif abs(pose.theta + np.pi / 2) < d_angle / 2:
            theta = -np.pi / 2
            position = [np.floor(pose.p[0]) + 0.28, pose.p[1]]
        else:
            theta = pose.theta
            position = pose.p
    elif omega > 0:
        radius = 0.72
        midpoint = [
            np.round(pose.p[0] + np.cos(pose.theta + np.pi / 2) * radius),
            np.round(pose.p[1] + np.sin(pose.theta + np.pi / 2) * radius),
        ]
        position = [
            midpoint[0] + np.sin(pose.theta) * radius,
            midpoint[1] - np.cos(pose.theta) * radius
        ]
        theta = pose.theta
    elif omega < 0:
        radius = 0.28
        midpoint = [
            np.round(pose.p[0] + np.cos(pose.theta - np.pi / 2) * radius),
            np.round(pose.p[1] + np.sin(pose.theta - np.pi / 2) * radius),
        ]
        position = [
            midpoint[0] - np.sin(pose.theta) * radius,
            midpoint[1] + np.cos(pose.theta) * radius
        ]
        theta = pose.theta

    pose_new = dw.SE2Transform(position, theta)

    if utils.distance(pose, pose_new) > 4 * d_dist:
        pose_new = pose
        print('Not able to put duckie on rails.')

    return pose_new


def isInFront(pose1, pose2, angle):
    # frame1: Own pose
    # frame2: Object's pose
    # angle: Angle of cone that defines "front" ("field of view")
    return abs(
        utils.subtractAngle(
            np.arctan2(pose2.p[1] - pose1.p[1], pose2.p[0] - pose1.p[0]),
            pose1.theta)) < angle / 2


def lane_distance(duckies, duckie_id, dt_map):
    # Find the nearest control point and send the relative distance along x axis
    duckie = duckies[duckie_id]
    lane_control_points = dt_map.lanes[duckie.next_point[
        'lane']].control_points
    lane_width = dt_map.lanes[duckie.next_point['lane']].width
    min_dist = 10000

    for cp in lane_control_points:
        dist = cp.p[0] - duckie.pose.p[0]
        if dist < min_dist:
            min_dist = dist

    return min_dist > lane_width / 2
