import random
import numpy as np
import duckietown_world as dw


def getRandomCommand(duckies, duckie, stop_distance, max_vel, tile_size, dt):
    # If no next_point, stand still
    if not duckie['next_point']:
        return {'linear': 0, 'angular': 0, 'on_rails': False}

    duckie_pose = duckie['pose']
    point_pose = duckie['next_point']['pose']
    ang_diff = limitAngle(point_pose.theta - duckie_pose.theta)

    linear = max_vel

    d_angle = linear / 0.28 * dt

    if ang_diff > d_angle / 2:
        radius = 0.72
        angular = linear / radius
    elif ang_diff < -d_angle / 2:
        radius = 0.28
        angular = -linear / radius
    else:
        angular = 0.0
        for visible_duckie in duckie['in_fov']:
            dist = distance(duckie['pose'],
                            duckies[visible_duckie]['pose']) * tile_size
            if dist < 2.0 * stop_distance:
                stop_vel = max((dist / stop_distance - 1.0) * max_vel, 0.0)
                linear = min(linear, stop_vel)

    return {'linear': linear, 'angular': angular, 'on_rails': True}


def getNextPoint(skeleton_graph, current_pose, current_point):
    lane_control_points = skeleton_graph.root2.children[current_point[
        'lane']].control_points
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
    edges = list(skeleton_graph.G.edges(data=True))
    edge_data = [
        edge for edge in edges if edge[2]['lane'] == current_point['lane']
    ]
    node = edge_data[0][1]
    node_next = random.choice(list(skeleton_graph.G.neighbors(node)))
    lane_next = skeleton_graph.G.get_edge_data(node, node_next)[0]['lane']
    pose_next = skeleton_graph.root2.children[lane_next].control_points[0]

    return {'lane': lane_next, 'point_index': 0, 'pose': pose_next}


def updateDuckie(duckies, duckie, command, skeleton_graph, tile_size, dt):
    pose_new = commandToPose(duckie['pose'], command, tile_size, dt)
    velocity_new = command
    next_point_new = getNextPoint(
        skeleton_graph, pose_new,
        duckie['next_point']) if duckie['next_point'] else None
    return {
        'pose': pose_new,
        'velocity': velocity_new,
        'next_point': next_point_new,
        'on_service': False
    }


def spawnDuckies(n, skeleton_graph):
    duckies = {}
    occupied_lanes = []
    for i in range(n):
        spawn_is_occupied = True
        while spawn_is_occupied:
            lane = random.sample(skeleton_graph.root2.children, 1)[0]
            if lane not in occupied_lanes:
                point_index = random.choice(
                    range(
                        len(skeleton_graph.root2.children[lane]
                            .control_points)))
                pose = skeleton_graph.root2.children[lane].control_points[
                    point_index]
                next_point = getNextPoint(skeleton_graph, pose, {
                    'lane': lane,
                    'point_index': point_index,
                    'pose': pose
                })
                duckies['duckie-%d' % i] = {
                    'pose': pose,
                    'velocity': {
                        'linear': 0,
                        'angular': 0
                    },
                    'next_point': next_point,
                    'on_service': False,
                    'in_fov': [],
                    'collision_level': 0
                }
                spawn_is_occupied = False
                occupied_lanes.append(lane)
    return duckies


def commandToPose(pose, command, tile_size, dt):
    v = command['linear'] / tile_size
    omega = command['angular']

    theta_new = limitAngle(pose.theta + omega * dt)

    theta_avg = pose.theta + subtractAngle(theta_new, pose.theta) / 2

    position_new = [
        pose.p[0] + v * np.cos(theta_avg) * dt,
        pose.p[1] + v * np.sin(theta_avg) * dt
    ]

    pose_new = dw.SE2Transform(position_new, theta_new)

    # If duckiebot on rails, put on rails
    if command['on_rails']:
        pose_new = putOnRails(pose_new, command, dt)

    return pose_new


def putOnRails(pose, command, dt):
    # Fixes deviations from the ideal path.
    # Assumes duckiebot is turning when angular velocity != 0 and places
    # duckiebot on curve depending on current orientation, which might
    # lead to "teleportation".

    omega = command['angular']
    linear = command['linear']
    d_angle = linear / 0.28 * dt

    if omega == 0:
        if abs(pose.theta) < d_angle / 2:
            theta = 0.0
            position = [pose.p[0], np.floor(pose.p[1]) + 0.28]
        elif abs(limitAngle(pose.theta - np.pi)) < d_angle / 2:
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
        theta = pose.theta
        if pose.theta >= 0 and pose.theta < np.pi / 2:
            position = [
                np.floor(pose.p[0]) + np.sin(theta) * radius,
                np.ceil(pose.p[1]) - np.cos(theta) * radius
            ]
        elif pose.theta >= np.pi / 2 and pose.theta < np.pi:
            position = [
                np.floor(pose.p[0]) + np.sin(theta) * radius,
                np.floor(pose.p[1]) - np.cos(theta) * radius
            ]
        elif pose.theta >= -np.pi and pose.theta < -np.pi / 2:
            position = [
                np.ceil(pose.p[0]) + np.sin(theta) * radius,
                np.floor(pose.p[1]) - np.cos(theta) * radius
            ]
        elif pose.theta >= -np.pi / 2 and pose.theta < 0:
            position = [
                np.ceil(pose.p[0]) + np.sin(theta) * radius,
                np.ceil(pose.p[1]) - np.cos(theta) * radius
            ]
    elif omega < 0:
        radius = 0.28
        theta = pose.theta
        if pose.theta >= 0 and pose.theta < np.pi / 2:
            position = [
                np.ceil(pose.p[0]) - np.sin(theta) * radius,
                np.floor(pose.p[1]) + np.cos(theta) * radius
            ]
        elif pose.theta >= np.pi / 2 and pose.theta < np.pi:
            position = [
                np.ceil(pose.p[0]) - np.sin(theta) * radius,
                np.ceil(pose.p[1]) + np.cos(theta) * radius
            ]
        elif pose.theta >= -np.pi and pose.theta < -np.pi / 2:
            position = [
                np.floor(pose.p[0]) - np.sin(theta) * radius,
                np.ceil(pose.p[1]) + np.cos(theta) * radius
            ]
        elif pose.theta >= -np.pi / 2 and pose.theta < 0:
            position = [
                np.floor(pose.p[0]) - np.sin(theta) * radius,
                np.floor(pose.p[1]) + np.cos(theta) * radius
            ]

    return dw.SE2Transform(position, theta)


def isInFront(pose1, pose2, angle):
    # frame1: Own pose
    # frame2: Object's pose
    # angle: Angle of cone that defines "front" ("field of view")
    return abs(
        subtractAngle(
            np.arctan2(pose2.p[1] - pose1.p[1], pose2.p[0] - pose1.p[0]),
            pose1.theta)) < angle / 2


def distance(pose1, pose2):
    # Distance from poses
    return np.hypot(pose1.p[0] - pose2.p[0], pose1.p[1] - pose2.p[1])


def limitAngle(angle):
    # Keep angle between -pi and pi
    return (angle + np.pi) % (2 * np.pi) - np.pi


def subtractAngle(angle1, angle2):
    # Select the difference that is less than pi
    diff = angle1 - angle2
    if abs(diff) > np.pi:
        return limitAngle(2 * np.pi - diff)
    return diff
