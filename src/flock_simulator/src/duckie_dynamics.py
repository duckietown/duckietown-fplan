import random
import numpy as np
import duckietown_world as dw


def getNextPoint(skeleton_graph, current_pose, current_point):
    lane_control_points = skeleton_graph.root2.children[current_point[
        'lane']].control_points
    current_point_pose = current_point['pose']

    # If current_point in front, keep it as next_point
    if abs(
            np.arctan2(current_point_pose.p[1] - current_pose.p[1],
                       current_point_pose.p[0] - current_pose.p[0]) -
            current_pose.theta) < np.pi / 2:
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


def getRandomCommand(duckie):
    if not duckie['next_point']:
        return {'linear': 0, 'angular': 0}

    duckie_pose = duckie['pose']
    point_pose = duckie['next_point']['pose']
    lin_diff = np.hypot(point_pose.p[0] - duckie_pose.p[0],
                        point_pose.p[1] - duckie_pose.p[1])
    ang_diff = point_pose.theta - duckie_pose.theta

    linear = 0.2

    if ang_diff > 0.02:
        # Some geometry
        radius = lin_diff / (2 * np.sin(ang_diff / 2))
        angular = linear / radius
    else:
        angular = 0

    return {'linear': linear, 'angular': angular}


def updateDuckie(duckies, duckie, command, skeleton_graph, tile_size, dt):
    pose_current = duckie['pose']
    theta_new = pose_current.theta + command['angular'] * dt
    theta_avg = (pose_current.theta + theta_new) / 2
    position_new = [
        pose_current.p[0] +
        (command['linear'] / tile_size) * np.cos(theta_avg) * dt,
        pose_current.p[1] +
        (command['linear'] / tile_size) * np.sin(theta_avg) * dt
    ]  # Small angle approximation
    pose_new = dw.SE2Transform(position_new, theta_new)
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
                    'on_service': False
                }
                spawn_is_occupied = False
                occupied_lanes.append(lane)
    return duckies
