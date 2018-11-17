import random
import numpy as np


def getNewPose(pose, distance, tile, action, heading):
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


def setNewPose(duckie, path, distance):
    pose = duckie['pose']
    action = duckie['action']
    heading = duckie['heading']
    tile = np.around([pose['x'], pose['y']]).tolist()
    pose_new = getNewPose(pose, distance, tile, action, heading)
    tile_new = np.around([pose_new['x'], pose_new['y']]).tolist()

    if tile_new == tile:
        duckie['pose'] = pose_new
        return

    heading_new = getNewHeading(path)
    action_new = getNewAction(heading, heading_new)

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

    pose_new = getNewPose(pose_trans, distance - distance_over, tile_new,
                          action_new, heading_new)

    duckie['pose'] = pose_new
    duckie['action'] = action_new
    duckie['heading'] = heading_new


def getDesiredVelocity(duckies, nodes, path, max_vel, duckie):
    pose = duckies[duckie]['pose']
    tile = np.around([pose['x'], pose['y']]).tolist()
    heading = duckies[duckie]['heading']

    # Check for duckies in front
    current_tile = 'tile-%d-%d' % (tile[0], tile[1])
    front_tile = 'tile-%d-%d' % (tile[0] + heading[0], tile[1] + heading[1])
    current_tile_duckies = nodes[current_tile]['duckies']
    front_tile_duckies = nodes[front_tile]['duckies']
    both_tiles_duckies = current_tile_duckies + front_tile_duckies
    if both_tiles_duckies:
        for tile_duckie in both_tiles_duckies:
            diff = [
                duckies[tile_duckie]['pose']['x'] - pose['x'],
                duckies[tile_duckie]['pose']['y'] - pose['y']
            ]
            distance_in_front = heading[0] * diff[0] + heading[1] * diff[1]
            if np.linalg.norm(diff) < 0.5 and distance_in_front > 0:
                return 0

    # Check for right of way
    on_intersection = nodes[current_tile]['type'] == 'intersection'
    next_tile_intersection = len(nodes[front_tile]['type']) == 'intersection'
    if on_intersection or not next_tile_intersection or not front_tile_duckies:
        return max_vel

    heading_next = getNewHeading(path)
    action_next = getNewAction(heading, heading_next)
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
    right_tile_duckies = nodes[right_tile]['duckies']
    if right_tile_duckies:
        right_duckie_heading = [-right_heading[0], -right_heading[1]]
        for right_duckie in right_tile_duckies:
            diff_from_mid = [
                right_duckie['pose']['x'] - right_coord[0],
                right_duckie['pose']['y'] - right_coord[1]
            ]
            dist_from_mid = right_duckie_heading[0] * diff_from_mid[0] + right_duckie_heading[1] * diff_from_mid[1]
            if right_duckie['heading'] == right_duckie_heading and dist_from_mid > 0:
                return 0

    return max_vel


def getNewHeading(path):
    return [b - a for a, b in zip(path[0], path[1])]


def getNewAction(heading, heading_new):
    heading_cross = np.cross(heading.append(0), heading_new.append(0))
    if heading_cross[2] == 0:
        return 'straight'
    elif heading_cross[2] > 0:
        return 'curve_left'
    else:
        return 'curve_right'


def spawnDuckies(n, map_graph):
    duckies = {}
    offset = {
        '[0, 1]': [0.25, 0, 0],
        '[1, 0]': [0, -0.25, -1.0 / 2.0 * np.pi],
        '[0, -1]': [-0.25, 0, np.pi],
        '[-1, 0]': [0, 0.25, 1.0 / 2.0 * np.pi]
    }
    for i in range(n):
        spawnIsValid = False
        while not spawnIsValid:
            spawn_node = random.choice([
                node for node, data in map_graph.nodes(data=True)
                if data['type'] == 'straight'
            ])
            direction = random.choice(map_graph.nodes[spawn_node]['exits'])
            pose = {
                'x':
                map_graph.nodes[spawn_node]['coord'][0] +
                offset[str(direction)][0],
                'y':
                map_graph.nodes[spawn_node]['coord'][1] +
                offset[str(direction)][1],
                'phi':
                offset[str(direction)][2]
            }
            coord = [pose['x'], pose['y']]
            spawnIsValid = True
            for duckie in duckies:
                duckie_coord = [
                    duckies[duckie]['pose']['x'], duckies[duckie]['pose']['y']
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
        map_graph.nodes[spawn_node]['duckies'].append('duckie-%d' % i)
    return duckies
