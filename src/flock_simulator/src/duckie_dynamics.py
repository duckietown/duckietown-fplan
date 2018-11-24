import random
import numpy as np
import duckietown_world as dw


def getRandomCommand(map, duckie, dt):
    linear = 0
    angular = 0
    return {'linear': linear, 'angular': angular}


def updateDuckie(duckies, duckie, command, tile_size, dt):
    pose_current = duckie['pose']
    theta_new = pose_current.theta + command['angular'] * dt
    theta_avg = (pose_current.theta + theta_new) / 2
    pose_new = [
        pose_current.p[0] +
        tile_size * command['linear'] * np.cos(theta_avg) * dt,
        pose_current.p[1] +
        tile_size * command['linear'] * np.sin(theta_avg) * dt
    ]  # Small angle approximation
    velocity_new = command
    return {
        'pose': dw.SE2Transform(pose_new, theta_new),
        'velocity': velocity_new,
        'on_service': False
    }


def spawnDuckies(n, skeleton_graph):
    duckies = {}
    occupied_lanes = []
    for i in range(n):
        spawn_is_occupied = True
        while spawn_is_occupied:
            lane = random.sample(skeleton_graph.root.children, 1)
            if lane not in occupied_lanes:
                pose = random.choice(
                    skeleton_graph.root.children['ls055'].control_points)
                duckies['duckie-%d' % i] = {
                    'pose': pose,
                    'velocity': {
                        'linear': 0,
                        'angular': 0
                    },
                    'on_service': False
                }
                spawn_is_occupied = False
                occupied_lanes.append(lane)
    return duckies
