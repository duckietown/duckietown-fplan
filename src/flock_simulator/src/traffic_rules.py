import utils
import numpy as np


def getVelocity(duckies, duckie, stop_distance, duckiebot_length, max_vel,
                tile_size, skeleton_graph):
    # Distance to position where duckie wants to stand still
    waypoint_distance = 2 * tile_size

    # Check for duckies in front
    for visible_duckie in duckie['in_fov']:
        distance_to_duckie = utils.distance(
            duckie['pose'], duckies[visible_duckie]['pose']) * tile_size
        if isInPath(duckie['pose'], duckies[visible_duckie]['pose'],
                    2 * duckiebot_length, tile_size / 2, tile_size):
            waypoint_distance = min(
                distance_to_duckie - stop_distance - duckiebot_length,
                waypoint_distance)

    # If approaching intersection
    next_point = duckie['next_point']
    node = utils.nodeFromLane(
        list(skeleton_graph.G.edges(data=True)), next_point['lane'])

    if len(list(skeleton_graph.G.neighbors(
            node))) > 1 and next_point['point_index'] == len(
                skeleton_graph.root2.children[next_point['lane']]
                .control_points) - 1:

        for visible_duckie_id in duckie['in_fov']:
            visible_duckie = duckies[visible_duckie_id]

            # Check for duckies on intersection
            next_tile = [
                np.floor(
                    duckie['pose'].p[0] + np.cos(duckie['pose'].theta) * 1.0),
                np.floor(
                    duckie['pose'].p[1] + np.sin(duckie['pose'].theta) * 1.0)
            ]
            visible_duckie_tile = [
                np.floor(visible_duckie['pose'].p[0]),
                np.floor(visible_duckie['pose'].p[1])
            ]
            duckie_on_intersection = next_tile == visible_duckie_tile

            # Check for right of way
            angle_to_duckie = utils.subtractAngle(
                np.arctan2(visible_duckie['pose'].p[1] - duckie['pose'].p[1],
                           visible_duckie['pose'].p[0] - duckie['pose'].p[0]),
                duckie['pose'].theta)
            on_right_side = angle_to_duckie < 0 and angle_to_duckie > -np.pi / 2

            duckie_is_close = utils.distance(
                duckie['pose'],
                visible_duckie['pose']) * tile_size < np.sqrt(2.0) * tile_size

            duckie_towards_intersection = np.round(
                utils.limitAngle(
                    visible_duckie['pose'].theta - duckie['pose'].theta),
                2) == np.round(np.pi / 2, 2)

            if duckie_on_intersection or (on_right_side and duckie_is_close
                                          and duckie_towards_intersection):
                # Stop on next point
                distance_to_point = utils.distance(duckie['pose'],
                                                   next_point['pose'])
                waypoint_distance = min(
                    distance_to_point - stop_distance - duckiebot_length / 2,
                    waypoint_distance)

    # Prevent waypoint to be negative
    waypoint_distance = max(waypoint_distance, 0.0)

    # Start braking if waypoint is 2 duckiebot_lengths away
    return min((waypoint_distance / (2 * duckiebot_length)) * max_vel, max_vel)


def isInPath(duckie_pose, obstacle_pose, width, length, tile_size):
    # Checks if obstacle is in rectangle in front of duckie

    # Check if close enough
    if utils.distance(duckie_pose, obstacle_pose) * tile_size > length:
        return False

    left_corner = [
        duckie_pose.p[0] + np.cos(duckie_pose.theta + np.pi / 2) * width / 2,
        duckie_pose.p[1] + np.sin(duckie_pose.theta + np.pi / 2) * width / 2
    ]
    right_corner = [
        duckie_pose.p[0] + np.cos(duckie_pose.theta - np.pi / 2) * width / 2,
        duckie_pose.p[1] + np.sin(duckie_pose.theta - np.pi / 2) * width / 2
    ]

    within_left_border = utils.subtractAngle(
        np.arctan2(obstacle_pose.p[1] - left_corner[1],
                   obstacle_pose.p[0] - left_corner[0]), duckie_pose.theta) < 0

    within_right_border = utils.subtractAngle(
        np.arctan2(obstacle_pose.p[1] - right_corner[1],
                   obstacle_pose.p[0] - right_corner[0]),
        duckie_pose.theta) > 0

    return within_left_border and within_right_border
