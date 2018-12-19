import random
import numpy as np


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


def isInFront(pose1, pose2, angle):
    # frame1: Own pose
    # frame2: Object's pose
    # angle: Angle of cone that defines "front" ("field of view")
    return abs(
        subtractAngle(
            np.arctan2(pose2.p[1] - pose1.p[1], pose2.p[0] - pose1.p[0]),
            pose1.theta)) < angle / 2
