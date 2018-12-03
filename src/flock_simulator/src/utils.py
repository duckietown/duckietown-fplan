import numpy as np


def nodeFromLane(edges, lane):
    edge_data = [edge for edge in edges if edge[2]['lane'] == lane]
    return edge_data[0][1]


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
