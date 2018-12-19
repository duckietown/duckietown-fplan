import random
import numpy as np
import networkx as nx


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


def isInFront(pose1, pose2, angle):
    # frame1: Own pose
    # frame2: Object's pose
    # angle: Angle of cone that defines "front" ("field of view")
    return abs(
        subtractAngle(
            np.arctan2(pose2.p[1] - pose1.p[1], pose2.p[0] - pose1.p[0]),
            pose1.theta)) < angle / 2
