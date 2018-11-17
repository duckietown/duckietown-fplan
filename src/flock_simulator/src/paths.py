import random


def generateAllPaths(duckies, nodes):
    paths = {}
    for duckie in duckies:
        paths[duckie] = generatePath(duckies, nodes, duckie, None)
    return paths


def generatePath(duckies, nodes, duckie, path):
    pose = duckies[duckie]['pose']
    heading = duckies[duckie]['heading']
    current_pos = [round(pose['x']), round(pose['y'])]
    if path:
        path_start = path[0]
    else:
        path_start = [current_pos[0] + heading[0], current_pos[1] + heading[1]]
    path_node1 = current_pos
    while path_node1 == current_pos:
        random_direction = random.choice(
            nodes['tile-%d-%d' % (path_start[0], path_start[1])]['exits'])
        path_node1 = [
            path_start[0] + random_direction[0],
            path_start[1] + random_direction[1]
        ]
    path_node2 = path_start
    while path_node2 == path_start:
        random_direction = random.choice(
            nodes['tile-%d-%d' % (path_node1[0], path_node1[1])]['exits'])
        path_node2 = [
            path_node1[0] + random_direction[0],
            path_node1[1] + random_direction[1]
        ]

    return [path_start, path_node1, path_node2]
