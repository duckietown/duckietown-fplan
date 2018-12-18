import random


class Request(object):
    def __init__(self, id, nodes, start_time):
        self.id = id
        self.start_time = start_time
        self.pickup_time = None
        self.end_time = None
        self.duckie_id = None
        self.status = 'WAITING'
        self.start_node = None
        self.end_node = None
        self.initializeNodes(nodes)

    def initializeNodes(self, nodes):
        self.start_node = random.choice(nodes)[0]
        self.end_node = self.start_node
        while self.end_node == self.start_node:
            self.end_node = random.choice(nodes)[0]