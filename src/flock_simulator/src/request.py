import random
import utils


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
        self.initialize(nodes)

    def initialize(self, nodes):
        self.start_node = random.choice(nodes)[0]
        self.end_node = self.start_node
        while self.end_node == self.start_node:
            self.end_node = random.choice(nodes)[0]

    def update(self, duckie, dt_map, timestep):
        if self.status == 'PICKEDUP':
            if duckie.status != 'DRIVINGWITHCUSTOMER':
                print('%s picked up %s but is not driving with customer!' %
                      (duckie.id, self.id))
                return
            pose_end = dt_map.nodeToPose(self.end_node)
            dist = utils.distance(duckie.pose, pose_end) * dt_map.tile_size
            if dist < duckie.length:
                print('%s has been dropped off.' % self.id)
                duckie.status = 'IDLE'
                self.status = 'FILLED'
                self.end_time = timestep
