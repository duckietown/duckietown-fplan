import random

# TODO Implement duckie initial(t=0) status = 'IDLE'


class Dispatcher(object):
    def __init__(self):
        self.commands = []

    def update(self, state):
        duckies = state.duckies
        open_requests = state.open_requests

        for duckie in duckies:
            pose = duckies[duckie]['pose']
            status = duckies[duckie]['status']
            target_location

            #IDLE
            if status == 'IDLE':

                #find new costumer
                open_request = self.getClosestRequest(self, open_requests, pose)
                start_loc = open_request['startlocation']
                end_loc = open_request['endlocation']

                #pick up costumer
                if start_loc == pose:
                    #change status AND delete open_request
                    status == 'WITH_COSTUMER'
                    target_location = end_loc  #put to duckietarget location
                    open_requests.remove(open_request)

                #going to costumer pickup costumer
                elif start_loc != pose:
                    status == 'GOINGTO_COSTUMER'
                    target_location = start_loc  #put to duckietarget location

                #no requests TODO REBALANCE
                else:
                    status == 'IDLE'
                    target_location = pose

            #WITH COSTUMER
            elif duckie.status == 'WITH_COSTUMER':

                #endlocation reached, request fullfilled
                if pose == target_location:
                    duckie.status == 'IDLE'
                    target_location = 0

        return self.generateCommands(self, pose, target_location)

    def generateCommants(self, pose, target_location):
        msg = FlockCommand()
        msg.header.stamp = rospy.Time.now()



    def getClosestRequest(self, open_requests, pose):
        if open_requests == 0:
            return 0  #todo check this case
        closest_request = open_requests[0]
        for open_request in open_requests:
            if self.dist(pose, open_request) < self.dist(pose, closest_request):
                closest_request = open_request
            return closest_request

    def dist(self, pose, request):
        # TODO DISTANCE REQUEST TO POSE
        # to measure with rail distances
        return 10
