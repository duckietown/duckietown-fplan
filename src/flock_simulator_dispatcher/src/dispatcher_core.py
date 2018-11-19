import random
import numpy as np

def getclosestresquest(requests, pose):
    closestRequestId = np.find_nearest(requests.locations, pose)
    return costumerlocation = requests(closestRequestId).start


def dispatcher(requests, duckies):

    for duckies in range(duckie,duckies):
        #IDLE
        if duckie.status == 'IDLE':

            #picking costumer up
            if costumerlocation == pose:
                duckie.status == 'WITH_COSTUMER'
            #going to costumer
            else:
                costumerlocation = getclosestrequest(requests, duckies.id.pose)

        #WREBALANCE TODO LATER
        #elif duckie.status == 'REBALANCE_DRIVE'
            #targetlocation = getRebalance(requests, duckies.id.pose)

        #WITH COSTUMER
        elif duckie.status == 'WITH_COSTUMER':
            targetlocation = duckie.request.endlocation

    return duckies #with end location


