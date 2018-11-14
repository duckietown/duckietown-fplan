#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class StateManagerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters
        self.sim_frequency = 20  # Frequency of simulation in Hz

        # Subscribers
        self.sub_requests = rospy.Subscriber(
            "~requests", String, self.cbRequests, queue_size=1)
        self.sub_locations = rospy.Subscriber(
            "~locations", String, self.cbLocations, queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher("~state", String, queue_size=1)

        # Timer
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.sim_frequency), self.cbTimer)

        # other stuff

    def updateState(self, event):
        # Update state

        # Publish
        msg_state = String()
        self.pub_state.publish(msg_state)

    def cbRequests(self, data):
        # Do something with requests
        print(data)

    def cbLocations(self, data):
        # Do something with locations
        print(data)

    def cbTimer(self, event):
        self.updateState

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node("state_manager_node", anonymous=False)
    state_manager_node = StateManagerNode()
    rospy.on_shutdown(state_manager_node.onShutdown)
    rospy.spin()
