#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class RequestGeneratorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters
        self.request_rate = 10  # Seconds per request

        # Publishers
        self.pub_requests = rospy.Publisher("~requests", String, queue_size=1)

        # Timer
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(self.request_rate), self.cbTimer)

        # other stuff

    def generateRequests(self):
        # Generate random requests
        msg_requests = String()
        self.pub_requests.publish(msg_requests)

    def cbTimer(self, event):
        self.generateRequests

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node("request_generator_node", anonymous=False)
    request_generator_node = RequestGeneratorNode()
    rospy.on_shutdown(request_generator_node.onShutdown)
    rospy.spin()
