#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class DuckieDynamicsNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Subscribers
        self.sub_commands = rospy.Subscriber(
            "~commands", String, self.cbCommands, queue_size=1)

        # Publishers
        self.pub_locations = rospy.Publisher(
            "~locations", String, queue_size=1)

        # other stuff

    def updateLocations(self, commands):
        # Calculate new locations
        locations = ['locationdata']
        return locations

    def cbCommands(self, data):
        commands = data
        locations = self.updateLocations(commands)
        msg_locations = String()
        msg_locations.data = locations
        self.pub_locations.publish(msg_locations)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node("duckie_dynamics_node", anonymous=False)
    duckie_dynamics_node = DuckieDynamicsNode()
    rospy.on_shutdown(duckie_dynamics_node.onShutdown)
    rospy.spin()
