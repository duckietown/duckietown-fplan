#!/usr/bin/env python

import rospy
import state_manager
from std_msgs.msg import String


class FlockSimulatorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.state_manager = state_manager.StateManager()

        # Parameters
        self.sim_frequency = 20.0  # Frequency of simulation in Hz
        self.dt = 1.0 / self.sim_frequency

        # Subscribers
        self.sub_paths = rospy.Subscriber(
            '~paths', String, self.cbPaths, queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher('~state', String, queue_size=1)

        # Timer
        self.isUpdating = False
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(self.dt), self.cbTimer)

    def cbPaths(self, data):
        # Check for validity
        # Generate path for invalid
        # Store paths in self.paths
        pass

    def cbTimer(self, event):
        # Don't update if last timer callback hasn't finished
        if self.isUpdating:
            rospy.logwarn('State not finished updating. Skipping timestep.')
            return

        # Update state
        self.isUpdating = True
        self.state_manager.updateState(self.dt)
        self.isUpdating = False

        # Publish
        msg_state = String()
        self.pub_state.publish(msg_state)

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('flock_simulator_node', anonymous=False)
    flock_simulator_node = FlockSimulatorNode()
    rospy.on_shutdown(flock_simulator_node.onShutdown)
    rospy.spin()