#!/usr/bin/env python

import rospy
import time  # TODO
from flock_simulator.msg import FlockState, FlockCommand


class FlockPlannerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Subscribers
        self.sub_paths = rospy.Subscriber(
            '/flock_simulator_node/flock_state',
            FlockState,
            self.cbState,
            queue_size=1)

        # Publishers
        self.pub_commands = rospy.Publisher(
            '/flock_simulator_node/flock_commands', FlockCommand, queue_size=1)

        # Timer
        self.sim_frequency = 20.0  # Frequency of simulation in Hz
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.sim_frequency), self.cbTimer)
        self.isUpdating = False

    def cbState(self, msg):
        pass

    def cbTimer(self, event):
        # Don't update if last timer callback hasn't finished
        if self.isUpdating:
            rospy.logwarn('Dispatcher not ready. Skipping timestep.')
            return

        # Update state
        self.isUpdating = True
        time.sleep(.01)
        self.isUpdating = False

        # Publish
        msg_commands = self.generateCommands()
        self.pub_commands.publish(msg_commands)

    def generateCommands(self):
        msg = FlockCommand()
        msg.header.stamp = rospy.Time.now()
        return msg

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node(
        'flock_planner_node', anonymous=False, log_level=rospy.DEBUG)
    flock_planner_node = FlockPlannerNode()
    rospy.on_shutdown(flock_planner_node.onShutdown)
    rospy.spin()