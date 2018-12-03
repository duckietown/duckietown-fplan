#!/usr/bin/env python

import rospy
import time  # TODO
import dispatcher
from flock_simulator.msg import FlockState, FlockCommand


class FlockPlannerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Dispatcher
        self.dispatcher = dispatcher.Dispatcher()
        self.state = []

        # Subscribers
        self.sub_paths = rospy.Subscriber(
            '/flock_simulator/state', FlockState, self.cbState, queue_size=1)

        # Publishers
        self.pub_commands = rospy.Publisher(
            '/flock_simulator/commands', FlockCommand, queue_size=1)

        # Timer
        self.sim_frequency = 30.0  # Frequency of simulation in Hz
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.sim_frequency), self.cbTimer)
        self.isUpdating = False

    def cbState(self, msg):
        self.state = self.getStateFromMessage(msg)

    def cbTimer(self, event):
        # Don't update if last timer callback hasn't finished
        if self.isUpdating:
            rospy.logwarn('Dispatcher not ready. Skipping timestep.')
            return

        # Update state
        self.isUpdating = True
        self.dispatcher.update(self.state)
        self.isUpdating = False

        # Publish
        msg_commands = self.generateCommands(self.dispatcher.commands)
        self.pub_commands.publish(msg_commands)

    def generateCommands(self, commands):
        msg = FlockCommand()
        msg.header.stamp = rospy.Time.now()
        msg.dt.data = 1.0 / self.sim_frequency
        return msg

    def getStateFromMessage(self, msg):
        pass

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node(
        'flock_planner_node', anonymous=False, log_level=rospy.DEBUG)
    flock_planner_node = FlockPlannerNode()
    rospy.on_shutdown(flock_planner_node.onShutdown)
    rospy.spin()