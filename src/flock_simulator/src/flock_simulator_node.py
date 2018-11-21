#!/usr/bin/env python

import rospy
import state_manager
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose2D
from flock_simulator.msg import FlockState, Fleetplan


class FlockSimulatorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.state_manager = state_manager.StateManager()

        # Parameters
        self.sim_frequency = 20.0  # Frequency of simulation in Hz
        self.dt = 1.0 / self.sim_frequency

        # Subscribers
        self.sub_paths = rospy.Subscriber(
            '~paths', Fleetplan, self.cbPaths, queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher('~state', FlockState, queue_size=1)

        # Timer
        self.isUpdating = False
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(self.dt), self.cbTimer)

    def generateFlockStateMsg(self, duckies):
        msg = FlockState()
        msg.header.stamp = rospy.Time.now()
        for duckie in duckies:
            msg.location.append(
                Pose2D(duckies[duckie]['pose']['x'],
                       duckies[duckie]['pose']['y'],
                       duckies[duckie]['pose']['phi']))
            msg.on_service.append(Bool(duckies[duckie]['on_service']))
            msg.duckie_id.append(String(duckie))
        return msg

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
        msg_state = self.generateFlockStateMsg(self.state_manager.duckies)
        self.pub_state.publish(msg_state)

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('flock_simulator_node', anonymous=False)
    flock_simulator_node = FlockSimulatorNode()
    rospy.on_shutdown(flock_simulator_node.onShutdown)
    rospy.spin()