#!/usr/bin/env python

import rospy
import state_manager
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose2D, Twist
from flock_simulator.msg import FlockState, FlockCommand, DuckieState


class FlockSimulatorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.state_manager = state_manager.StateManager()

        # Parameters
        self.sim_frequency = 20.0  # Frequency of simulation in Hz
        self.dt = 1.0 / self.sim_frequency

        # Subscribers
        self.sub_paths = rospy.Subscriber(
            '~commands', FlockCommand, self.cbCommands, queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher('~state', FlockState, queue_size=1)

        # Timer
        self.isUpdating = False
        self.request_timer = rospy.Timer(
            rospy.Duration.from_sec(self.dt), self.cbTimer)

    def cbCommands(self, data):
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

    def generateFlockStateMsg(self, duckies):
        msg = FlockState()
        msg.header.stamp = rospy.Time.now()
        for duckie in duckies:
            duckiestate_msg = DuckieState()
            duckiestate_msg.duckie_id = String(data=duckie)
            duckiestate_msg.on_service = Bool(
                data=duckies[duckie]['on_service'])
            duckiestate_msg.pose = Pose2D(
                x=duckies[duckie]['pose']['x'],
                y=duckies[duckie]['pose']['y'],
                theta=duckies[duckie]['pose']['phi'])
            msg.duckie_states.append(duckiestate_msg)
        return msg

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('flock_simulator_node', anonymous=False)
    flock_simulator_node = FlockSimulatorNode()
    rospy.on_shutdown(flock_simulator_node.onShutdown)
    rospy.spin()