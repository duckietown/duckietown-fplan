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

        # Subscribers
        self.sub_paths = rospy.Subscriber(
            '~flock_commands', FlockCommand, self.cbCommands, queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher(
            '~flock_state', FlockState, queue_size=1)

    def cbCommands(self, msg):
        # Update state
        commands = self.getCommands(msg)
        self.state_manager.updateState(commands, msg.dt)

        # Publish
        msg_state = self.generateFlockStateMsg(self.state_manager.duckies)
        self.pub_state.publish(msg_state)

    def getCommands(self, msg):
        commands = []
        for command in msg.duckie_commands:
            commands.append({
                'duckie_id': command.duckie_id,
                'command': {
                    'linear': command.linear.x,
                    'angular': command.angular.z
                }
            })
        return commands

    def generateFlockStateMsg(self, duckies):
        msg = FlockState()
        msg.header.stamp = rospy.Time.now()
        for duckie_id in duckies:
            duckiestate_msg = DuckieState()
            duckiestate_msg.duckie_id = String(data=duckie_id)
            duckiestate_msg.on_service = Bool(
                data=duckies[duckie_id]['on_service'])
            duckiestate_msg.pose = Pose2D(
                x=duckies[duckie_id]['pose']['x'],
                y=duckies[duckie_id]['pose']['y'],
                theta=duckies[duckie_id]['pose']['theta'])
            duckiestate_msg.velocity = Twist(
                linear=[duckies[duckie_id]['velocity']['linear'], 0, 0],
                angular=[0, 0, duckies[duckie_id]['velocity']['angular']])
            msg.duckie_states.append(duckiestate_msg)
        return msg

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('flock_simulator_node', anonymous=False)
    flock_simulator_node = FlockSimulatorNode()
    rospy.on_shutdown(flock_simulator_node.onShutdown)
    rospy.spin()