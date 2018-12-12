#!/usr/bin/env python

import rospy
import tf
import state_manager
from std_msgs.msg import String, Bool, Int8
from geometry_msgs.msg import Pose2D, Twist, Vector3
from flock_simulator.msg import FlockState, FlockCommand, DuckieState


class FlockSimulatorNode(object):
    def __init__(self, map_name, n_duckies):
        self.node_name = rospy.get_name()

        self.state_manager = state_manager.StateManager(map_name, n_duckies)

        # Subscribers
        self.sub_paths = rospy.Subscriber(
            '/flock_simulator/commands',
            FlockCommand,
            self.cbCommands,
            queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher(
            '/flock_simulator/state', FlockState, queue_size=1)
        self.msg_state = FlockState()

        self.isUpdating = False

    def cbCommands(self, msg):
        # Return same state if last callback has not finished
        if self.isUpdating:
            rospy.logwarn(
                'State not finished updating. Publishing previous state again.'
            )
            self.pub_state.publish(self.msg_state)
            return

        # Update state
        self.isUpdating = True
        dt = msg.dt.data
        commands = self.getCommands(msg)
        self.state_manager.updateState(commands, dt)
        self.isUpdating = False

        # Publish
        self.msg_state = self.generateFlockStateMsg(self.state_manager.duckies)
        self.pub_state.publish(self.msg_state)
        self.publishTf(self.state_manager.duckies)

    def getCommands(self, msg):
        commands = {}
        for command in msg.duckie_commands:
            on_rails = command.on_rails.data
            if on_rails:
                commands[command.duckie_id.data] = {
                    'goal_node': command.node_command.data,
                    'on_rails': on_rails
                }
            else:
                commands[command.duckie_id.data] = {
                    'linear': command.velocity_command.linear.x,
                    'angular': command.velocity_command.angular.z,
                    'on_rails': on_rails
                }
        return commands

    def generateFlockStateMsg(self, duckies):
        msg = FlockState()
        msg.header.stamp = rospy.Time.now()
        for duckie_id in duckies:
            duckie = duckies[duckie_id]
            duckiestate_msg = DuckieState()
            duckiestate_msg.duckie_id = String(data=duckie_id)
            duckiestate_msg.status = String(data=duckie['status'])
            duckiestate_msg.lane = String(data=duckie['next_point']['lane'])
            duckiestate_msg.pose = Pose2D(
                x=duckie['pose'].p[0] * self.state_manager.map.tile_size,
                y=duckie['pose'].p[1] * self.state_manager.map.tile_size,
                theta=duckie['pose'].theta)
            duckiestate_msg.velocity = Twist(
                linear=Vector3(duckie['velocity']['linear'], 0, 0),
                angular=Vector3(0, 0, duckie['velocity']['angular']))
            duckiestate_msg.in_fov = [
                String(data=visible_duckie)
                for visible_duckie in duckie['in_fov']
            ]
            duckiestate_msg.collision_level = Int8(
                data=duckie['collision_level'])
            msg.duckie_states.append(duckiestate_msg)
        return msg

    def publishTf(self, duckies):
        for duckie_id in duckies:
            duckie = duckies[duckie_id]
            stamp = rospy.Time.now()
            theta = duckie['pose'].theta
            x = duckie['pose'].p[0] * self.state_manager.map.tile_size
            y = duckie['pose'].p[1] * self.state_manager.map.tile_size

            transform_broadcaster.sendTransform((x, y, 0), \
                tf.transformations.quaternion_from_euler(0, 0, theta), \
                stamp, duckie_id, "duckiebot_link")

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('flock_simulator_node', anonymous=False)
    map_name = rospy.get_param('~map_name')
    n_duckies = rospy.get_param('~n_duckies')
    transform_broadcaster = tf.TransformBroadcaster()
    flock_simulator_node = FlockSimulatorNode(map_name, n_duckies)
    rospy.on_shutdown(flock_simulator_node.onShutdown)
    rospy.spin()
