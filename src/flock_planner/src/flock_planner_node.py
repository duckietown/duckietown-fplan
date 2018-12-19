#!/usr/bin/env python

import rospy
import dispatcher
import duckietown_world as dw
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose2D
from flock_simulator.msg import FlockState, FlockCommand, DuckieCommand


class FlockPlannerNode(object):
    def __init__(self, map_name):
        self.node_name = rospy.get_name()

        # Map
        self.map = dw.load_map(map_name)
        self.skeleton_graph = dw.get_skeleton_graph(self.map['tilemap'])

        # Dispatcher
        self.dispatcher = dispatcher.Dispatcher(self.skeleton_graph)
        self.state = {'seq': 0, 'duckies': {}, 'requests': []}

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
        msg_commands = self.generateMessages(self.dispatcher.commands)
        self.pub_commands.publish(msg_commands)

    def generateMessages(self, commands):
        msg = FlockCommand()
        msg.header.stamp = rospy.Time.now()
        msg.dt.data = 1.0 / self.sim_frequency
        for command in commands:
            command_msg = DuckieCommand()
            command_msg.duckie_id = String(data=command['duckie_id'])
            command_msg.request_id = String(data=command['request_id'])
            command_msg.on_rails = Bool(data=True)
            command_msg.path = []
            for node in command['path']:
                command_msg.path.append(String(data=node))
            msg.duckie_commands.append(command_msg)
        return msg

    def getStateFromMessage(self, msg):
        state = {'seq': msg.header.seq, 'duckies': {}, 'requests': {}}
        for duckie in msg.duckie_states:
            state['duckies'][duckie.duckie_id.data] = {
                'status': duckie.status.data,
                'lane': duckie.lane.data
            }
        for request in msg.requests:
            state['requests'][request.request_id.data] = {
                'time': request.start_time.data,
                'duckie_id': request.duckie_id.data,
                'start_node': request.start_node.data,
                'end_node': request.end_node.data
            }
        return state

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node(
        'flock_planner_node', anonymous=False, log_level=rospy.DEBUG)
    map_name = rospy.get_param('~map_name')
    flock_planner_node = FlockPlannerNode(map_name)
    rospy.on_shutdown(flock_planner_node.onShutdown)
    rospy.spin()