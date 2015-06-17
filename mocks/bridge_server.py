#!/usr/bin/env python

import threading
import json
import zmq
import rospy

from actionlib import SimpleActionServer as ActionServer

from pandora_gui_msgs.msg import ValidateVictimGUIAction
from pandora_gui_msgs.msg import ValidateVictimGUIResult


class BridgeServer(object):

    def __init__(self, ros_topic, name):

        # Action server configuration.
        self.name = name
        self.operator_responded = threading.Event()
        self.ros_topic = ros_topic
        self.result = ValidateVictimGUIResult()
        self.victim_found = False
        self.action = ActionServer(self.ros_topic, ValidateVictimGUIAction,
                                   execute_cb=self.handle_goals,
                                   auto_start=False)
        # ZMQ server configuration.
        self.context = zmq.Context()

        self.victim_sub = self.context.socket(zmq.SUB)
        self.victim_sub.connect("tcp://127.0.0.1:7777")

        self.victim_pub = self.context.socket(zmq.PUB)
        self.victim_pub.bind('tcp://*:6666')

    def handle_goals(self, goal):

        self.current_goal = json.dumps({
            "x": goal.victimFoundx,
            "y": goal.victimFoundy,
            "probability": goal.probability,
            "sensors": goal.sensorIDsFound})

        if self.action.is_preempt_requested():
            print('Preempting validate victim action')
            self.operator_responded.clear()
            self.action.set_preempted()

        # Send the goal to the subscribers.
        self.victim_pub.send('%s %s' % ('victim_goal', self.current_goal))
        #self.operator_responded.wait()
        self.result.victimValid = True
        self.action.set_succeeded(self.result)
        self.operator_responded.clear()

    def start(self):
        # Start the action server.
        rospy.init_node(self.name)
        self.action.start()

        while True:
            res = self.victim_sub.recv()
            if res:
                self.operator_responded.set()


if __name__ == '__main__':
    server = BridgeServer('/gui/validate_victim', 'bridge_server')
    server.start()
