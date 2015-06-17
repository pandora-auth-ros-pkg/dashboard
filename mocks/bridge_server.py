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

        self.response_receiver = self.context.socket(zmq.SUB)
        self.response_receiver.connect("tcp://127.0.0.1:7777")
        self.response_receiver.setsockopt(zmq.SUBSCRIBE, 'victim_validation')

        self.alert_publisher = self.context.socket(zmq.PUB)
        self.alert_publisher.bind('tcp://*:6666')

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
        self.alert_publisher.send('%s %s' % ('victim_goal', self.current_goal))
        print('Checking if the operator responded...')
        print(self.operator_responded.is_set())
        self.operator_responded.wait()

        # Send the response to the client.
        self.result.victimValid = self.operator_response
        self.action.set_succeeded(self.result)

        # Reset the environment.
        self.operator_responded.clear()

    def start(self):
        # Start the action server.
        rospy.init_node(self.name)
        self.action.start()

        while True:
            self.operator_responded.clear()
            print('Waiting for message')
            res = self.response_receiver.recv()
            print(res)
            self.operator_response = True if res == 'true' else False
            self.operator_responded.set()


if __name__ == '__main__':
    server = BridgeServer('/gui/validate_victim', 'bridge_server')
    server.start()
