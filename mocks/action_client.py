#!/usr/bin/env python

from __future__ import print_function
import rospy
from random import random, randint
from actionlib import SimpleActionClient as Client
from pandora_gui_msgs.msg import ValidateVictimGUIAction, ValidateVictimGUIGoal

topic = '/gui/validate_victim'

rospy.init_node('validation_client')

client = Client(topic, ValidateVictimGUIAction)
goal = ValidateVictimGUIGoal()
goal.victimId = randint(1, 10)
goal.victimFoundx = random() * 10
goal.victimFoundy = random() * 10
goal.probability = 0.6
goal.sensorIDsFound = ['so2', 'thermal']

print('Waiting for the GUI action server.')
client.wait_for_server()
print('sending..')
client.send_goal(goal)
print('Waiting for response.')
client.wait_for_result()

print(client.get_state())
print(client.get_result())
