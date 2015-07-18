#!/usr/bin/env python

import rospy
import random

from pandora_data_fusion_msgs.srv import GetVictimProbabilities, GetVictimProbabilitiesResponse
from pandora_data_fusion_msgs.msg import VictimProbabilities


def send_random_victim_probabilities(req):
    print 'Sending random probabilities'

    sensors = ['co2', 'thermal', 'visual']
    probabilities = VictimProbabilities()
    probabilities.co2 = random.random()
    probabilities.thermal = random.random()
    probabilities.sound = random.random()
    probabilities.motion = random.random()
    probabilities.hazmat = random.random()
    probabilities.visualVictim = random.random()

    return GetVictimProbabilitiesResponse(success=True, sensors=sensors,
                                          probabilities=probabilities)


def start_service():
    rospy.init_node("data_fusion")
    rospy.Service('/data_fusion/get_probabilities', GetVictimProbabilities,
                  send_random_victim_probabilities)
    print 'Service initialized'
    rospy.spin()


if __name__ == '__main__':
    start_service()
