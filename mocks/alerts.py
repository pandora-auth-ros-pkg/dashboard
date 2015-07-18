#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import time
import random

from geometry_msgs.msg import PoseStamped

from pandora_audio_msgs.msg import SoundAlertVector
from pandora_audio_msgs.msg import SoundAlert

from pandora_data_fusion_msgs.msg import QrInfo

from pandora_vision_msgs.msg import HazmatAlertVector
from pandora_vision_msgs.msg import HazmatAlert

from pandora_vision_msgs.msg import ThermalAlertVector
from pandora_vision_msgs.msg import ThermalAlert

from pandora_common_msgs.msg import GeneralAlertVector
from pandora_common_msgs.msg import GeneralAlertInfo


def qr_alert(delay):
    pub = rospy.Publisher('/data_fusion/qr_info', QrInfo)
    msg = QrInfo()

    while not rospy.is_shutdown():
        print('Sending QR alert...')
        msg.id = random.randint(0, 10)
        msg.qrFrameId = 'kinect_frame'
        msg.timeFound = rospy.Time.now()
        msg.qrPose = PoseStamped()
        msg.probability = random.random()
        msg.content = 'qr content'

        pub.publish(msg)
        time.sleep(delay)


def hazmat_alert(delay=1):
    pub = rospy.Publisher('/vision/hazmat_alert', HazmatAlertVector)
    msg = HazmatAlertVector()

    while not rospy.is_shutdown():
        alert = HazmatAlert()
        alert.patternType = random.randint(0, 3)
        alert.info.probability = random.random()

        msg.alerts = [alert]

        pub.publish(msg)
        time.sleep(delay)


def thermal_alert(delay=1):
    pub = rospy.Publisher('/vision/thermal_direction_alert',
                          ThermalAlertVector)
    msg = ThermalAlertVector()

    while not rospy.is_shutdown():
        alert = ThermalAlert()
        alert.temperature = random.random() * 40
        alert.info.probability = random.random()
        msg.alerts = [alert]

        pub.publish(msg)
        time.sleep(delay)


def sound_alert(delay=1):
    pub = rospy.Publisher('/sound/complete_alert', SoundAlertVector)
    msg = SoundAlertVector()

    while not rospy.is_shutdown():
        alert = SoundAlert()
        alert.word = str(random.randint(0, 9))
        alert.info.probability = random.random()
        msg.alerts = [alert]

        pub.publish(msg)
        time.sleep(delay)


def visual_alert(delay=1):
    pub = rospy.Publisher('/vision/victim_direction_alert', GeneralAlertVector)
    msg = GeneralAlertVector()

    while not rospy.is_shutdown():
        alert = GeneralAlertInfo()
        alert.probability = random.random()
        msg.alerts = [alert]

        pub.publish(msg)
        time.sleep(delay)


def co2_alert(delay=1):
    pub = rospy.Publisher('/sensor_processing/co2_alert', GeneralAlertVector)
    msg = GeneralAlertVector()

    while not rospy.is_shutdown():
        alert = GeneralAlertInfo()
        alert.probability = random.random()
        msg.alerts = [alert]

        pub.publish(msg)
        time.sleep(delay)


def motion_alert(delay=1):
    pub = rospy.Publisher('/vision/motion_alert', GeneralAlertVector)
    msg = GeneralAlertVector()

    while not rospy.is_shutdown():
        alert = GeneralAlertInfo()
        alert.probability = random.random()
        msg.alerts = [alert]

        pub.publish(msg)
        time.sleep(delay)


if __name__ == '__main__':

    rospy.init_node('alert_mocks')

    alert = sys.argv[1]
    delay = float(sys.argv[2])

    if alert == 'visual':
        visual_alert(delay)
    elif alert == 'sound':
        sound_alert(delay)
    elif alert == 'co2':
        co2_alert(delay)
    elif alert == 'hazmat':
        hazmat_alert(delay)
    elif alert == 'thermal':
        thermal_alert(delay)
    elif alert == 'motion':
        motion_alert(delay)
